// Full-horizon shared-pipeline evaluator: a drop-in replacement for
// evaluateFitnessAndDetectLocalBest (source/src/hls_pso.cpp), with the same signature and
// interface pragmas.
//
// The deployed evaluator calls System::nmpc_cost_function_topflow once per particle, serially.
// Here all _n_S particles share ONE pipelined model. For each horizon step i, and for each RK4
// stage s (an outer loop that is NOT flattened, so each stage drains), the particles are issued
// at initiation interval PROTO_II. The cost terms are accumulated in the same per-particle,
// per-component, step-by-step order as the original:
//   s == 0: one_step_u_error on the step-i control (Jui);
//   s == 3: update_state_final, then one_step_error on the new state (Ji), with Qf at i == N-1.
// Then J_error's sequential sums, then the local-best update. Every arithmetic expression is
// copied from hls_system.hpp / hls_pso.cpp; the type conversions and fixed-point updates are
// shared non-inlined functions (see rk4_multi.cpp for why).
#include "hls_pso.hpp"
#include "model_pipe.hpp"

#ifndef PROTO_II
#define PROTO_II 8
#endif

typedef _hw_top_real top_t;
typedef _hw_model_real model_t;

#define EV_S   _n_S
#define EV_NX  _Nx
#define EV_NU  _n_U
#define EV_NH  _Nh
#define EV_PART (_Nu * _n_U)

// Names used by the copied interface pragmas.
static const unsigned int n_S_ev = EV_S;
static const unsigned int Nx_ev = EV_NX;
static const unsigned int size_xref_ev = _Nx * _Nh;
static const unsigned int part_S_mem_ev = EV_S * EV_PART;

#ifndef PROTO_CONV
#define PROTO_CONV ((EV_NX + PROTO_II - 1) / PROTO_II)
#endif

static model_t to_model(top_t v) {
#pragma HLS INLINE off
    model_t r = v;
    return r;
}
static top_t to_top(model_t v) {
#pragma HLS INLINE off
    top_t r = v;
    return r;
}
static top_t stage_update(top_t state, top_t Ts_local, top_t k) {
#pragma HLS INLINE off
    top_t r = state + Ts_local * k;
    return r;
}
static top_t final_update(top_t state, top_t k1, top_t k2, top_t k3, top_t k4, top_t Ts_6) {
#pragma HLS INLINE off
    top_t rk4_middle_sum;
    top_t rk4_weighted_sum;
    top_t rk4_delta;
    rk4_middle_sum = k2 + k3;
    rk4_weighted_sum = (k1 + k4) + (top_t)2.0 * rk4_middle_sum;
    rk4_delta = Ts_6 * rk4_weighted_sum;
    top_t r = state + rk4_delta;
    return r;
}

// One component of System::one_step_error (j-th state, horizon step i_step).
static top_t state_error_term(top_t Ji_in, top_t current_x_hat, top_t current_x_ref, int j, bool last) {
#pragma HLS INLINE off
    top_t penality;
    const top_t top_pi = 3.14159265358979323846;
    const top_t top_two_pi = 6.28318530717958647692;
    top_t tmp_err = current_x_hat - current_x_ref;
    if (_controlled_state[j] == 2) {
        if (tmp_err > top_pi)
            tmp_err -= top_two_pi;
        else if (tmp_err <= -top_pi)
            tmp_err += top_two_pi;
    } else if (_controlled_state[j] != 1)
        tmp_err = (top_t)0.0;
    top_t Q_local;
    if (last)
        Q_local = _Qf[j];
    else
        Q_local = _Q[j];
    if ((_state_lower_limits[j] <= current_x_hat) && (current_x_hat <= _state_upper_limits[j]))
        penality = (top_t)Q_local;
    else
        penality = (top_t)1e4;
    top_t r = Ji_in + penality * tmp_err * tmp_err;
    return r;
}

// One component of System::one_step_u_error.
static top_t u_error_term(top_t Jui_in, top_t current_uu, int j) {
#pragma HLS INLINE off
    top_t penality;
    top_t tmp_err = (current_uu - _uss[j]);
    if ((_u_min[j] <= current_uu) && (current_uu <= _u_max[j]))
        penality = _R[j];
    else
        penality = (top_t)1e4;
    top_t r = Jui_in + penality * tmp_err * tmp_err;
    return r;
}

void evaluateFitnessAndDetectLocalBest(
    _pso_hw_real local_x[_pso_n_S * _pso_Nu * _pso_n_U],
    _pso_hw_real local_y[_pso_n_S * _pso_Nu * _pso_n_U],
    _pso_hw_real local_x_curr[_pso_Nx],
    _pso_hw_real local_xref[_pso_Nx * _pso_Nu],
    _pso_hw_real local_find[_pso_n_S]) {
#pragma HLS INTERFACE mode=m_axi port=local_x offset=off depth=part_S_mem_ev bundle=x_mem
#pragma HLS INTERFACE mode=m_axi port=local_y offset=off depth=part_S_mem_ev bundle=y_mem
#pragma HLS INTERFACE mode=m_axi port=local_x_curr offset=direct depth=Nx_ev bundle=current_mem
#pragma HLS INTERFACE mode=m_axi port=local_xref offset=direct depth=size_xref_ev bundle=current_mem
#pragma HLS INTERFACE mode=m_axi port=local_find offset=direct depth=n_S_ev bundle=pso_mem

    // Same constants as the System constructor.
    const top_t Ts = _Ts;
    const top_t Ts_2 = _Ts * (top_t)0.5;
    const top_t Ts_6 = _Ts * (top_t)0.1666666667;

    top_t cg[EV_S][EV_PART];
#pragma HLS ARRAY_PARTITION variable=cg type=cyclic factor=EV_NU dim=2
    top_t xref_l[EV_NH][EV_NX];
#pragma HLS ARRAY_PARTITION variable=xref_l type=complete dim=2
    top_t xs[EV_S][EV_NX], tmp[EV_S][EV_NX], k1[EV_S][EV_NX], k2[EV_S][EV_NX], k3[EV_S][EV_NX];
#pragma HLS ARRAY_PARTITION variable=xs  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=tmp type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k1  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k2  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k3  type=complete dim=2
    top_t Ji[EV_S][EV_NX], Jui[EV_S][EV_NU];
#pragma HLS ARRAY_PARTITION variable=Ji  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=Jui type=complete dim=2
    top_t fx[EV_S];

load_x: for (int k = 0; k < EV_S * EV_PART; k++)
        cg[k / EV_PART][k % EV_PART] = local_x[k];
load_ref: for (int k = 0; k < EV_NX * EV_NH; k++)
        xref_l[k / EV_NX][k % EV_NX] = local_xref[k];
load_state: for (int i = 0; i < EV_NX; i++) {
        const top_t v = local_x_curr[i];
        for (int p = 0; p < EV_S; p++) {
            xs[p][i] = v;
            Ji[p][i] = (top_t)0.0;
            if (i < EV_NU) Jui[p][i] = (top_t)0.0;
        }
    }

horizon: for (int step = 0; step < EV_NH; step++) {
    stage_loop: for (int s = 0; s < 4; s++) {
#pragma HLS LOOP_FLATTEN off
            const top_t Ts_local = (s == 2) ? Ts : Ts_2;
        particle_loop: for (int p = 0; p < EV_S; p++) {
#pragma HLS PIPELINE II=PROTO_II
            // Valid ONLY because stage_loop and horizon are not flattened: each stage drains
            // before the next, and iteration p touches only row p. Re-run the RTL comparison
            // after any schedule change (csim cannot see these pragmas).
#pragma HLS DEPENDENCE variable=tmp type=inter false
#pragma HLS DEPENDENCE variable=xs  type=inter false
#pragma HLS DEPENDENCE variable=Ji  type=inter false
#pragma HLS DEPENDENCE variable=Jui type=inter false
#pragma HLS ALLOCATION function instances=to_model limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=to_top limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=stage_update limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=final_update limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=state_error_term limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=u_error_term limit=1
                top_t uu[EV_NU];
                model_t um[EV_NU];
                for (int j = 0; j < EV_NU; j++) {
                    uu[j] = cg[p][step * EV_NU + j];   // get_next_uu: Nu == Nh, so u_i = cg[i*n_U..]
                    um[j] = to_model(uu[j]);
                }
                if (s == 0)
                    for (int j = 0; j < EV_NU; j++)
                        Jui[p][j] = u_error_term(Jui[p][j], uu[j], j);

                model_t sm[EV_NX], sdm[EV_NX];
                for (int i = 0; i < EV_NX; i++)
                    sm[i] = to_model((s == 0) ? xs[p][i] : tmp[p][i]);
                __model_pipe<model_t>(sdm, sm, um);

                for (int i = 0; i < EV_NX; i++) {
                    const top_t kk = to_top(sdm[i]);
                    if (s < 3) {
                        if (s == 0) k1[p][i] = kk;
                        else if (s == 1) k2[p][i] = kk;
                        else k3[p][i] = kk;
                        tmp[p][i] = stage_update(xs[p][i], Ts_local, kk);
                    } else {
                        const top_t xn = final_update(xs[p][i], k1[p][i], k2[p][i], k3[p][i], kk, Ts_6);
                        Ji[p][i] = state_error_term(Ji[p][i], xn, xref_l[step][i], i, step == EV_NH - 1);
                        xs[p][i] = xn;
                    }
                }
            }
        }
    }

    // J_error: sequential sums in the top type, as in hls_system.hpp.
j_error: for (int p = 0; p < EV_S; p++) {
        top_t Ji_acum = 0.0;
        top_t Jui_acum = 0.0;
        for (int j = 0; j < EV_NX; j++) {
#pragma HLS PIPELINE off
            Ji_acum = Ji_acum + (top_t)Ji[p][j];
        }
        for (int j = 0; j < EV_NU; j++) {
#pragma HLS PIPELINE off
            Jui_acum = Jui_acum + (top_t)Jui[p][j];
        }
        fx[p] = Ji_acum + Jui_acum;
    }

    // Local-best update, as in evaluateFitnessAndDetectLocalBest.
local_best: for (int p = 0; p < EV_S; p++) {
        if (fx[p] < local_find[p]) {
            for (int k = 0; k < EV_PART; k++)
                local_y[p * EV_PART + k] = cg[p][k];
            local_find[p] = fx[p];
        }
    }
}
