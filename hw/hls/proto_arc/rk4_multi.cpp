#include "rk4_multi.hpp"
#include "hls_model.hpp"   // brings in the original model header (unused here) and its helpers
#include "model_pipe.hpp"  // __model_pipe: the same model without pragmas, see gen_headers.sh

// Type conversions as separate, non-inlined functions so HLS can share a few converter
// instances across the II cycles of the pipelined particle loop. Inlined, each of the 24
// per-call conversions (ap_fixed<32,17> <-> half, with AP_SAT_SYM saturation) becomes its own
// ~1-2k-LUT expression tree, which HLS does not share: the first sweep spent 55k LUT on them.
// The conversion itself is the plain assignment System::model() uses (dest[i] = src[i]).
static proto_model_t to_model(proto_top_t v) {
#pragma HLS INLINE off
    proto_model_t r = v;
    return r;
}
static proto_top_t to_top(proto_model_t v) {
#pragma HLS INLINE off
    proto_top_t r = v;
    return r;
}

// Converter instances per direction: enough for 12 conversions per II cycles.
#ifndef PROTO_CONV
#define PROTO_CONV ((PROTO_NX + PROTO_II - 1) / PROTO_II)
#endif

// Same conversions as System::model(): top -> model type, evaluate, model -> top. The control
// is converted once per particle at load (a pure function of the same value, so identical).
static void model_conv(proto_top_t state_dot[PROTO_NX],
                       const proto_top_t state[PROTO_NX],
                       const proto_model_t control_model[PROTO_NU]) {
#pragma HLS INLINE
    proto_model_t state_dot_model[PROTO_NX];
    proto_model_t state_model[PROTO_NX];
    proto_model_t control_local[PROTO_NU];
    for (int i = 0; i < PROTO_NX; i++)
        state_model[i] = to_model(state[i]);
    for (int i = 0; i < PROTO_NU; i++)
        control_local[i] = control_model[i];
    __model_pipe<proto_model_t>(state_dot_model, state_model, control_local);
    for (int i = 0; i < PROTO_NX; i++)
        state_dot[i] = to_top(state_dot_model[i]);
}

// The fixed-point RK4 updates, also as shared non-inlined functions. Each returns the same
// full-precision expression as update_state()/update_state_final() in hls_system.hpp, quantized
// once to the top type on return (as the original assignment does).
static proto_top_t stage_update(proto_top_t state, proto_top_t Ts_local, proto_top_t k) {
#pragma HLS INLINE off
    proto_top_t r = state + Ts_local * k;
    return r;
}
static proto_top_t final_update(proto_top_t state, proto_top_t k1, proto_top_t k2, proto_top_t k3,
                                proto_top_t k4, proto_top_t Ts_6) {
#pragma HLS INLINE off
    proto_top_t rk4_middle_sum;
    proto_top_t rk4_weighted_sum;
    proto_top_t rk4_delta;
    rk4_middle_sum = k2 + k3;
    rk4_weighted_sum = (k1 + k4) + (proto_top_t)2.0 * rk4_middle_sum;
    rk4_delta = Ts_6 * rk4_weighted_sum;
    proto_top_t r = state + rk4_delta;
    return r;
}

void rk4_multi(proto_top_t x[PROTO_S * PROTO_NX],
               proto_top_t u[PROTO_S * PROTO_NU],
               proto_top_t x_next[PROTO_S * PROTO_NX]) {
    // Same constants as the System constructor (Ts_2, Ts_6 computed in the top type).
    const proto_top_t Ts = _Ts;
    const proto_top_t Ts_2 = _Ts * (proto_top_t)0.5;
    const proto_top_t Ts_6 = _Ts * (proto_top_t)0.1666666667;

    proto_top_t xs[PROTO_S][PROTO_NX], tmp[PROTO_S][PROTO_NX];
    proto_model_t us[PROTO_S][PROTO_NU];
    proto_top_t k1[PROTO_S][PROTO_NX], k2[PROTO_S][PROTO_NX], k3[PROTO_S][PROTO_NX];
#pragma HLS ARRAY_PARTITION variable=xs  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=us  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=tmp type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k1  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k2  type=complete dim=2
#pragma HLS ARRAY_PARTITION variable=k3  type=complete dim=2

load: for (int p = 0; p < PROTO_S; p++) {
        for (int i = 0; i < PROTO_NX; i++) xs[p][i] = x[p * PROTO_NX + i];
        for (int i = 0; i < PROTO_NU; i++) us[p][i] = to_model(u[p * PROTO_NU + i]);
    }

stage_loop: for (int s = 0; s < 4; s++) {
#pragma HLS LOOP_FLATTEN off
        // stage_1/2 use Ts_2, stage_3 uses Ts (update_state in one_step_prediction).
        const proto_top_t Ts_local = (s == 2) ? Ts : Ts_2;
    particle_loop: for (int p = 0; p < PROTO_S; p++) {
#pragma HLS PIPELINE II=PROTO_II
            // Valid ONLY because stage_loop is LOOP_FLATTEN off: each stage drains before the
            // next, and within a stage iteration p touches only row p of tmp. Any schedule that
            // overlaps stages (flattening, a wavefront) makes this pragma silently wrong, and
            // csim cannot detect it: re-run the RTL dump-and-cmp after any schedule change.
#pragma HLS DEPENDENCE variable=tmp type=inter false
#pragma HLS ALLOCATION function instances=to_model limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=to_top limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=stage_update limit=PROTO_CONV
#pragma HLS ALLOCATION function instances=final_update limit=PROTO_CONV
            proto_top_t in[PROTO_NX], kk[PROTO_NX];
            for (int i = 0; i < PROTO_NX; i++)
                in[i] = (s == 0) ? xs[p][i] : tmp[p][i];
            model_conv(kk, in, us[p]);

            for (int i = 0; i < PROTO_NX; i++) {
                if (s < 3) {
                    if (s == 0) k1[p][i] = kk[i];
                    else if (s == 1) k2[p][i] = kk[i];
                    else k3[p][i] = kk[i];
                    tmp[p][i] = stage_update(xs[p][i], Ts_local, kk[i]);
                } else {
                    x_next[p * PROTO_NX + i] =
                        final_update(xs[p][i], k1[p][i], k2[p][i], k3[p][i], kk[i], Ts_6);
                }
            }
        }
    }
}
