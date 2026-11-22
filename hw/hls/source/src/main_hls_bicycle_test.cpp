// Host check of the bicycle model (BICYCLE_CONFIG) and the PSO redesign switches in the solver's own
// code paths (g++, not Vitis HLS; C level only). Driven by app/rc_car/hls_bicycle_check.py, which
// generates the inputs, builds this file (fixed: -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS; half: the same
// plus -DHW_MODEL_HALF_CSIM from a header copy without `volatile`, linked to the fpo bit-accurate
// floating-point model; _n_S/_maxiter patched in the header copy as validate_hw_run.py does) and
// compares the outputs with app/ackermann (the model), app/rc_car/hls_bicycle_emul.py (a bit-level
// emulation of these code paths) and the Python controller (app/rc_car/nmpc_engine.py).
//
//   ./bicycle_test model <cases.txt> <out.txt>
//       "S x y psi z_v z_kappa": one System::one_step_prediction (RK4) from (x, y, psi)
//       "R z_v z_kappa": a _Nh-step rollout from (0, 0, 0) with a constant control
//       -> "S i x y psi" and "R i k x y psi" (k = 1.._Nh); with -DBICYCLE_PIPE_CHECK also
//       __model == __model_pipe (the engine's generated copy) on every "S" input
//   ./bicycle_test cost <problems.txt> <out.txt>
//       per problem "P id K", x_curr[_Nx], u_curr[_n_U], xref[_Nh*_Nx], rd_aux[_RD_AUX_LEN] (if any
//       PSO_RD_* switch is on), then K plans of _Nu*_n_U values (normalized)
//       -> "J id k value": System::nmpc_cost_function_topflow, the evaluator's per-particle cost
//   ./bicycle_test solve <problems.txt> <out.txt> <seed>
//       per problem "P id", x_curr, u_curr, xref, rd_aux, last_best[_Nu*_n_U]; srand(seed + id), then
//       the cores in pso_fsm's order: initializeParticles_set, then _maxiter times
//       evaluateFitnessAndDetectLocalBest, detectGlobalMinimum, updateParticlesWithDuConstrains
//       (its _swrng C-simulation variant) -> "B id new_best...", "F id bestfitness[0.._maxiter-1]",
//       "N id next_rand" (the next rand() value, so the caller can check how many draws were used)
// All values are read as doubles on the ap_fixed<32,17> grid and printed exactly (%.15f).
// Build at -O0: host GCC miscompiles the multi-word ap_private paths at -O1/-O2.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "hls_pso.hpp"     // config.hpp first
#include "hls_system.hpp"
#ifdef BICYCLE_PIPE_CHECK
#include "model_pipe.hpp"  // gen_headers.sh's transformation of hls_bicycle.hpp (pragma lines deleted)
#endif

#ifndef BICYCLE_CONFIG
#error "main_hls_bicycle_test.cpp needs -DBICYCLE_CONFIG"
#endif
#if !defined(HW_ARITH_CSIM)
#error "main_hls_bicycle_test.cpp needs -DHW_ARITH_CSIM (ap_fixed top level, the software rand port)"
#endif

#ifdef PSO_RD_ANY
#define AUX_LEN _RD_AUX_LEN
#else
#define AUX_LEN 0
#endif
#define PART (_Nu * _n_U)

typedef System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> BicycleSystem;

static BicycleSystem make_system() {
    return BicycleSystem(_u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
                         _Q, _Qf, _R, _uss, _Ts);
}

static void print_values(FILE *out, const _hw_top_real *v, int n) {
    for (int i = 0; i < n; i++) fprintf(out, " %.15f", (double)v[i]);
    fprintf(out, "\n");
}

static bool read_values(FILE *in, _hw_top_real *v, int n) {
    for (int i = 0; i < n; i++) {
        double d;
        if (fscanf(in, "%lf", &d) != 1) return false;
        v[i] = d;
    }
    return true;
}

#ifdef BICYCLE_PIPE_CHECK
static long pipe_mismatches = 0, pipe_compared = 0;
static void compare_pipe(const _hw_top_real *x, const _hw_top_real *u) {
    _hw_model_real xs[_Nx], us[_n_U], a[_Nx], b[_Nx];
    for (int i = 0; i < _Nx; i++) xs[i] = x[i];
    for (int i = 0; i < _n_U; i++) us[i] = u[i];
    __model<_hw_model_real>(a, xs, us);
    __model_pipe<_hw_model_real>(b, xs, us);
    for (int i = 0; i < _Nx; i++) {
        pipe_compared++;
        if (!(a[i] == b[i])) pipe_mismatches++;
    }
}
#endif

static int run_model(FILE *in, FILE *out) {
    BicycleSystem sys = make_system();
    char kind[4];
    int n_step = 0, n_roll = 0;
    while (fscanf(in, "%3s", kind) == 1) {
        if (strcmp(kind, "S") == 0) {
            _hw_top_real x[_Nx], u[_n_U], next[_Nx];
            if (!read_values(in, x, _Nx) || !read_values(in, u, _n_U)) return 2;
            sys.one_step_prediction(next, x, u);
#ifdef BICYCLE_PIPE_CHECK
            compare_pipe(x, u);
#endif
            fprintf(out, "S %d", n_step++);
            print_values(out, next, _Nx);
        } else if (strcmp(kind, "R") == 0) {
            _hw_top_real x[_Nx] = {0, 0, 0}, u[_n_U], next[_Nx];
            if (!read_values(in, u, _n_U)) return 2;
            for (int k = 1; k <= _Nh; k++) {
                sys.one_step_prediction(next, x, u);
                for (int i = 0; i < _Nx; i++) x[i] = next[i];
                fprintf(out, "R %d %d", n_roll, k);
                print_values(out, x, _Nx);
            }
            n_roll++;
        } else {
            fprintf(stderr, "bad case kind %s\n", kind);
            return 2;
        }
    }
    printf("model: %d one-step cases, %d rollouts\n", n_step, n_roll);
#ifdef BICYCLE_PIPE_CHECK
    printf("pipe: __model vs __model_pipe: %ld outputs compared, %ld differ\n", pipe_compared, pipe_mismatches);
    if (pipe_mismatches || pipe_compared == 0) return 3;
#endif
    return 0;
}

struct ProblemIn {
    _hw_top_real x_curr[_Nx], u_curr[_n_U], xref[_Nh * _Nx];
    _hw_top_real aux[AUX_LEN + 1];
};

static bool read_problem(FILE *in, ProblemIn &p) {
    return read_values(in, p.x_curr, _Nx) && read_values(in, p.u_curr, _n_U) && read_values(in, p.xref, _Nh * _Nx)
           && read_values(in, p.aux, AUX_LEN);
}

static int run_cost(FILE *in, FILE *out) {
    char kind[4];
    int n = 0, total = 0;
    while (fscanf(in, "%3s", kind) == 1) {
        int id, K;
        ProblemIn p;
        if (strcmp(kind, "P") != 0 || fscanf(in, "%d %d", &id, &K) != 2 || !read_problem(in, p)) return 2;
        for (int k = 0; k < K; k++) {
            _hw_top_real plan[PART], J;
            if (!read_values(in, plan, PART)) return 2;
            BicycleSystem sys = make_system();  // as the evaluator: one System per particle
#ifdef PSO_RD_ANY
            sys.nmpc_cost_function_topflow(p.x_curr, plan, p.xref, p.u_curr, p.aux, &J);
#else
            sys.nmpc_cost_function_topflow(p.x_curr, plan, p.xref, &J);
#endif
            fprintf(out, "J %d %d %.15f\n", id, k, (double)J);
            total++;
        }
        n++;
    }
    printf("cost: %d problems, %d plans\n", n, total);
    return 0;
}

static int run_solve(FILE *in, FILE *out, int seed) {
    char kind[4];
    int n = 0;
    while (fscanf(in, "%3s", kind) == 1) {
        int id;
        ProblemIn p;
        _hw_top_real last_best[PART];
        if (strcmp(kind, "P") != 0 || fscanf(in, "%d", &id) != 1 || !read_problem(in, p)
            || !read_values(in, last_best, PART)) return 2;

        static _hw_top_real x[_n_S * PART], y[_n_S * PART], v[_n_S * PART];
        _hw_top_real f_ind[_n_S], bestfitness[_maxiter], global_min[PART];
        _hw_top_real u_curr_local[_n_U], x_curr_local[_Nx], xref_local[_Nx * _Nh];
        _hw_top_real du_max[_n_U], du_min[_n_U], u_max[_n_U], u_min[_n_U], uss[_n_U];
        for (int i = 0; i < _n_U; i++) {
            du_max[i] = _du_max[i];
            u_max[i] = _u_max[i];
            u_min[i] = _u_min[i];
            uss[i] = _uss[i];
        }
        srand(seed + id);
        initializeParticles_set(p.u_curr, p.x_curr, p.xref, last_best, u_curr_local, x_curr_local, xref_local,
                                x, y, v, f_ind, du_min, du_max, u_min, u_max, uss
#ifdef PSO_RD_ANY
                                , p.aux
#endif
        );
        for (int k = 0; k < _maxiter; k++) {
            evaluateFitnessAndDetectLocalBest(x, y, x_curr_local, xref_local, f_ind
#ifdef PSO_RD_ANY
                                              , u_curr_local, p.aux
#endif
            );
            detectGlobalMinimum(f_ind, bestfitness, global_min, k, y);
            updateParticlesWithDuConstrains_swrng(u_curr_local, x, y, v, global_min, du_max, du_min, u_max, u_min, k);
        }
        fprintf(out, "B %d", id);
        print_values(out, global_min, PART);
        fprintf(out, "F %d", id);
        print_values(out, bestfitness, _maxiter);
        fprintf(out, "N %d %d\n", id, rand());
        n++;
    }
    printf("solve: %d problems (n_S %d, maxiter %d, Nu %d)\n", n, _n_S, _maxiter, _Nu);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s model|cost|solve in.txt out.txt [seed]\n", argv[0]);
        return 2;
    }
    FILE *in = fopen(argv[2], "r"), *out = fopen(argv[3], "w");
    if (!in || !out) {
        fprintf(stderr, "cannot open %s or %s\n", argv[2], argv[3]);
        return 2;
    }
    printf("config: Nh %d Nu %d n_S %d maxiter %d aux %d  Ts %.15f  du_max %.15f %.15f\n", _Nh, _Nu, _n_S, _maxiter,
           AUX_LEN, (double)_Ts, (double)_du_max[0], (double)_du_max[1]);
    int rc;
    if (strcmp(argv[1], "model") == 0) rc = run_model(in, out);
    else if (strcmp(argv[1], "cost") == 0) rc = run_cost(in, out);
    else if (strcmp(argv[1], "solve") == 0) rc = run_solve(in, out, argc > 4 ? atoi(argv[4]) : 1);
    else rc = 2;
    fclose(in);
    fclose(out);
    printf(rc ? "BICYCLE_TEST_FAIL (%d)\n" : "BICYCLE_TEST_DONE\n", rc);
    return rc;
}
