// Host-side validation helper for hardware solver runs (built with g++ at -O0, not Vitis HLS).
//
//   main_hls_validate cost  < x_curr[Nx] xref[Nx*Nh] u[Nu*n_U]
//       Prints the cost J(u) computed by the C evaluator (evaluateFitnessAndDetectLocalBest).
//   main_hls_validate solve <seed> < x_curr[Nx] xref[Nx*Nh] u_curr[n_U] last_best[Nu*n_U]
//       Runs the software PSO (execute) with srand(seed) and prints its control sequence and J.
//   main_hls_validate trajectory < x_curr[Nx] xref[Nx*Nh] u[Nu*n_U]
//       Runs the cost function one horizon step at a time and prints, per step, the predicted
//       state and the accumulated state and control objective terms ("S i x[0..Nx-1] Ji Jui").
//   main_hls_validate serve < { seed x_curr[Nx] xref[Nx*Nh] u_curr[n_U] last_best[Nu*n_U] }...
//       Closed-loop helper: prints "config n_S maxiter Nh Nu", then solves one problem per record
//       until end of input, printing "u ...", "Jrecomputed J" and "end" for each. A seed > 0 calls
//       srand(seed) before that solve; 0 continues the current random stream, as the hardware's
//       free-running generator does between solves.
//
// Builds (see app/kria-pynq/validate_hw_run.py):
//   float:  no HW_ARITH_CSIM. "trajectory" uses double arithmetic with std::sin: the reference
//           (REF). It checks numerical fidelity to the model equations, not the physical plant.
//   hw:     -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS: ap_fixed<32,17> top level, float model.
//   half:   as hw plus -DHW_MODEL_HALF_CSIM, from a header copy without `volatile`: the
//           hardware's own arithmetic (ap_fixed top level, half model, fast sin).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "hls_pso.hpp"
#include "hls_system.hpp"

static const int part = _pso_Nu * _pso_n_U;

static void read_values(_pso_hw_real *dst, int n) {
    for (int i = 0; i < n; i++) {
        double v;
        if (scanf("%lf", &v) != 1) {
            fprintf(stderr, "input ended after %d of %d values\n", i, n);
            exit(2);
        }
        dst[i] = v;
    }
}

static double cost(_pso_hw_real *x_curr, _pso_hw_real *xref, _pso_hw_real *u) {
    static _pso_hw_real x[_pso_n_S * part], y[_pso_n_S * part], find[_pso_n_S];
    for (int i = 0; i < _pso_n_S; i++) {
        memcpy(&x[i * part], u, sizeof(_pso_hw_real) * part);
        find[i] = H_MAX;
    }
    evaluateFitnessAndDetectLocalBest(x, y, x_curr, xref, find);
    return (double)find[0];
}

// Exposes one horizon step of the cost function (protected in System).
template <typename Top, typename Model>
struct StepProbe : public System<Top, Model, _Nh, _Nx, _n_U, _Nu> {
    typedef System<Top, Model, _Nh, _Nx, _n_U, _Nu> Base;
    template <typename... A> explicit StepProbe(A... a) : Base(a...) {}
    using Base::nmpc_cost_function_lowflow;
};

template <typename T, int N> struct Converted {
    T v[N];
    explicit Converted(const _hw_top_real *s) { for (int i = 0; i < N; i++) v[i] = (T)(double)s[i]; }
};

template <typename Top, typename Model>
static void trajectory(const double *x0, const double *xref, const double *u) {
    static Converted<Top, _n_U> u_max(_u_max), u_min(_u_min), du_max(_du_max), R(_R), uss(_uss);
    static Converted<Top, _Nx> upper(_state_upper_limits), lower(_state_lower_limits), Q(_Q), Qf(_Qf);
    StepProbe<Top, Model> sys(u_max.v, u_min.v, du_max.v, _controlled_state, upper.v, lower.v,
                              Q.v, Qf.v, R.v, uss.v, (Top)(double)_Ts);
    Top x[_Nx], x_next[_Nx], ji[_Nx], ji_next[_Nx], jui[_n_U], jui_next[_n_U], ref[_Nx], uu[_n_U];
    for (int j = 0; j < _Nx; j++) { x[j] = (Top)x0[j]; ji[j] = 0; }
    for (int k = 0; k < _n_U; k++) jui[k] = 0;
    for (unsigned i = 0; i < _Nh; i++) {
        unsigned ui = i < _Nu ? i : _Nu - 1;
        for (int j = 0; j < _Nx; j++) ref[j] = (Top)xref[i * _Nx + j];
        for (int k = 0; k < _n_U; k++) uu[k] = (Top)u[ui * _n_U + k];
        sys.nmpc_cost_function_lowflow(x, uu, ref, jui, ji, i, x_next, jui_next, ji_next);
        double ji_sum = 0, jui_sum = 0;
        for (int j = 0; j < _Nx; j++) ji_sum += (double)ji_next[j];
        for (int k = 0; k < _n_U; k++) jui_sum += (double)jui_next[k];
        printf("S %u", i);
        for (int j = 0; j < _Nx; j++) printf(" %.9g", (double)x_next[j]);
        printf(" %.9g %.9g\n", ji_sum, jui_sum);
        for (int j = 0; j < _Nx; j++) { x[j] = x_next[j]; ji[j] = ji_next[j]; }
        for (int k = 0; k < _n_U; k++) jui[k] = jui_next[k];
    }
}

static void read_doubles(double *dst, int n) {
    for (int i = 0; i < n; i++)
        if (scanf("%lf", &dst[i]) != 1) {
            fprintf(stderr, "input ended after %d of %d values\n", i, n);
            exit(2);
        }
}

int main(int argc, char **argv) {
    static _pso_hw_real x_curr[_pso_Nx], xref[_pso_Nx * _pso_Nh];
    static _pso_hw_real u_curr[_pso_n_U], last_best[part], u[part];

    if (argc >= 2 && strcmp(argv[1], "cost") == 0) {
        read_values(x_curr, _pso_Nx);
        read_values(xref, _pso_Nx * _pso_Nh);
        read_values(u, part);
        printf("J %.6f\n", cost(x_curr, xref, u));
        return 0;
    }
    if (argc >= 3 && strcmp(argv[1], "solve") == 0) {
        srand((unsigned)atoi(argv[2]));
        read_values(x_curr, _pso_Nx);
        read_values(xref, _pso_Nx * _pso_Nh);
        read_values(u_curr, _pso_n_U);
        read_values(last_best, part);
        _pso_hw_real J[1];
        execute(x_curr, u_curr, 0, last_best, xref, u, J);
        printf("J %.6f\n", (double)J[0]);
        printf("Jrecomputed %.6f\n", cost(x_curr, xref, u));
        printf("u");
        for (int i = 0; i < part; i++)
            printf(" %.6f", (double)u[i]);
        printf("\n");
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "serve") == 0) {
        printf("config %d %d %d %d\n", _pso_n_S, _pso_maxiter, _pso_Nh, _pso_Nu);
        fflush(stdout);
        double seed;
        while (scanf("%lf", &seed) == 1) {
            if (seed > 0)
                srand((unsigned)seed);
            read_values(x_curr, _pso_Nx);
            read_values(xref, _pso_Nx * _pso_Nh);
            read_values(u_curr, _pso_n_U);
            read_values(last_best, part);
            _pso_hw_real J[1];
            execute(x_curr, u_curr, 0, last_best, xref, u, J);
            printf("u");
            for (int i = 0; i < part; i++)
                printf(" %.9g", (double)u[i]);
            printf("\nJrecomputed %.6f\nend\n", cost(x_curr, xref, u));
            fflush(stdout);
        }
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "trajectory") == 0) {
        static double x0[_Nx], ref[_Nx * _Nh], uu[part];
        read_doubles(x0, _Nx);
        read_doubles(ref, _Nx * _Nh);
        read_doubles(uu, part);
#if defined(HW_ARITH_CSIM)
        trajectory<_hw_top_real, _hw_model_real>(x0, ref, uu);
#else
        trajectory<double, double>(x0, ref, uu);
#endif
        return 0;
    }
    fprintf(stderr, "usage: %s cost | solve <seed> | trajectory | serve  (values on stdin)\n", argv[0]);
    return 1;
}
