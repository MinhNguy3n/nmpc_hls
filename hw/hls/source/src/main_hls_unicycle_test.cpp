// Host check of the unicycle model (UNICYCLE_CONFIG) in the evaluator's own code paths (g++, not
// Vitis HLS). Driven by app/husky/hls_unicycle_check.py, which generates the cases, builds this
// file in two modes and compares the outputs with app/husky/unicycle_model.py (double) and the
// hw_estimates.py emulation of variant A:
//   mode "fixed": -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS: ap_fixed<32,17> top level, float model
//   mode "half":  the same plus -DHW_MODEL_HALF_CSIM from a header copy without `volatile`,
//                 linked to the fpo bit-accurate floating-point model (as validate_hw_run.py)
//
//   ./unicycle_test <cases.txt> <out.txt>
//
// cases.txt, one case per line (states in m, m, rad; controls normalized to +-100):
//   S x y psi z_v z_omega   one RK4 step (System::one_step_prediction) from (x, y, psi)
//   R z_v z_omega           a 25-step rollout from (0, 0, 0) with a constant control
// out.txt: "S i x y psi" and "R i k x y psi" (k = 1..Nh), the ap_fixed values printed exactly.
//
// Also checked here, with a non-zero exit code on failure:
//   Test 3: System::one_step_error / one_step_u_error on hand cases with known J;
//   Test 4 (with -DUNICYCLE_PIPE_CHECK and the generated model_pipe.hpp on the include path):
//           __model and __model_pipe give identical outputs (==) on every "S" input.
// Build at -O0: host GCC miscompiles the multi-word ap_private paths at -O1/-O2.
#include <cmath>
#include <cstdio>
#include <cstring>
#include "hls_pso.hpp"     // config.hpp first, as main_hls_model_unit_test.cpp does
#include "hls_system.hpp"
#ifdef UNICYCLE_PIPE_CHECK
#include "model_pipe.hpp"
#endif

typedef System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> UnicycleSystem;

// Exposes the protected cost terms.
struct CostProbe : public UnicycleSystem {
    using Base = UnicycleSystem;
    using Base::Base;
    using Base::one_step_error;
    using Base::one_step_u_error;
};

static int failures = 0;
static void check(bool ok, const char *what, double got, double expected) {
    printf("%s %s: got %.9g expected %.9g\n", ok ? "ok  " : "FAIL", what, got, expected);
    failures += !ok;
}

static const double kPi = 3.14159265358979323846;
static const double kLsb = 1.0 / 32768.0;  // ap_fixed<32,17>

static void test_cost() {
    CostProbe sys(_u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
                  _Q, _Qf, _R, _uss, _Ts);
    const double top_two_pi = (double)(_hw_top_real)6.28318530717958647692;
    struct StateCase { int j; double x_hat; double ref; unsigned step; double expected_err; bool penalty; const char *name; };
    const StateCase cases[] = {
        {2, 3.0, -3.0, 0, 6.0 - top_two_pi, false, "psi residual across +pi, stage weight Q"},
        {2, -3.0, 3.0, 0, -6.0 + top_two_pi, false, "psi residual across -pi, stage weight Q"},
        {2, 3.0, -3.0, _Nh - 1, 6.0 - top_two_pi, false, "psi residual across +pi, terminal weight Qf"},
        {2, 0.5, 0.25, 0, 0.25, false, "psi residual inside (-pi, pi]"},
        {0, 999.5, 999.0, 0, 0.5, false, "x inside the +-1e3 limit: weight Q"},
        {0, 1000.5, 1000.0, 0, 0.5, true, "x outside the +-1e3 limit: penalty 1e4"},
        {1, -1000.5, -1000.0, _Nh - 1, -0.5, true, "y below -1e3 at the last step: penalty 1e4"},
    };
    for (const StateCase &c : cases) {
        _hw_top_real x[_Nx] = {0}, ref[_Nx] = {0}, ji_in[_Nx] = {0}, ji_out[_Nx];
        x[c.j] = c.x_hat;
        ref[c.j] = c.ref;
        sys.one_step_error(ji_out, c.step, ji_in, x, ref);
        const double w = c.penalty ? 1e4 : (double)(c.step == _Nh - 1 ? _Qf[c.j] : _Q[c.j]);
        const double expected = w * c.expected_err * c.expected_err;
        check(std::fabs((double)ji_out[c.j] - expected) <= 2 * kLsb, c.name, (double)ji_out[c.j], expected);
        for (int k = 0; k < _Nx; k++)
            if (k != c.j && ji_out[k] != 0) check(false, "other states contribute 0", (double)ji_out[k], 0);
    }

    struct ControlCase { double z_v; double z_w; double expected_v; double expected_w; const char *name; };
    const double r_w = (double)_R[1];
    const double sat = 65536.0 - kLsb;  // AP_SAT_SYM maximum of ap_fixed<32,17>
    const ControlCase ucases[] = {
        {-100, 0, 0, 0, "u = uss gives 0"},
        {-50, 40, 0, r_w * 1600, "R_v = 0; omega term R_omega z^2"},
        {100, -100, 0, r_w * 10000, "both at the box edge (inside)"},
        {-100, 100.5, 0, sat, "omega outside the box: 1e4 penalty, saturates"},
    };
    for (const ControlCase &c : ucases) {
        _hw_top_real u[_n_U] = {c.z_v, c.z_w}, ju_in[_n_U] = {0}, ju_out[_n_U];
        sys.one_step_u_error(ju_out, ju_in, u);
        char what[160];
        snprintf(what, sizeof what, "%s (v term)", c.name);
        check(std::fabs((double)ju_out[0] - c.expected_v) <= 2 * kLsb, what, (double)ju_out[0], c.expected_v);
        snprintf(what, sizeof what, "%s (omega term)", c.name);
        check(std::fabs((double)ju_out[1] - c.expected_w) <= 2 * kLsb, what, (double)ju_out[1], c.expected_w);
    }
    printf("weights as stored: Q {%.9g, %.9g, %.9g} Qf {%.9g, %.9g, %.9g} R {%.9g, %.9g} (R_omega source 5e-5)\n",
           (double)_Q[0], (double)_Q[1], (double)_Q[2], (double)_Qf[0], (double)_Qf[1], (double)_Qf[2],
           (double)_R[0], (double)_R[1]);
    printf("stored: _Ts %.9g  Ts/2 %.9g  Ts/6 %.9g  du_max {%.9g, %.9g}  uss {%.9g, %.9g}  R_omega(5e-6) -> %.9g\n",
           (double)_Ts, (double)(_Ts * (_hw_top_real)0.5), (double)(_Ts * (_hw_top_real)0.1666666667),
           (double)_du_max[0], (double)_du_max[1], (double)_uss[0], (double)_uss[1], (double)(_hw_top_real)0.000005);
}

static void print_state(FILE *out, const _hw_top_real *x) {
    for (int i = 0; i < _Nx; i++) fprintf(out, " %.15f", (double)x[i]);
    fprintf(out, "\n");
}

#ifdef UNICYCLE_PIPE_CHECK
// Test 4: the generated pipelined copy computes the same outputs as the original model, with the
// conversions System::model() applies (top type -> model type -> top type).
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

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s cases.txt out.txt\n", argv[0]);
        return 2;
    }
    printf("Test 3: cost terms\n");
    test_cost();

    FILE *in = fopen(argv[1], "r"), *out = fopen(argv[2], "w");
    if (!in || !out) {
        fprintf(stderr, "cannot open %s or %s\n", argv[1], argv[2]);
        return 2;
    }
    UnicycleSystem sys(_u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
                       _Q, _Qf, _R, _uss, _Ts);
    char kind[4];
    int n_step = 0, n_roll = 0;
    while (fscanf(in, "%3s", kind) == 1) {
        if (strcmp(kind, "S") == 0) {
            double s[3], z[2];
            if (fscanf(in, "%lf %lf %lf %lf %lf", &s[0], &s[1], &s[2], &z[0], &z[1]) != 5) break;
            _hw_top_real x[_Nx] = {s[0], s[1], s[2]}, u[_n_U] = {z[0], z[1]}, next[_Nx];
            sys.one_step_prediction(next, x, u);
            fprintf(out, "S %d", n_step);
            print_state(out, next);
#ifdef UNICYCLE_PIPE_CHECK
            compare_pipe(x, u);
#endif
            n_step++;
        } else if (strcmp(kind, "R") == 0) {
            double z[2];
            if (fscanf(in, "%lf %lf", &z[0], &z[1]) != 2) break;
            _hw_top_real x[_Nx] = {0, 0, 0}, u[_n_U] = {z[0], z[1]}, next[_Nx];
            for (int k = 1; k <= _Nh; k++) {
                sys.one_step_prediction(next, x, u);
                for (int i = 0; i < _Nx; i++) x[i] = next[i];
                fprintf(out, "R %d %d", n_roll, k);
                print_state(out, x);
            }
            n_roll++;
        } else {
            fprintf(stderr, "bad case kind %s\n", kind);
            return 2;
        }
    }
    fclose(in);
    fclose(out);
    printf("Tests 1-2: %d one-step cases and %d rollouts written to %s\n", n_step, n_roll, argv[2]);
#ifdef UNICYCLE_PIPE_CHECK
    printf("Test 4: __model vs __model_pipe: %ld outputs compared, %ld differ\n", pipe_compared, pipe_mismatches);
    if (pipe_mismatches || pipe_compared == 0) failures++;
#endif
    printf(failures ? "UNICYCLE_TEST_FAIL (%d)\n" : "UNICYCLE_TEST_PASS\n", failures);
    return failures ? 1 : 0;
}
