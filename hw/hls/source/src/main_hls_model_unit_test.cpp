// Host unit tests for the hardware model helpers (built with g++, not Vitis HLS):
//   1. fast_cossin_table: 1024 entries of sin(2*pi*k/1024)
//   2. fastsin index rounding and wrap: negative values, rounding ties, exact/near multiples of
//      the table period and wrap boundaries
//   3. local_sin accuracy against std::sin (float/double over [-4pi, 4pi], half over the model's
//      operating range: tilt angles +-0.8 rad, also shifted by +pi/2 for cos)
//   4. one_step_error penalty switch at, just inside and just outside the tightened roll/pitch
//      cost limit (pi/4 - _angle_limit_margin), and that the cost limit stays inside +-pi/4
//   5. one_step_error angle residual (controlled_state 2) is wrap(x_hat - ref) in (-pi, pi] for
//      nonzero references of both signs, on both sides of +-pi and across the +-pi crossings,
//      for any |x_hat - ref| < 3*pi (e.g. an unwrapped state against a wrapped reference)
//
// Build from hw/hls/source:
//   g++ -std=c++11 -O0 -D__VITIS__ -DPSO_CONFIG -DSNIFFBOT_CONFIG -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS \
//       -Iinclude -Iinclude/models -I$XILINX_HLS/include src/main_hls_model_unit_test.cpp \
//       -L$XILINX_VIVADO/lnx64/tools/fpo_v7_1 -lIp_floating_point_v7_1_bitacc_cmodel \
//       -Wl,-rpath,$XILINX_VIVADO/lnx64/tools/fpo_v7_1 -o model_unit_test
// Build at -O0: host GCC miscompiles the multi-word ap_private paths (>64-bit ap_fixed
// intermediates) at -O1/-O2, which corrupts the cost terms.
// Returns 0 when all checks pass.
#include <cmath>
#include <cstdio>
#include "hls_pso.hpp"
#include "hls_system.hpp"

static int failures = 0;
static void check(bool ok, const char *what, double got, double expected) {
    if (!ok) {
        printf("FAIL %s: got %.9g expected %.9g\n", what, got, expected);
        failures++;
    }
}

static const double kPi = 3.14159265358979323846;
static const double kStep = 2 * kPi / MAX_CIRCLE_ANGLE;

static void test_table() {
    fast_cossin_table_class<double> t;
    int n = sizeof(t.table) / sizeof(t.table[0]);
    check(n == 1024, "table length", n, 1024);
    double worst = 0;
    for (int k = 0; k < n; k++)
        worst = std::fmax(worst, std::fabs(t.table[k] - std::sin(2 * kPi * k / 1024)));
    check(worst < 1e-15, "table entries vs sin(2*pi*k/1024)", worst, 0);
    printf("table: %d entries, max |entry - sin(2*pi*k/1024)| = %.3g\n", n, worst);
}

static void test_index_wrap() {
    // fastsin takes the angle already scaled to table units (angle * 512/pi).
    const double scaled[] = {0, 0.49, 0.5, -0.49, -0.5, 1, -1, 255.5, 256, 511.5, 1023.4, 1023.5,
                             1023.6, 1024, -1024, -1023.5, -1024.5, 2047.5, 2048, -2048, 3000.25};
    int worst_case = 0;
    for (double n : scaled) {
        long r = (long)(n >= 0 ? n + 0.5 : n - 0.5);   // round half away from zero
        double expected = std::sin(2 * kPi * r / 1024);
        double got = fastsin<double>(n);
        bool ok = std::fabs(got - expected) < 1e-12;
        check(ok, "fastsin index/wrap", got, expected);
        worst_case += !ok;
    }
    printf("index/wrap: %zu cases, %d mismatches\n", sizeof(scaled) / sizeof(scaled[0]), worst_case);
}

template <typename T>
static double max_sin_error(double lo, double hi) {
    double worst = 0;
    for (double x = lo; x <= hi; x += 1e-4) {
        T xt = (T)x;
        worst = std::fmax(worst, std::fabs((double)local_sin<T>(xt) - std::sin((double)xt)));
    }
    return worst;
}

static void test_accuracy() {
    double ef = max_sin_error<float>(-4 * kPi, 4 * kPi), ed = max_sin_error<double>(-4 * kPi, 4 * kPi);
    // Rounding to the nearest entry: |error| <= step/2 (plus float rounding of the scaled angle).
    check(ed <= kStep / 2 + 1e-9, "local_sin<double> error over [-4pi, 4pi]", ed, kStep / 2);
    check(ef <= kStep / 2 + 1e-5, "local_sin<float> error over [-4pi, 4pi]", ef, kStep / 2);
    // half: the scaled angle (up to ~400 table units) is itself quantised to 0.25, so allow
    // step/2 + step/8 + the half quantisation of the input angle (2^-11 at |x| < 2).
    double bound_half = kStep / 2 + kStep / 8 + std::ldexp(1.0, -11);
    double eh = std::fmax(max_sin_error<half>(-0.8, 0.8), max_sin_error<half>(kPi / 2 - 0.8, kPi / 2 + 0.8));
    check(eh <= bound_half, "local_sin<half> error over the operating range", eh, bound_half);
    printf("accuracy: double %.5f  float %.5f  (bound %.5f);  half %.5f (bound %.5f)\n", ed, ef, kStep / 2, eh, bound_half);
}

// Exposes the protected penalty term of the cost function.
struct ErrorProbe : public System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> {
    using Base = System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu>;
    using Base::Base;
    using Base::one_step_error;
};

static void test_thresholds() {
    ErrorProbe sys(_u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
                   _Q, _Qf, _R, _uss, _Ts);
    const _hw_top_real limit = _state_upper_limits[3];
    const _hw_top_real lsb = _hw_top_real(1) >> 15;                // one ap_fixed<32,17> step
    const double plant = kPi / 4;
    check((double)limit < plant, "cost limit inside the plant limit", (double)limit, plant);
    check(std::fabs(((double)_hw_top_real(drone_pi) / 4 - (double)limit) - 0.01) < 1e-4,
          "cost limit margin", (double)_hw_top_real(drone_pi) / 4 - (double)limit, 0.01);

    struct Case { _hw_top_real roll; bool inside_cost; bool inside_plant; const char *name; };
    const Case cases[] = {
        {limit, true, true, "at cost limit"},
        {limit - lsb, true, true, "just inside cost limit"},
        {limit + lsb, false, true, "just outside cost limit"},
        {_hw_top_real(plant) - lsb, false, true, "just inside plant limit"},
        {_hw_top_real(plant) + _hw_top_real(0.001), false, false, "just outside plant limit"},
        {-limit, true, true, "at negative cost limit"},
        {-limit - lsb, false, true, "just outside negative cost limit"},
    };
    for (const Case &c : cases) {
        _hw_top_real x[_Nx] = {0}, ref[_Nx] = {0}, ji_in[_Nx] = {0}, ji_out[_Nx];
        x[3] = c.roll;
        sys.one_step_error(ji_out, 0, ji_in, x, ref);              // step 0: weight Q[3]
        double err2 = (double)c.roll * (double)c.roll;
        double expected = (c.inside_cost ? (double)_Q[3] : 1e4) * err2;
        check(std::fabs((double)ji_out[3] - expected) <= 1e-3 * expected + 1e-3, c.name, (double)ji_out[3], expected);
        bool plant_ok = std::fabs((double)c.roll) <= plant;
        check(plant_ok == c.inside_plant, c.name, plant_ok, c.inside_plant);
    }
    printf("thresholds: cost limit %.6f (plant %.6f), %zu cases\n", (double)limit, plant, sizeof(cases) / sizeof(cases[0]));
}

static void test_angle_wrap() {
    ErrorProbe sys(_u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
                   _Q, _Qf, _R, _uss, _Ts);
    const int j = 5;  // yaw: controlled_state 2, limits +-1e3, so no penalty switch
    struct Case { double x_hat; double ref; const char *name; };
    const Case cases[] = {
        {0.2, -0.1, "small difference"},
        {3.0, -3.0, "both inside, difference above +pi"},
        {-3.0, 3.0, "both inside, difference below -pi"},
        {3.3, 0.5, "x_hat > pi, ref < pi, nonzero ref"},
        {3.3, 3.0, "x_hat > pi, ref < pi, close"},
        {3.3, 0.0, "x_hat > pi, ref = 0"},
        {-3.3, 3.0, "x_hat < -pi, ref > -pi"},
        {-3.3, -0.5, "x_hat < -pi, ref > -pi, close"},
        {7.0, 7.1, "both unwrapped beyond 2*pi"},
        {7.0, 0.8, "unwrapped x_hat, wrapped ref"},
        {-7.0, -0.8, "negative unwrapped x_hat, wrapped ref"},
        {1.0 + 2.9 * kPi, 1.0, "difference 2.9*pi (inside the 3*pi contract)"},
    };
    for (const Case &c : cases) {
        _hw_top_real x[_Nx] = {0}, ref[_Nx] = {0}, ji_in[_Nx] = {0}, ji_out[_Nx];
        x[j] = c.x_hat;
        ref[j] = c.ref;
        sys.one_step_error(ji_out, 0, ji_in, x, ref);              // step 0: weight Q[5]
        double d = std::remainder((double)x[j] - (double)ref[j], 2 * kPi);  // wrap to [-pi, pi]
        double expected = (double)_Q[j] * d * d;
        check(std::fabs((double)ji_out[j] - expected) <= 1e-3 * expected + 2e-3, c.name, (double)ji_out[j], expected);
    }
    printf("angle wrap: %zu cases\n", sizeof(cases) / sizeof(cases[0]));
}

int main() {
    test_table();
    test_index_wrap();
    test_accuracy();
    test_thresholds();
    test_angle_wrap();
    printf(failures ? "UNIT_FAIL (%d)\n" : "UNIT_PASS\n", failures);
    return failures ? 1 : 0;
}
