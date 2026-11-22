// Exact-equality testbench for the shared-pipeline RK4 prototype.
//
// Reference: System::one_step_prediction from source/include/hls_system.hpp (the fix6/3x24
// evaluator's predictor), called once per particle. Device under test: rk4_multi(), which runs
// all PROTO_S particles through one shared model. Every next-state word must match bit for bit;
// there is deliberately no tolerance (main_hls_evaluate.cpp allows 1 %, which cannot show that
// a reschedule preserved the arithmetic).
//
// Build for C simulation with the hardware arithmetic (see run_csim.sh):
//   -DHW_ARITH_CSIM -DHW_MODEL_HALF_CSIM -DUSE_FAST_SIN_COS, linked to the bit-accurate FPO model.
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "rk4_multi.hpp"
#include "hls_system.hpp"

static double uniform(double lo, double hi) {
    return lo + (hi - lo) * (std::rand() / (double)RAND_MAX);
}

// A state drawn from the ranges the closed loop visits, with some samples near the roll/pitch
// limits and some angles large enough to exercise the fast-sin wrap.
static void random_state(proto_top_t *x, int trial) {
    x[0] = uniform(-10, 10);  x[1] = uniform(-10, 12);  x[2] = uniform(-3, 3);
    const double tilt = (trial % 5 == 0) ? 0.78 : 0.5;
    x[3] = uniform(-tilt, tilt);  x[4] = uniform(-tilt, tilt);  x[5] = uniform(-3.2, 3.2);
    for (int i = 6; i < 9; i++) x[i] = uniform(-3, 3);
    for (int i = 9; i < 12; i++) x[i] = uniform(-2, 2);
}

static double tiny(void) {  // +-[1e-5, 1e-3], log-uniform: near-hover rates and tilts
    const double mag = std::pow(10.0, uniform(-5, -3));
    return (std::rand() & 1) ? mag : -mag;
}

// Input classes, cycled per trial and particle. Classes 1-4 target the region where the half
// model's small intermediate values (possible subnormals in the FP cores) and the ap_fixed
// limits matter; class 0 is the closed-loop-like random case.
//   0 random state, random control (every 7th trial: saturated +-100 controls)
//   1 exact zero state, zero control (u = 0 gives w = b = m*g/4 per rotor: hover)
//   2 tiny rates/tilts/velocities (1e-5..1e-3), hover control
//   3 tiny rates/tilts, small controls (+-1e-2)
//   4 near the ap_fixed<32,17> limits: large positions and velocities, random control
static void make_input(proto_top_t *x, proto_top_t *u, int trial, int particle) {
    const int cls = (trial + particle) % 5;
    if (cls == 0) {
        random_state(x, trial + particle);
        for (int j = 0; j < PROTO_NU; j++)
            u[j] = (trial % 7 == 0) ? ((j + particle) % 2 ? 100 : -100) : uniform(-100, 100);
        return;
    }
    for (int i = 0; i < PROTO_NX; i++) x[i] = 0;
    for (int j = 0; j < PROTO_NU; j++) u[j] = 0;
    if (cls == 1) return;
    if (cls == 2 || cls == 3) {
        x[0] = uniform(-10, 10);  x[1] = uniform(-10, 12);  x[2] = uniform(-3, 3);
        for (int i = 3; i < PROTO_NX; i++) x[i] = tiny();
        if (cls == 3)
            for (int j = 0; j < PROTO_NU; j++) u[j] = uniform(-1e-2, 1e-2);
        return;
    }
    // cls == 4: positions near +-65000 and velocities in the thousands (the top type saturates
    // at +-65536); angles and rates stay in range so the model itself remains finite.
    for (int i = 0; i < 3; i++) x[i] = (std::rand() & 1) ? uniform(6.0e4, 6.5e4) : -uniform(6.0e4, 6.5e4);
    x[3] = uniform(-0.5, 0.5);  x[4] = uniform(-0.5, 0.5);  x[5] = uniform(-3.2, 3.2);
    for (int i = 6; i < 9; i++) x[i] = uniform(-3000, 3000);
    for (int i = 9; i < 12; i++) x[i] = uniform(-2, 2);
    for (int j = 0; j < PROTO_NU; j++) u[j] = uniform(-100, 100);
}

int main(int argc, char **argv) {
    const int trials = (argc > 1) ? std::atoi(argv[1]) : 200;
    // Optional raw dump of every output word (hex bits), to compare two RTL designs directly.
    FILE *dump = (argc > 2) ? std::fopen(argv[2], "w") : NULL;
    std::srand(12345);

    System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> reference(
        _u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
        _Q, _Qf, _R, _uss, _Ts);

    static proto_top_t x[PROTO_S * PROTO_NX], u[PROTO_S * PROTO_NU], x_next[PROTO_S * PROTO_NX];
    long words = 0, mismatches = 0;
    double worst = 0.0;

    for (int t = 0; t < trials; t++) {
        for (int p = 0; p < PROTO_S; p++)
            make_input(&x[p * PROTO_NX], &u[p * PROTO_NU], t, p);
        rk4_multi(x, u, x_next);
        if (dump) {
            for (int k = 0; k < PROTO_S * PROTO_NX; k++)
                std::fprintf(dump, "%d %d %08x\n", t, k, (unsigned)x_next[k].range(31, 0).to_uint());
        }

        for (int p = 0; p < PROTO_S; p++) {
            proto_top_t state[PROTO_NX], control[PROTO_NU], want[PROTO_NX];
            for (int i = 0; i < PROTO_NX; i++) state[i] = x[p * PROTO_NX + i];
            for (int j = 0; j < PROTO_NU; j++) control[j] = u[p * PROTO_NU + j];
            reference.one_step_prediction(want, state, control);
            for (int i = 0; i < PROTO_NX; i++) {
                const proto_top_t got = x_next[p * PROTO_NX + i];
                words++;
                if (got != want[i]) {
                    const double diff = std::fabs(got.to_double() - want[i].to_double());
                    if (diff > worst) worst = diff;
                    if (mismatches < 10)
                        std::printf("MISMATCH trial %d particle %d state %d: got %.9f want %.9f\n",
                                    t, p, i, got.to_double(), want[i].to_double());
                    mismatches++;
                }
            }
        }
    }
    if (dump) std::fclose(dump);
    std::printf("rk4_multi S=%d II=%d: %ld words compared over %d trials, %ld mismatches",
                PROTO_S, PROTO_II, words, trials, mismatches);
    if (mismatches)
        std::printf(" (worst |diff| %.3g)\nFAIL\n", worst);
    else
        std::printf("\nPASS (bit-exact)\n");
    return mismatches ? 1 : 0;
}
