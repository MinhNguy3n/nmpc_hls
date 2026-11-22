// Shared testbench for evaluateFitnessAndDetectLocalBest: the deployed serial version
// (source/src/hls_pso.cpp) or the shared-pipeline version (eval_multi.cpp), whichever is linked.
// It dumps the raw bits of local_find and local_y after every call; the dumps of two designs
// (C or RTL) are compared with cmp. Exposing J: calls 0 and 2 of each trial reset local_find to
// the largest value, so every particle with J below it updates and local_find[p] == J. Call 1
// keeps local_find, to exercise the compare branch with persistent state.
//   tb_eval TRIALS DUMPFILE [SEED [CALLS [STOREDFILE]]]   (short cosims with different seeds keep xsim
//   memory bounded; CALLS < 3 stops each trial early, and its dump is a prefix of the 3-call dump for 1 trial)
// Stored-cost mode (STOREDFILE given, 1 trial, 1 call, dump labelled c1): the update-and-retain test.
// Starting from trial 0, call 0 of the same seed, each particle's stored best is preset to its own call-0
// cost J0[p] (read from STOREDFILE, a C dump of that call) and its stored position to its own call-0
// controls. The call then evaluates rotated controls: particle 0 keeps its own (J == stored, so the strict
// compare must retain it), particles 1..S-1 take the controls of the next particle in the cycle
// 1 -> 2 -> ... -> S-1 -> 1. With distinct costs a cycle has at least one update and one retain, and the
// stored costs are real, non-saturated values. The inputs do not depend on the DUT's outputs.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "hls_pso.hpp"

typedef _pso_hw_real top_t;
#define TB_S _pso_n_S
#define TB_NX _pso_Nx
#define TB_NU _pso_n_U
#define TB_NH _pso_Nh
#define TB_PART (_pso_Nu * _pso_n_U)

static double uniform(double lo, double hi) { return lo + (hi - lo) * (std::rand() / (double)RAND_MAX); }
static double tiny(void) {
    const double mag = std::pow(10.0, uniform(-5, -3));
    return (std::rand() & 1) ? mag : -mag;
}

#ifdef UNICYCLE_CONFIG
// Unicycle [x, y, psi]: random pose (psi over the full circle, so the reference psi = psi +- drift
// below also crosses +-pi and exercises the wrapped residual), exact zero (the host's local frame),
// or tiny values.
static void make_state(top_t *x, int cls) {
    for (int i = 0; i < TB_NX; i++) x[i] = 0;
    if (cls == 1) return;
    if (cls == 2) {
        for (int i = 0; i < TB_NX; i++) x[i] = tiny();
        return;
    }
    x[0] = uniform(-10, 10);  x[1] = uniform(-10, 12);  x[2] = uniform(-3.2, 3.2);
}
#else
// Current state: closed-loop-like, exact zero, or tiny rates/tilts (near hover).
static void make_state(top_t *x, int cls) {
    for (int i = 0; i < TB_NX; i++) x[i] = 0;
    if (cls == 1) return;
    x[0] = uniform(-10, 10);  x[1] = uniform(-10, 12);  x[2] = uniform(-3, 3);
    if (cls == 2) {
        for (int i = 3; i < TB_NX; i++) x[i] = tiny();
        return;
    }
    const double tilt = (std::rand() % 4 == 0) ? 0.78 : 0.5;
    x[3] = uniform(-tilt, tilt);  x[4] = uniform(-tilt, tilt);  x[5] = uniform(-3.2, 3.2);
    for (int i = 6; i < 9; i++) x[i] = uniform(-3, 3);
    for (int i = 9; i < 12; i++) x[i] = uniform(-2, 2);
}
#endif

// A particle's control sequence: random, saturated, hover (all zero), small, or a slow ramp
// that respects |du| <= 20 like the PSO's update.
static void make_controls(top_t *u, int cls) {
    for (int k = 0; k < TB_PART; k++) {
        double v;
        switch (cls) {
        case 0: v = uniform(-100, 100); break;
        case 1: v = (std::rand() & 1) ? 100 : -100; break;
        case 2: v = 0; break;
        case 3: v = uniform(-1e-2, 1e-2); break;
        default: v = (k < TB_NU) ? uniform(-20, 20) : (double)u[k - TB_NU] + uniform(-5, 5); break;
        }
        u[k] = v;
    }
}

int main(int argc, char **argv) {
    const int trials = (argc > 1) ? std::atoi(argv[1]) : 10;
    FILE *dump = (argc > 2) ? std::fopen(argv[2], "w") : NULL;
    std::srand((argc > 3) ? (unsigned)std::atoi(argv[3]) : 4242u);
    const int calls = (argc > 4) ? std::atoi(argv[4]) : 3;
    const char *stored_file = (argc > 5) ? argv[5] : NULL;

    static top_t local_x[TB_S * TB_PART], local_y[TB_S * TB_PART];
    static top_t x_curr[TB_NX], xref[TB_NX * TB_NH], local_find[TB_S];
    top_t big;
    big.range() = 0x7FFFFFFF;  // largest ap_fixed<32,17> value

    long improved = 0, evaluated = 0;
    if (stored_file) {
        top_t j0[TB_S];
        int got = 0;
        FILE *sf = std::fopen(stored_file, "r");
        if (!sf) { std::printf("cannot open %s\n", stored_file); return 1; }
        char line[256];
        while (std::fgets(line, sizeof line, sf)) {
            int p; unsigned v;
            if (std::sscanf(line, "t0 c0 find %d %x", &p, &v) == 2 && p >= 0 && p < TB_S) {
                j0[p].range(31, 0) = v;
                got++;
            }
        }
        std::fclose(sf);
        if (got != TB_S) { std::printf("%s: expected %d t0 c0 find lines, got %d\n", stored_file, TB_S, got); return 1; }
        // Same random sequence as trial 0, call 0 of the normal mode.
        make_state(x_curr, 0);
        for (int i = 0; i < TB_NH; i++)
            for (int j = 0; j < TB_NX; j++) {
                double r = 0;
                if (j < 3) r = x_curr[j].to_double() + 0.05 * (i + 1) * ((j % 2) ? 1 : -1);
                if (j == 5) r = 0.01 * i;
                xref[i * TB_NX + j] = r;
            }
        static top_t base[TB_S * TB_PART];
        for (int p = 0; p < TB_S; p++) make_controls(&base[p * TB_PART], p % 5);
        int updates = 0, retains = 0, saturated = 0;
        for (int p = 0; p < TB_S; p++) {
            const int q = (p == 0) ? 0 : (p == TB_S - 1 ? (TB_S > 2 ? 1 : p) : p + 1);
            for (int k = 0; k < TB_PART; k++) {
                local_y[p * TB_PART + k] = base[p * TB_PART + k];
                local_x[p * TB_PART + k] = base[q * TB_PART + k];
            }
            local_find[p] = j0[p];
            if (j0[p] == big) saturated++;
            // Expected decision with the C costs: J(controls of q) = J0[q].
            if (j0[q] < j0[p]) updates++; else retains++;
        }
        evaluateFitnessAndDetectLocalBest(local_x, local_y, x_curr, xref, local_find);
        int changed = 0;
        for (int p = 0; p < TB_S; p++) if (local_find[p] != j0[p]) changed++;
        if (dump) {
            for (int p = 0; p < TB_S; p++)
                std::fprintf(dump, "t0 c1 find %d %08x\n", p, (unsigned)local_find[p].range(31, 0).to_uint());
            for (int k = 0; k < TB_S * TB_PART; k++)
                std::fprintf(dump, "t0 c1 y %d %08x\n", k, (unsigned)local_y[k].range(31, 0).to_uint());
            std::fclose(dump);
        }
        std::printf("tb_eval S=%d stored-cost mode: expected %d updates and %d retains (C costs), %d stored "
                    "costs saturated; %d local bests changed\n", TB_S, updates, retains, saturated, changed);
        return 0;
    }
    for (int t = 0; t < trials; t++) {
        make_state(x_curr, t % 3);
        // Reference: a slow drift from the current position (or zero), angles near zero.
        for (int i = 0; i < TB_NH; i++)
            for (int j = 0; j < TB_NX; j++) {
                double r = 0;
                if (t % 3 != 1 && j < 3) r = x_curr[j].to_double() + 0.05 * (i + 1) * ((j % 2) ? 1 : -1);
                if (t % 3 != 1 && j == 5) r = 0.01 * i;
                xref[i * TB_NX + j] = r;
            }
        for (int k = 0; k < TB_S * TB_PART; k++) local_y[k] = 0;

        for (int call = 0; call < calls; call++) {
            if (call != 1)
                for (int p = 0; p < TB_S; p++) local_find[p] = big;
            for (int p = 0; p < TB_S; p++)
                make_controls(&local_x[p * TB_PART], (t + call + p) % 5);
            top_t before[TB_S];
            for (int p = 0; p < TB_S; p++) before[p] = local_find[p];

            evaluateFitnessAndDetectLocalBest(local_x, local_y, x_curr, xref, local_find);

            for (int p = 0; p < TB_S; p++) {
                evaluated++;
                if (local_find[p] != before[p]) improved++;
            }
            if (dump) {
                for (int p = 0; p < TB_S; p++)
                    std::fprintf(dump, "t%d c%d find %d %08x\n", t, call, p,
                                 (unsigned)local_find[p].range(31, 0).to_uint());
                for (int k = 0; k < TB_S * TB_PART; k++)
                    std::fprintf(dump, "t%d c%d y %d %08x\n", t, call, k,
                                 (unsigned)local_y[k].range(31, 0).to_uint());
            }
        }
    }
    if (dump) std::fclose(dump);
    std::printf("tb_eval S=%d: %d trials x %d calls, %ld particle evaluations, %ld local-best updates\n",
                TB_S, trials, calls, evaluated, improved);
    return 0;
}
