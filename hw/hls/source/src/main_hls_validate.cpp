// Host-side validation helper for hardware solver runs (built with g++, not Vitis HLS).
//
//   main_hls_validate cost  < x_curr[Nx] xref[Nx*Nh] u[Nu*n_U]
//       Prints the cost J(u) computed by the C evaluator (evaluateFitnessAndDetectLocalBest).
//   main_hls_validate solve <seed> < x_curr[Nx] xref[Nx*Nh] u_curr[n_U] last_best[Nu*n_U]
//       Runs the software PSO (execute) with srand(seed) and prints its control sequence and J.
//
// Build with -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS for the hardware top-level arithmetic
// (ap_fixed<32,17>, fast sin/cos), or without for the float reference. Run one solve per
// process: updateParticlesWithDuConstrains keeps its inertia weight in a static variable.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "hls_pso.hpp"

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
    fprintf(stderr, "usage: %s cost | solve <seed>  (values on stdin)\n", argv[0]);
    return 1;
}
