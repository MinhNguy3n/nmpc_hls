// Self-checking C/RTL co-simulation testbench for evaluateFitnessAndDetectLocalBest.
//
// The evaluator runs the cost function once per particle through FIFOs that persist
// in hardware across calls. Calling it twice exercises that state over 2 * n_S cost
// evaluations: an unbalanced FIFO deadlocks the RTL (C simulation cannot show it).
//
// Returns 0 on success so cosim reports PASS.
#include <cstdio>
#include "hls_pso.hpp"

static const int part_size = _pso_n_S * _pso_Nu * _pso_n_U;
static const _pso_hw_real find_init = 30000.0;

int main() {
    static _pso_hw_real x[part_size];
    static _pso_hw_real y[part_size];
    static _pso_hw_real x_curr[_pso_Nx];
    static _pso_hw_real xref[_pso_Nx * _pso_Nu];
    static _pso_hw_real find[2][_pso_n_S];

    for (int i = 0; i < part_size; i++)
        x[i] = _uss[i % _pso_n_U];
    for (int i = 0; i < _pso_Nx; i++)
        x_curr[i] = 0;
    x_curr[0] = 7.0;
    x_curr[1] = 10.0;
    for (int i = 0; i < _pso_Nx * _pso_Nu; i++)
        xref[i] = 0;

    for (int call = 0; call < 2; call++) {
        for (int i = 0; i < part_size; i++)
            y[i] = 0;
        for (int i = 0; i < _pso_n_S; i++)
            find[call][i] = find_init;
        evaluateFitnessAndDetectLocalBest(x, y, x_curr, xref, find[call]);
    }

    int errors = 0;
    for (int i = 0; i < _pso_n_S; i++) {
        printf("find[%d] = %f / %f\n", i, (double)find[0][i], (double)find[1][i]);
        if (find[0][i] != find[1][i]) {
            printf("ERROR: particle %d cost differs between calls\n", i);
            errors++;
        }
        if (!(find[1][i] < find_init)) {
            printf("ERROR: particle %d cost was not improved\n", i);
            errors++;
            continue;
        }
        for (int j = 0; j < _pso_Nu * _pso_n_U; j++) {
            int idx = i * _pso_Nu * _pso_n_U + j;
            if (y[idx] != x[idx]) {
                printf("ERROR: local_y[%d] was not updated from local_x\n", idx);
                errors++;
                break;
            }
        }
    }

    printf(errors ? "TB_FAIL (%d errors)\n" : "TB_PASS\n", errors);
    return errors;
}
