// Self-checking C/RTL co-simulation testbench for detectGlobalMinimum.
//
// Two calls with different iteration indices and best particles: checks that the minimum
// fitness lands in local_bestfitness[interaction], that local_global_min receives the best
// particle's local_y row, and that no other bestfitness entry is written.
//
// Returns 0 on success so cosim reports PASS.
#include <cstdio>
#include "hls_pso.hpp"

static const int part = _pso_Nu * _pso_n_U;
static const _pso_hw_real sentinel = -1.0;

int main() {
    static _pso_hw_real find[_pso_n_S];
    static _pso_hw_real bestfitness[_pso_maxiter];
    static _pso_hw_real global_min[part];
    static _pso_hw_real y[_pso_n_S * part];

    for (int i = 0; i < _pso_maxiter; i++)
        bestfitness[i] = sentinel;
    for (int i = 0; i < _pso_n_S; i++)
        for (int j = 0; j < part; j++)
            y[i * part + j] = i * 100 + j;

    // Scaled to the PSO size so they stay valid indices (10 particles x 100 iterations: 3/7 and 6/2).
    const int interaction[2] = {3 % _pso_maxiter, 7 % _pso_maxiter};
    const int best[2] = {6 % _pso_n_S, 2 % _pso_n_S};
    int errors = 0;

    for (int call = 0; call < 2; call++) {
        for (int i = 0; i < _pso_n_S; i++)
            find[i] = 1000 + 10 * i;
        find[best[call]] = 50 + call;

        detectGlobalMinimum(find, bestfitness, global_min, interaction[call], y);

        printf("call %d: bestfitness[%d] = %f (expected %f)\n", call, interaction[call],
               (double)bestfitness[interaction[call]], (double)find[best[call]]);
        if (bestfitness[interaction[call]] != find[best[call]]) {
            printf("ERROR: wrong minimum fitness\n");
            errors++;
        }
        for (int j = 0; j < part; j++) {
            if (global_min[j] != y[best[call] * part + j]) {
                printf("ERROR: local_global_min[%d] = %f, expected particle %d\n", j,
                       (double)global_min[j], best[call]);
                errors++;
                break;
            }
        }
    }

    for (int i = 0; i < _pso_maxiter; i++) {
        if (i != interaction[0] && i != interaction[1] && bestfitness[i] != sentinel) {
            printf("ERROR: bestfitness[%d] was overwritten\n", i);
            errors++;
        }
    }

    printf(errors ? "TB_FAIL (%d errors)\n" : "TB_PASS\n", errors);
    return errors;
}
