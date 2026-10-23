// Self-checking C/RTL co-simulation testbench for updateParticlesWithDuConstrains.
//
// Two calls driven by a deterministic random stream (the RTL reads the same values through
// its AXI-Stream port). Checks the PSO constraints on every element: positions within
// [u_min, u_max], each step within du_max of the previous control (starting from u_curr),
// |v| <= max_v, and that the random stream is fully consumed. The printed checksums let the
// C run and the RTL post-check be compared.
//
// Requires -DHW_ARITH_CSIM so C simulation exposes the hardware random port.
// Returns 0 on success so cosim reports PASS.
#include <cstdio>
#include "hls_pso.hpp"

static const int part = _pso_Nu * _pso_n_U;
static const int total = _pso_n_S * part;

int main() {
    static _pso_hw_real x[total], y[total], v[total];
    static _pso_hw_real global_min[part], u_curr[_pso_n_U];
    static _pso_hw_real du_max[_pso_n_U], du_min[_pso_n_U], u_max[_pso_n_U], u_min[_pso_n_U];

    for (int k = 0; k < _pso_n_U; k++) {
        du_max[k] = _du_max[k];
        du_min[k] = -_du_max[k];
        u_max[k] = _u_max[k];
        u_min[k] = _u_min[k];
        u_curr[k] = 0;
    }
    for (int i = 0; i < total; i++) {
        x[i] = (i * 7) % 41 - 20;
        y[i] = (i * 13) % 37 - 18;
        v[i] = (i * 5) % 11 - 5;
    }
    for (int j = 0; j < part; j++)
        global_min[j] = (j * 3) % 17 - 8;

    int errors = 0;
    for (int call = 0; call < 2; call++) {
        _rand_real_stream rand_port;
        for (int r = 0; r < 2 * total; r++)
            rand_port.write((_pso_hw_real)((r * 29 % 97) / 97.0));

        updateParticlesWithDuConstrains(u_curr, x, y, v, global_min, du_max, du_min, u_max, u_min, rand_port);

        if (!rand_port.empty()) {
            printf("ERROR: random stream not fully consumed\n");
            errors++;
        }

        double sum_x = 0, sum_v = 0;
        for (int i = 0; i < _pso_n_S; i++) {
            for (int k = 0; k < _pso_n_U; k++) {
                _pso_hw_real prev = u_curr[k];
                for (int j = 0; j < _pso_Nu; j++) {
                    int idx = i * part + j * _pso_n_U + k;
                    _pso_hw_real xi = x[idx], vi = v[idx];
                    sum_x += (double)xi;
                    sum_v += (double)vi;
                    if (xi > u_max[k] || xi < u_min[k]) {
                        printf("ERROR: x[%d] = %f outside [u_min, u_max]\n", idx, (double)xi);
                        errors++;
                    }
                    if (xi - prev > du_max[k] || xi - prev < du_min[k]) {
                        printf("ERROR: x[%d] steps %f from previous control\n", idx, (double)(xi - prev));
                        errors++;
                    }
                    if (vi > (_pso_hw_real)_max_v || vi < (_pso_hw_real)(-_max_v)) {
                        printf("ERROR: v[%d] = %f exceeds max_v\n", idx, (double)vi);
                        errors++;
                    }
                    prev = xi;
                }
            }
        }
        printf("call %d: checksum x = %.5f v = %.5f\n", call, sum_x, sum_v);
    }

    printf(errors ? "TB_FAIL (%d errors)\n" : "TB_PASS\n", errors);
    return errors;
}
