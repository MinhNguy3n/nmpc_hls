// Self-checking C/RTL co-simulation testbench for evaluateFitnessAndDetectLocalBest.
//
// The evaluator runs the cost function once per particle through FIFOs that persist
// in hardware across calls. Calling it twice exercises that state over 2 * n_S cost
// evaluations: an unbalanced FIFO deadlocks the RTL (C simulation cannot show it).
//
// A third call checks the angle residual: yaw 3.0 rad against a -3.0 rad reference (a difference
// across +-pi) with matching positions, so the wrapped yaw error dominates the cost. Each
// particle's cost must match the C cost function (System, not replaced by RTL in cosim) within
// 1 %, which covers the half-precision RTL model against the float C model but not a wrong wrap.
//
// Returns 0 on success so cosim reports PASS.
#include <cmath>
#include <cstdio>
#include "hls_pso.hpp"
#include "hls_system.hpp"

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

    // Angle wrap across +-pi.
    static _pso_hw_real find_wrap[_pso_n_S];
    x_curr[5] = 3.0;
    for (int k = 0; k < _pso_Nu; k++) {
        xref[k * _pso_Nx + 0] = 7.0;
        xref[k * _pso_Nx + 1] = 10.0;
        xref[k * _pso_Nx + 5] = -3.0;
    }
    for (int i = 0; i < _pso_n_S; i++)
        find_wrap[i] = find_init;
    evaluateFitnessAndDetectLocalBest(x, y, x_curr, xref, find_wrap);
    for (int i = 0; i < _pso_n_S; i++) {
        System<_pso_hw_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> reference(
            _u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
            _Q, _Qf, _R, _uss, _Ts);
        _pso_hw_real expected;
        reference.nmpc_cost_function_topflow(x_curr, &x[i * _pso_Nu * _pso_n_U], xref, &expected);
        double got = (double)find_wrap[i], want = (double)expected;
        printf("wrap: find[%d] = %f, C cost %f\n", i, got, want);
        if (!(std::fabs(got - want) <= 0.01 * std::fabs(want) + 1e-3)) {
            printf("ERROR: particle %d angle-wrap cost differs from the C cost function\n", i);
            errors++;
        }
    }

    printf(errors ? "TB_FAIL (%d errors)\n" : "TB_PASS\n", errors);
    return errors;
}
