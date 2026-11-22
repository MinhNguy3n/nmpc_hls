// Reference design for RTL-vs-RTL comparison: the SAME interface as rk4_multi, implemented with
// the original fix6/3x24 predictor (System::one_step_prediction, with its own pragmas and
// operator caps), one particle after another. Synthesized and co-simulated with
// tb_rk4_multi.cpp, its RTL output is the current hardware's answer for the same inputs.
#include "rk4_multi.hpp"
#include "hls_system.hpp"

void rk4_multi(proto_top_t x[PROTO_S * PROTO_NX],
               proto_top_t u[PROTO_S * PROTO_NU],
               proto_top_t x_next[PROTO_S * PROTO_NX]) {
    System<_hw_top_real, _hw_model_real, _Nh, _Nx, _n_U, _Nu> sys(
        _u_max, _u_min, _du_max, _controlled_state, _state_upper_limits, _state_lower_limits,
        _Q, _Qf, _R, _uss, _Ts);
serial: for (int p = 0; p < PROTO_S; p++) {
        proto_top_t state[PROTO_NX], control[PROTO_NU], next[PROTO_NX];
        for (int i = 0; i < PROTO_NX; i++) state[i] = x[p * PROTO_NX + i];
        for (int j = 0; j < PROTO_NU; j++) control[j] = u[p * PROTO_NU + j];
        sys.one_step_prediction(next, state, control);
        for (int i = 0; i < PROTO_NX; i++) x_next[p * PROTO_NX + i] = next[i];
    }
}
