#ifndef RK4_MULTI_HPP
#define RK4_MULTI_HPP
// Day-1 prototype (ARC 2027): one RK4 prediction step for PROTO_S independent particles that
// share ONE pipelined instance of the quadrotor model.
//
// Schedule: the four RK4 stages run in an outer loop that is not flattened. Inside a stage, the
// particles are issued into the pipelined model at initiation interval PROTO_II; the stage then
// drains before the next stage starts, because stage s+1 of a particle needs stage s. Expected
// latency per stage is about depth + (S-1)*II ("burst then drain"); the HLS report gives the
// real numbers.
//
// Arithmetic is meant to be bit-identical, per particle, to System::one_step_prediction in
// source/include/hls_system.hpp (ap_fixed<32,17> top level, half model, fast sin): the same
// conversions and the same expressions in the same order. tb_rk4_multi.cpp checks this exactly.

#include "config.hpp"

#ifndef PROTO_S
#define PROTO_S 10
#endif
#ifndef PROTO_II
#define PROTO_II 8
#endif

#define PROTO_NX _Nx
#define PROTO_NU _n_U

typedef _hw_top_real proto_top_t;
typedef _hw_model_real proto_model_t;

void rk4_multi(proto_top_t x[PROTO_S * PROTO_NX],
               proto_top_t u[PROTO_S * PROTO_NU],
               proto_top_t x_next[PROTO_S * PROTO_NX]);

#endif
