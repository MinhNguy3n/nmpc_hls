#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include <time.h>
#include <sys/time.h>

// #ifdef __SYNTHESIS__
// #include "hls_math.h"
// typedef half _real_model;
// #else
// typedef float _real_model;
// #endif
// Definitions

#define N_DOFS 7
#define BILLION 1E9

// Hardware types (ap_fixed top level, half model) are selected for synthesis. C simulation
// built with -DHW_ARITH_CSIM uses the same ap_fixed top-level types, which C/RTL cosim
// requires: with plain float the testbench and the RTL ports disagree on the data format.
#if defined(__SYNTHESIS__) || defined(HW_ARITH_CSIM)
#include "hls_math.h"
#include "hls_stream.h"
#include "ap_fixed.h"

// typedef half _hw_top_real;
typedef ap_fixed<32,17,AP_RND_INF,AP_SAT_SYM> _hw_top_real; // round to +/-inf and simmetrical saturation
#if defined(__SYNTHESIS__) || defined(HW_MODEL_HALF_CSIM)
typedef half _hw_model_real;
#else
// HW_ARITH_CSIM: top-level types match the RTL ports (needed for cosim); the model stays
// float because the C++ half emulation cannot operate on the model's volatile ports.
// HW_MODEL_HALF_CSIM selects half anyway; it only builds from a header copy without
// `volatile` (app/kria-pynq/validate_hw_run.py makes one for its half-model build).
typedef float _hw_model_real;
#endif
// typedef ap_fixed<32,16,AP_RND_INF,AP_SAT_SYM> _hw_model_real;
typedef hls::stream<_hw_top_real> _rand_real_stream;
#else
typedef float _hw_top_real;
typedef float _hw_model_real;
#endif

// #define SNIFFBOT_CONFIG

// #ifdef PSO_CONFIG
#define _KPSO 1
#define _stable_zero 1
#define _n_S 3
#define _maxiter 24
#define _max_v 30
#define _w0 0.9
#define _wf 0.1 // Drone
// #define _wf 0.4 // Inverted Pendulum
#define _c1 2.1 
#define _c2 1.0
#define _threshold 1e-2
#define _stop_criteria 0
#define _slope (_wf-_w0)/_maxiter
const _hw_top_real rand_min = 0.0;
const _hw_top_real rand_max = 1.0;
// const int rand_seed[] = {
    // 84680,
    // 577726,
    // 273600,
    // 804402,
    // 747952
// };
// #endif

#ifdef INVERTED_PENDULUM_CONFIG

    #define _Nh     25       // Prediction Horizon
    #define _Nu     25       // Control Horizon
    #define _Nx     4         // Number of States
    #define _n_U    1         // Number of Inputs

    const _hw_top_real  _Ts = 0.1;
    const _hw_top_real  _u_max[]  = {50.0};
    const _hw_top_real  _u_min[]  = {-50.0};
    const _hw_top_real  _du_max[] = {50.0};
    const _hw_top_real  _uss[] = {0.0};

    #define _Parametrization 0
    const _hw_top_real _Lambda = 10;
    const _hw_top_real _q_param = 10;
    const _hw_top_real _pmax[] = {1.0};
    const _hw_top_real _pmin[] = {-1.0};

    const unsigned short _controlled_state[] = {1, 1, 2, 0};
    const _hw_top_real _state_upper_limits[] = {1, 1e3, 1e3, 1e3} ;
    const _hw_top_real _state_lower_limits[] = {-1, -1e3, -1e3, -1e3} ;
    const _hw_top_real _Q[] = { 2  , 0.015, 1  , 0.0};
    const _hw_top_real _Qf[] = {.1, 0.015, 0.1, 0.0};
    const _hw_top_real _R[] = {.001};

    #define _Rising_Time 0
    const _hw_top_real _tr[] =  {0, 0, 0, 0};
    // float x_ss[] = {0.4, 0.3, 0.2, 0.1};
#elif defined(SNIFFBOT_CONFIG)

    #define _Nh 25
    #define _Nu 25
    #define _Nx 12
    #define _n_U 4
    const _hw_top_real  _Ts = 0.05;
    const _hw_top_real  _u_max[] =  {100, 100, 100, 100};
    const _hw_top_real  _u_min[] =  {-100, -100, -100, -100};
    const _hw_top_real  _du_max[] =  {
        20, 
        20, 
        20, 
        20
    };
    const _hw_top_real  _uss[] = {0.0, 0.0, 0.0, 0.0};

    #define  _Parametrization 0
    const _hw_top_real  _Lambda = 5;
    const _hw_top_real  _q_param = 2;
    const _hw_top_real  _pmax[] =  {1, 1, 1, 1};
    const _hw_top_real  _pmin[] =  {-1, -1, -1, -1};

    const _hw_top_real drone_pi = 3.14159265359;
    // Roll/pitch limits are tightened by this margin in the cost so the remaining model error
    // (fast sin, half precision: <= ~1.5 mrad) cannot turn into a real violation of +-pi/4.
    const _hw_top_real _angle_limit_margin = 0.01;
    const unsigned short _controlled_state[] = {1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0};
    const _hw_top_real  _state_upper_limits[_Nx] =  { 
         1e3,  
         1e3,  
         1e3,  
         drone_pi*(_hw_top_real).25 - _angle_limit_margin,  
         drone_pi*(_hw_top_real).25 - _angle_limit_margin,  
         1e3,  
         1e3, 1e3, 1e3, 1e3, 1e3, 1e3
    };
    const _hw_top_real  _state_lower_limits[_Nx] =  {
        -1e3, 
        -1e3, 
        -1e3, 
        -drone_pi*(_hw_top_real).25 + _angle_limit_margin, 
        -drone_pi*(_hw_top_real).25 + _angle_limit_margin, 
        -1e3, 
        -1e3, -1e3, -1e3, -1e3, -1e3, -1e3};
    const _hw_top_real  _Q[] =  {
        1, 
        1, 
        1, 
        3, 
        3, 
        3, 
        0, 0, 0, 0, 0, 0};
    const _hw_top_real  _Qf[] =  {
        10, 
        10, 
        10, 
        30, 
        30, 
        30, 
        0, 0, 0, 0, 0, 0};
    const _hw_top_real  _R[] =  {
        0.0005, 
        0.0005, 
        0.0005, 
        0.0005
    };

    #define  _Rising_Time 0
    const _hw_top_real _tr[] =  {10, 10, 10, 0, 0, 8, 0, 0, 0, 0, 0, 0};
#elif defined(UNICYCLE_CONFIG)

    // Unicycle (Husky A300 SIL model): state [x (m), y (m), psi (rad)], control [v, omega] in
    // normalized units (+-100; the model maps them to m/s and rad/s, models/hls_unicycle.hpp).
    // Sources: docs/husky_a300/unicycle_mpc_pso_model.md (model note) sections 7.1-7.4 and 8;
    // app/husky/sil/pso.py CostConfig and sil/controller.py CoreConfig (branch husky-unicycle-mpc).
    // PROVISIONAL: every limit and weight below comes from the A300 Gazebo SIL envelope or the
    // open weight comparison; none is an A200 or robot value. Evaluator latency does not depend
    // on these values. The PSO parameters (_n_S, _maxiter, _max_v, _w0, _wf, _c1, _c2) are the
    // shared definitions above, the same values as sil/pso.py PsoConfig.
    #define _Nh 25
    #define _Nu 25      // = _Nh: the engine reads u_k as cg[p][k*n_U + j]
    #define _Nx 3
    #define _n_U 2
    const _hw_top_real  _Ts = 0.05;
    const _hw_top_real  _u_max[] =  {100, 100};
    const _hw_top_real  _u_min[] =  {-100, -100};
    // PROVISIONAL (A300 SIL envelope v in [0, 0.3] m/s, |omega| <= 0.75 rad/s; accel 2 / decel 4 m/s^2,
    // alpha 4 rad/s^2): du_max = 100 min(rate) Ts / half-range per dimension (model note 7.4),
    // = unicycle_model.normalized_slew_limits(). The shared PSO velocity clamp _max_v = 30 binds in v.
    const _hw_top_real  _du_max[] =  {
        66.6666667,
        26.6666667
    };
    // z_v(v = 0) = -100 (v_min = 0), omega 0: the effort term pulls toward standstill (model note 8)
    const _hw_top_real  _uss[] = {-100.0, 0.0};

    #define  _Parametrization 0
    // Not used by the evaluator; mirrored from the quadrotor block so code naming them compiles.
    const _hw_top_real  _Lambda = 5;
    const _hw_top_real  _q_param = 2;
    const _hw_top_real  _pmax[] =  {1, 1};
    const _hw_top_real  _pmin[] =  {-1, -1};

    // psi uses the wrapped residual (controlled_state 2, commit 8c61f1b); the host solves in the
    // local frame (x0 = y0 = psi0 = 0), so |psi| <= Nh omega_max Ts = 0.94 rad here.
    const unsigned short _controlled_state[] = {1, 1, 2};
    // No psi limit (it would switch the cost to the 1e4 penalty): +-1e3 on every state.
    const _hw_top_real  _state_upper_limits[_Nx] =  {1e3, 1e3, 1e3};
    const _hw_top_real  _state_lower_limits[_Nx] =  {-1e3, -1e3, -1e3};
    // PROVISIONAL weights = sil/pso.py CostConfig: q (1, 1, 1), qf (10, 10, 3) [1/m^2, 1/m^2, 1/rad^2];
    // r (0, 5e-5) per normalized unit^2 (R_v = 0, model note 7.3). 5e-5 is below the ap_fixed<32,17>
    // resolution (2^-15 = 3.05e-5) and is stored as 6.10e-5 (2 LSB); the 5e-6 candidate of the open
    // weight comparison would round to 0. Weight comparison open (GPT-6 review 2026-11-19).
    const _hw_top_real  _Q[] =  {1, 1, 1};
    const _hw_top_real  _Qf[] =  {10, 10, 3};
    const _hw_top_real  _R[] =  {0.0, 0.00005};

    #define  _Rising_Time 0
    const _hw_top_real _tr[] =  {0, 0, 0};
#endif

#if defined(_POSIX_MONOTONIC_CLOCK)
/*  The identifier for the system-wide monotonic clock, which is defined
 *  as a clock whose value cannot be set via clock_settime() and which
 *  cannot have backward clock jumps. */

    #define CLOCK_ID CLOCK_MONOTONIC
#else
    #define CLOCK_ID CLOCK_REALTIME
#endif

const int _pragma_Nh = _Nh;
const int _pragma_Nu = _Nu;
const int _pragma_Nx = _Nx;
const int _pragma_n_U = _n_U;

#if !(defined(__SYNTHESIS__) || defined(HW_ARITH_CSIM))
const _hw_top_real H_MAX = 3.4028234664e30;
#else
const _hw_top_real H_MAX = 60000.0;
#endif

#endif
