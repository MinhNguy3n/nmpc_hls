#ifndef HLS_NONSOLV_HPP
#define HLS_NONSOLV_HPP

/***************************** Include Files *********************************/
#include "config.hpp"
#include "aux_functions.hpp"

/************************** Class Definitions ****************************/

// typedef _hw_top_real _pso_real;

int nonlinear_solver_wrapper(
    volatile float *x_curr,//[_Nx], 
    volatile float *u_curr,//[_n_U], 
    int iteration, 
    volatile float *last_best,//[_Nu*_n_U], 
    volatile float *xref,//[_Nu*_Nx], 
    // volatile float *uref,//[_n_U], 
    // volatile float *xss,//[_Nx],
    // volatile float *uss,//[_n_U], 
    
    float *new_best,//[_Nu*_n_U],
    float *J
#ifdef PSO_RD_ANY
    , volatile float *rd_aux//[_RD_AUX_LEN]: PSO redesign (config.hpp layout)
#endif
);

#endif