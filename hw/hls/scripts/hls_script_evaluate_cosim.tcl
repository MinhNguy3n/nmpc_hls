############################################################
# C simulation + C/RTL co-simulation of evaluateFitnessAndDetectLocalBest
# (the nmpc_solver_costF IP) with the self-checking testbench main_hls_evaluate.cpp.
# Uses the same synthesis settings as hls_script_pso.tcl for nmpc_solver_costF.
#
# Run from hw/hls:
#   vitis-run --mode hls --tcl scripts/hls_script_evaluate_cosim.tcl
############################################################
set workspace [pwd]
set src_path ${workspace}/source/src
set incl_path ${workspace}/source/include
set model_flag "-DSNIFFBOT_CONFIG"

set c_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -I${incl_path} -I${incl_path}/models -std=c++11 -Wno-unknown-pragmas ${model_flag}"
# -DHW_ARITH_CSIM is required: plain csim uses float for the top-level arrays, while the RTL
# ports are ap_fixed<32,17>, so cosim would hand the RTL misinterpreted bit patterns.
set tb_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -DHW_ARITH_CSIM -I${incl_path} -I${incl_path}/models -std=c++11 -Wno-unknown-pragmas ${model_flag}"

open_project -reset nmpc_solver_costF_cosim
set_top evaluateFitnessAndDetectLocalBest

add_files ${src_path}/hls_pso.cpp -cflags $c_flags -csimflags $tb_flags
add_files -tb ${src_path}/main_hls_evaluate.cpp -cflags $tb_flags -csimflags $tb_flags
add_files -tb ${src_path}/hls_nonlinear_solver.cpp -cflags $tb_flags -csimflags $tb_flags
add_files -tb ${src_path}/aux_functions.cpp -cflags $tb_flags -csimflags $tb_flags

open_solution -reset "solution_system" -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 8.0 -name default
config_interface -m_axi_addr64=0
config_rtl -reset state

csim_design
csynth_design
cosim_design -rtl verilog -trace_level none

close_project
