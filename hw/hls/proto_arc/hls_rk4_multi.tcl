############################################################
# Synthesis (and optional C/RTL cosim) of the shared-pipeline RK4 prototype.
# Same part, 8 ns clock, interface and reset settings as the nmpc_solver_costF IP in
# scripts/hls_script_pso.tcl, so latency and resources are comparable with the fix6/3x24
# evaluator report.
#
# Run from any directory (one per parallel run); sources are found next to this script:
#   PROTO_S=10 PROTO_II=8 [PROTO_COSIM=1] vitis-run --mode hls --tcl hls_rk4_multi.tcl
############################################################
set here [file normalize [file dirname [info script]]]
set S  $::env(PROTO_S)
set II $::env(PROTO_II)
set cosim 0
if {[info exists ::env(PROTO_COSIM)]} { set cosim $::env(PROTO_COSIM) }
# PROTO_DUT=rk4_ref.cpp synthesizes the original serial predictor behind the same interface.
set dut rk4_multi.cpp
if {[info exists ::env(PROTO_DUT)]} { set dut $::env(PROTO_DUT) }
set dump [file normalize [pwd]/rtl_out.txt]
set trials 100
if {[info exists ::env(PROTO_TRIALS)]} { set trials $::env(PROTO_TRIALS) }
set incl "-I${here} -I${here}/gen/include -I${here}/gen/include/models"
set defs "-D__VITIS__ -DPSO_CONFIG -DSNIFFBOT_CONFIG -DUSE_FAST_SIN_COS -DPROTO_S=${S} -DPROTO_II=${II}"
set c_flags "${defs} ${incl} -std=c++11 -Wno-unknown-pragmas"
# The testbench needs the hardware arithmetic in C simulation (see run_csim.sh). HLS csim/cosim
# link their own half library, so HW_MODEL_HALF_CSIM is used for the reference as well.
set tb_flags "${c_flags} -DHW_ARITH_CSIM -DHW_MODEL_HALF_CSIM"

open_project -reset proj_rk4_S${S}_II${II}
set_top rk4_multi
add_files ${here}/${dut} -cflags $c_flags -csimflags $tb_flags
add_files -tb ${here}/tb_rk4_multi.cpp -cflags $tb_flags -csimflags $tb_flags
add_files -tb ${here}/../source/src/aux_functions.cpp -cflags $tb_flags -csimflags $tb_flags

open_solution -reset "sol" -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 8.0 -name default
config_interface -m_axi_addr64=0
config_rtl -reset state

csynth_design
if {$cosim} {
    csim_design -argv "${trials}"
    cosim_design -rtl verilog -trace_level none -argv "${trials} ${dump}"
}
close_project
