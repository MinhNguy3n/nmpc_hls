############################################################
# Synthesis + C/RTL cosim of an evaluateFitnessAndDetectLocalBest variant with tb_eval.cpp,
# using the deployed costF settings (scripts/hls_script_evaluate_cosim.tcl / hls_script_pso.tcl):
# xck26, 8 ns, m_axi_addr64=0, reset state, csim with -DHW_ARITH_CSIM.
#   EVAL_DUT=new|ref PROTO_S=10 PROTO_II=8 PROTO_TRIALS=4 vitis-run --mode hls --tcl hls_eval.tcl
# new: eval_multi.cpp with gen/S<S>/include.  ref: the deployed source/src/hls_pso.cpp with
# gen/ref_S<S>/include (source/include with only _n_S changed; identical for S = 3).
############################################################
set here [file normalize [file dirname [info script]]]
set dut $::env(EVAL_DUT)
set S  $::env(PROTO_S)
set II $::env(PROTO_II)
set trials 4
if {[info exists ::env(PROTO_TRIALS)]} { set trials $::env(PROTO_TRIALS) }
set cosim 1
if {[info exists ::env(PROTO_COSIM)]} { set cosim $::env(PROTO_COSIM) }
set dump [file normalize [pwd]/rtl_out.txt]
set seed 4242
set calls 3
if {[info exists ::env(PROTO_CALLS)]} { set calls $::env(PROTO_CALLS) }
# Stored-cost (update/retain) mode: a C dump of trial 0, call 0 of the same seed (see tb_eval.cpp).
set stored ""
if {[info exists ::env(PROTO_STORED)]} { set stored [file normalize $::env(PROTO_STORED)] }
if {[info exists ::env(PROTO_SEED)]} { set seed $::env(PROTO_SEED) }
# The deployed evaluator's xsim grows by >1 GB per call; deadlock detection is the suspect.
set cosim_opts ""
if {[info exists ::env(PROTO_NODEADLOCK)] && $::env(PROTO_NODEADLOCK)} { set cosim_opts "-disable_deadlock_detection" }
set src ${here}/../source/src
# PROTO_MODEL=unicycle: the UNICYCLE_CONFIG header sets (gen_eval_headers.sh with the same variable).
set model sniffbot
if {[info exists ::env(PROTO_MODEL)]} { set model $::env(PROTO_MODEL) }
if {$model eq "unicycle"} {
    set P uni_; set cfg_def UNICYCLE_CONFIG
} elseif {$model eq "sniffbot"} {
    set P ""; set cfg_def SNIFFBOT_CONFIG
} else {
    error "unknown PROTO_MODEL $model"
}
if {$dut eq "new"} {
    set inc ${here}/gen/${P}S${S}/include
} else {
    set inc ${here}/gen/${P}ref_S${S}/include
}
set c_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -D${cfg_def} -DPROTO_II=${II} -I${here} -I${inc} -I${inc}/models -std=c++11 -Wno-unknown-pragmas"
set tb_flags "${c_flags} -DHW_ARITH_CSIM"

open_project -reset proj_eval_${P}${dut}_S${S}_II${II}
set_top evaluateFitnessAndDetectLocalBest
if {$dut eq "new"} {
    add_files ${here}/eval_multi.cpp -cflags $c_flags -csimflags $tb_flags
} else {
    add_files ${src}/hls_pso.cpp -cflags $c_flags -csimflags $tb_flags
    add_files -tb ${src}/hls_nonlinear_solver.cpp -cflags $tb_flags -csimflags $tb_flags
}
add_files -tb ${here}/tb_eval.cpp -cflags $tb_flags -csimflags $tb_flags
add_files -tb ${src}/aux_functions.cpp -cflags $tb_flags -csimflags $tb_flags

open_solution -reset "sol" -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 8.0 -name default
config_interface -m_axi_addr64=0
config_rtl -reset state

csynth_design
if {$cosim} {
    cosim_design -rtl verilog -trace_level none {*}$cosim_opts -argv "${trials} ${dump} ${seed} ${calls} ${stored}"
}
# PROTO_EXPORT=1: Vivado out-of-context synthesis and implementation of the HLS RTL at the 8 ns HLS
# clock (export_design -flow impl); reports in sol/impl/report/verilog/.
if {[info exists ::env(PROTO_EXPORT)] && $::env(PROTO_EXPORT)} {
    export_design -flow impl -rtl verilog -format ip_catalog
}
close_project
