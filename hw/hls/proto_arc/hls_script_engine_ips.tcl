############################################################
# The five solver IPs with the interleaved evaluator, for the full-solver integration (ARC 2027).
# Same flow and settings as scripts/hls_script_pso.tcl (part, clocks, vendors, version 1.0,
# m_axi_addr64=0, reset state, ip_catalog export); the only difference is nmpc_solver_costF, which is
# built from proto_arc/eval_multi.cpp (same top, ports and register map as the deployed evaluator;
# checked on the S = 10 HLS RTL: 188 identical ports and port/parameter declarations).
# Driven by build_engine_overlay.sh; environment:
#   ENGINE_CFG_INC   source/include copy with _n_S/_maxiter set (the four unchanged IPs)
#   ENGINE_EVAL_INC  gen/S<S>/include (volatile removed + model_pipe.hpp) for eval_multi.cpp
#   ENGINE_II        initiation interval of the evaluator's particle pipeline
#   ENGINE_IP_REPO   output directory for the IP zips (hw/hls/vitis_ip_repo)
#   ENGINE_ONLY      optional: build only this project (e.g. nmpc_solver_costF)
############################################################
set here     [file normalize [file dirname [info script]]]
set src_path [file normalize $here/../source/src]
set cfg_inc  $::env(ENGINE_CFG_INC)
set eval_inc $::env(ENGINE_EVAL_INC)
set ii       $::env(ENGINE_II)
set ip_path  $::env(ENGINE_IP_REPO)
set only     [expr {[info exists ::env(ENGINE_ONLY)] ? $::env(ENGINE_ONLY) : ""}]

proc createVitisPrj {prj_name prj_top} {
    global here src_path cfg_inc eval_inc ii ip_path
    open_project -reset $prj_name
    if {$prj_name eq "nmpc_solver_costF"} {
        set c_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -DSNIFFBOT_CONFIG -DPROTO_II=${ii} -I${here} -I${eval_inc} -I${eval_inc}/models -std=c++11 -Wno-unknown-pragmas"
        add_files ${here}/eval_multi.cpp -cflags $c_flags
    } else {
        set c_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -I${cfg_inc} -I${cfg_inc}/models -std=c++11 -Wno-unknown-pragmas -DSNIFFBOT_CONFIG"
        add_files ${src_path}/hls_pso.cpp -cflags $c_flags
    }
    set_top $prj_top
    open_solution "solution_system" -flow_target vivado
    set_part {xck26-sfvc784-2LV-c}
    set clock_period 10.0
    set ip_vendor "tu-dresden_turun-yliopisto"
    if {$prj_name eq "nmpc_solver_costF"} { set clock_period 8.0; set ip_vendor "tu-dresden" }
    if {$prj_name eq "nmpc_solver_fsm"}   { set ip_vendor "tu-dresden" }
    create_clock -period $clock_period -name default
    config_interface -m_axi_addr64=0
    config_rtl -reset state
    config_export -display_name $prj_name -format ip_catalog -output $ip_path/$prj_name.zip -rtl verilog -vendor $ip_vendor -version 1.0
    csynth_design
    export_design -format ip_catalog
    close_project
}

foreach {prj_name prj_top} {
    nmpc_solver_fsm             pso_fsm
    nmpc_solver_init_s          initializeParticles_set
    nmpc_solver_update_s        updateParticlesWithDuConstrains
    nmpc_solver_costF           evaluateFitnessAndDetectLocalBest
    nmpc_solver_global_minimum  detectGlobalMinimum
} {
    if {$only ne "" && $only ne $prj_name} { continue }
    createVitisPrj $prj_name $prj_top
}
