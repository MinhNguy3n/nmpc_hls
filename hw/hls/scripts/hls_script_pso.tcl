############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
############################################################
proc createVitisPrj {prj_name prj_top model_flag} {
    set workspace [pwd]
    #set workspace [file dirname $workspace]
    
    set ip_path "${workspace}/vitis_ip_repo"
    set src_path ${workspace}/source/src
    set incl_path ${workspace}/source/include
    # set module_file "hls_nonlinear_solver"
    set main_name "main_hls_pso"

    open_project -reset -upgrade $prj_name

    set c_flags "-D__VITIS__ -DPSO_CONFIG -DUSE_FAST_SIN_COS -I${incl_path} -I${incl_path}/models -std=c++11 -Wno-unknown-pragmas ${model_flag}"
    set csim_tb_flags "-D__VITIS__ -DPSO_CONFIG -I${incl_path} -I${incl_path}/models -std=c++11 -DDEBUG_FILE -DPRINT_TO_TERMINAL -Wno-unknown-pragmas ${model_flag}"

    set_top $prj_top
    # foreach file [glob -dir $incl_path *.hpp] {
    #     add_files $file
    # }

    add_files ${src_path}/hls_pso.cpp -cflags $c_flags -csimflags $csim_tb_flags

    add_files -tb ${src_path}/hls_nonlinear_solver.cpp -cflags $c_flags -csimflags $csim_tb_flags
    add_files -tb ${src_path}/${main_name}.cpp -cflags $csim_tb_flags -csimflags $csim_tb_flags
    add_files -tb ${src_path}/aux_functions.cpp -cflags $csim_tb_flags -csimflags $csim_tb_flags

    open_solution "solution_system" -flow_target vivado

    set_part {xck26-sfvc784-2LV-c}
    set clock_period 10.0
    set ip_vendor "tu-dresden_turun-yliopisto"
    if {$prj_name eq "nmpc_solver_fsm" || $prj_name eq "nmpc_solver_costF"} {
        set clock_period 8.0
        if {$prj_name eq "nmpc_solver_fsm"} {
            set clock_period 10.0
        }
        set ip_vendor "tu-dresden"
    }
    create_clock -period $clock_period -name default

    # config_compile -pipeline_loops 6
    config_interface -m_axi_addr64=0
    config_rtl -reset state

    config_export -display_name $prj_name -format ip_catalog -output $ip_path/$prj_name.zip -rtl verilog -vendor $ip_vendor -version 1.0

    # config_core DSP48 -latency 4

    #set arg_str "${workspace}/source/config/sniffbot/project_config.txt ${workspace}/source/config/sniffbot/simulation_config_ring.txt"
    #csim_design -argv $arg_str -clean -O -profile
    csynth_design
    # cosim_design -O -rtl vhdl
    export_design -format ip_catalog

    close_project
}

set prj_name_list {
    "nmpc_solver_fsm"
    "nmpc_solver_init_s"
    "nmpc_solver_update_s"
    "nmpc_solver_costF"
    "nmpc_solver_global_minimum"
}

set prj_top_list {
    "pso_fsm"
    "initializeParticles_set"
    "updateParticlesWithDuConstrains"
    "evaluateFitnessAndDetectLocalBest"
    "detectGlobalMinimum"
}

#set flag "-DINVERTED_PENDULUM_CONFIG"
set flag "-DSNIFFBOT_CONFIG"

# foreach prj_name $prj_name_list prj_top $prj_top_list{
#     createVitisPrj {prj_name prj_top extra flag}
# }

foreach {prj_name} $prj_name_list {prj_top} $prj_top_list {
    createVitisPrj $prj_name $prj_top $flag
}