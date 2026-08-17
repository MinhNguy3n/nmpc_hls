set script_dir [file dirname [file normalize [info script]]]
set hdl_dir [file normalize "$script_dir/.."]
set build_root [file normalize "$hdl_dir/build"]
set project_name "nmpc_solver_kv260"
set project_dir [file normalize "$build_root/$project_name"]
set output_dir [file normalize "$hdl_dir/../exported_platform"]
set output_xsa [file normalize "$output_dir/design_nmpc_solver_wrapper.xsa"]

if {[file exists $project_dir]} {
  error "Build directory already exists: $project_dir. Remove or archive it before rebuilding."
}

file mkdir $build_root
file mkdir $output_dir
cd $build_root

set ::origin_dir_loc [file normalize "$script_dir/designs"]
set ::user_project_name $project_name
source [file normalize "$script_dir/designs/nmpc_solver.tcl"]

validate_bd_design
save_bd_design

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property STATUS [get_runs synth_1]] ne "synth_design Complete!"} {
  error "Synthesis failed: [get_property STATUS [get_runs synth_1]]"
}

launch_runs impl_1 -jobs 4 -to_step write_bitstream
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
  error "Implementation failed: [get_property STATUS [get_runs impl_1]]"
}

write_hw_platform -fixed -include_bit -force -file $output_xsa
puts "KV260 hardware platform written to $output_xsa"
