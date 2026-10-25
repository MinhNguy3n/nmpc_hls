# Build the ILA debug design (nmpc_solver_v2_debug.tcl) as a fresh Vivado 2025.2 project and export
# its XSA, leaving the original 2025.1 project in tcl_run/nmpc_solver_v2 untouched. The HLS IPs come
# from hw/hls/vitis_ip_repo, so rebuild and unpack them first.
#
# Usage, from anywhere:
#   vivado -mode batch -nojournal -source hw/hdl/scripts/build_debug_variant.tcl \
#     -tclargs <project_name> ?<jobs>?
# Project: hw/hdl/build/<project_name>; XSA, timing/utilization reports and debug probes:
# hw/exported_platform/<project_name>/.
if {$argc < 1} {
  puts "Usage: vivado -mode batch -source build_debug_variant.tcl -tclargs <project_name> ?<jobs>?"
  exit 2
}
set pname [lindex $argv 0]
set jobs  [expr {$argc >= 2 ? [lindex $argv 1] : 6}]
set hdl   [file normalize [file join [file dirname [info script]] ..]]
set out   [file normalize "$hdl/../exported_platform/$pname"]

file mkdir $hdl/build
cd $hdl/build
if {[file exists $hdl/build/$pname]} { error "Build dir exists: $hdl/build/$pname" }

set ::argv [list --origin_dir $hdl --project_name $pname]
set ::argc [llength $::argv]
source $hdl/nmpc_solver_v2_debug.tcl

# The imported incremental checkpoint was produced by 2025.1; do not reuse it.
set_property -quiet INCREMENTAL_CHECKPOINT {} [get_runs synth_1]
set_property -quiet AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]

update_ip_catalog -rebuild
set bd [get_files design_nmpc_solver.bd]
open_bd_design $bd
report_ip_status -name ip_status -file $hdl/build/${pname}_ip_status.rpt
validate_bd_design
save_bd_design
generate_target all $bd

launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
puts "==> synth_1: [get_property STATUS [get_runs synth_1]]"
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synth_1 failed" }

launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
puts "==> impl_1: [get_property STATUS [get_runs impl_1]]"
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "impl_1 failed" }

open_run impl_1
file mkdir $out
report_timing_summary -max_paths 10 -file $out/timing_summary.rpt
report_utilization -file $out/utilization.rpt
write_debug_probes -force $out/debug_nets.ltx
write_hw_platform -fixed -include_bit -force -file $out/nmpc_solver_v2.xsa
puts "==> DONE: $out"
exit 0
