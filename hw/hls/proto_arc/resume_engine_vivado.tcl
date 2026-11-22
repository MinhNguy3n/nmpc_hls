# Resume an engine overlay build after IP synthesis runs were killed (out of memory): reopen the
# project made by build_engine_vivado.tcl, re-run only the synthesis runs that did not finish, then
# the same implementation and export steps. Finished out-of-context IP runs are kept.
#   vivado -mode batch -nojournal -source hw/hls/proto_arc/resume_engine_vivado.tcl -tclargs <name> ?<jobs>?
# Run from the repository root (build_engine_overlay.sh <S> <K> <NAME> resume does this, after putting
# the configuration's own IPs back into hw/hls/vitis_ip_repo).
if {[llength $argv] < 1} { puts "Usage: ... -tclargs <project_name> ?<jobs>?"; exit 1 }
set pname [lindex $argv 0]
set jobs  [expr {[llength $argv] > 1 ? [lindex $argv 1] : 1}]
set script_dir [file normalize [file dirname [info script]]]
set hdl [file normalize $script_dir/../../hdl]
set out [file normalize $script_dir/../../exported_platform/$pname]
set pre [file normalize $script_dir/vivado_synth_pre.tcl]

open_project $hdl/build/$pname/$pname.xpr
update_ip_catalog -rebuild
report_ip_status -file $hdl/build/${pname}_ip_status_resume.rpt
# Stop if any IP is out of date against the repository: that would mean the repo holds another
# configuration's IPs, and re-running would mix them.
set stale {}
foreach ip [get_ips -quiet] {
    set locked 0; set upg ""
    catch {set locked [get_property IS_LOCKED $ip]}
    catch {set upg [get_property UPGRADE_VERSIONS $ip]}
    if {$locked eq "1" || $upg ne ""} { lappend stale "$ip (locked=$locked upgrade=$upg)" }
}
if {[llength $stale] > 0} { puts "ERROR: IPs locked or out of date: $stale"; exit 1 }
puts "==> IP status: [llength [get_ips -quiet]] IPs, none locked or out of date"

set redo {}
foreach r [get_runs -filter {IS_SYNTHESIS}] {
    set_property STEPS.SYNTH_DESIGN.TCL.PRE $pre $r
    if {$r ne "synth_1" && [get_property PROGRESS $r] ne "100%"} { lappend redo $r }
}
puts "==> re-running [llength $redo] IP synthesis runs: $redo"
foreach r $redo { reset_run $r }
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { reset_run synth_1 }

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
