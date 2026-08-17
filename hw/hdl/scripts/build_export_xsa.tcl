# build_export_xsa.tcl
# Usage:
#   vivado -mode batch -source build_export_xsa.tcl -tclargs <project.xpr> ?<out_dir>? ?<jobs>?
#
# Example:
#   vivado -mode batch -source scripts/build_export_xsa.tcl \
#     -tclargs tcl_run/nmpc_solver/nmpc_solver.xpr tcl_run/nmpc_solver/export 8

proc usage {} {
  puts "Usage: vivado -mode batch -source build_export_xsa.tcl -tclargs <project.xpr> ?<out_dir>? ?<jobs>?"
}

if {$argc < 1} { usage; exit 2 }

set xpr     [file normalize [lindex $argv 0]]
set out_dir [expr {$argc >= 2 ? [file normalize [lindex $argv 1]] : [file normalize "./export"]}]
set jobs    [expr {$argc >= 3 ? [lindex $argv 2] : 8}]

if {![file exists $xpr]} {
  puts "ERROR: project not found: $xpr"
  exit 2
}

file mkdir $out_dir

# ----------------------------------------------------------------------
# Open existing project
# ----------------------------------------------------------------------
puts "==> Opening project: $xpr"
open_project $xpr
puts "==> Current project: [get_property name [current_project]]"

# ----------------------------------------------------------------------
# Validate BD + generate output products (safe even if already generated)
# ----------------------------------------------------------------------
set bd_files [get_files -quiet *.bd]
if {[llength $bd_files] > 0} {
  puts "==> Found BD(s): $bd_files"
  foreach bd $bd_files {
    puts "==> Opening BD: $bd"
    catch { open_bd_design $bd }
    set v [catch { validate_bd_design } vmsg]
    if {$v != 0} {
      puts "ERROR: validate_bd_design failed for $bd"
      puts $vmsg
      exit 10
    }
    puts "==> Generating targets for BD: $bd"
    generate_target all [get_files $bd]
  }
} else {
  puts "==> No .bd files found (ok if pure HDL)."
}

update_compile_order -fileset sources_1

# ----------------------------------------------------------------------
# Helper: run a given run and fail on errors
# ----------------------------------------------------------------------
proc run_and_check {run_name jobs} {
  if {[llength [get_runs -quiet $run_name]] == 0} {
    puts "ERROR: Run '$run_name' not found in project."
    exit 4
  }

  puts "==> Launching: $run_name (jobs=$jobs)"
  launch_runs $run_name -jobs $jobs
  wait_on_run $run_name

  set status [get_property STATUS [get_runs $run_name]]
  puts "==> $run_name STATUS: $status"

  if {[string match "*failed*" [string tolower $status]]} {
    puts "ERROR: Run '$run_name' failed."
    exit 5
  }
}

# ----------------------------------------------------------------------
# 1) Synthesis
# ----------------------------------------------------------------------
run_and_check synth_1 $jobs

# ----------------------------------------------------------------------
# 2) Implementation through bitstream
# ----------------------------------------------------------------------
puts "==> Launching impl_1 through write_bitstream"
if {[llength [get_runs -quiet impl_1]] == 0} {
  puts "ERROR: Run 'impl_1' not found in project."
  exit 4
}

launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "==> impl_1 STATUS: $impl_status"
if {[string match "*failed*" [string tolower $impl_status]]} {
  puts "ERROR: Implementation/bitstream failed."
  exit 6
}

# ----------------------------------------------------------------------
# 3) Export hardware platform (.xsa) INCLUDING bitstream
#    Note: -include_bit ensures the bitstream is packaged into the XSA.
# ----------------------------------------------------------------------
puts "==> Opening implemented design"
open_run impl_1

set proj_name [get_property name [current_project]]
set xsa_out   [file join $out_dir "${proj_name}.xsa"]

puts "==> Exporting hardware to: $xsa_out (including bitstream)"
write_hw_platform -fixed -include_bit -force -file $xsa_out

puts "==> Done."
puts "    Bitstream is under: <proj>.runs/impl_1/*.bit"
puts "    XSA exported to: $xsa_out"
exit 0
