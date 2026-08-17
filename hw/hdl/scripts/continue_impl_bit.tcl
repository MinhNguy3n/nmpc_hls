# continue_impl_bit.tcl
# Usage: vivado -mode batch -source continue_impl_bit.tcl -tclargs <project.xpr> ?<jobs>?

set xpr  [lindex $argv 0]
set jobs [expr {$argc >= 2 ? [lindex $argv 1] : 8}]

open_project $xpr

# Sanity check: synthesis must be complete
set s_status [get_property STATUS [get_runs synth_1]]
puts "synth_1 status: $s_status"
if {![string match "*Complete*" $s_status]} {
    puts "ERROR: synth_1 is not complete. Run synthesis first."
    exit 1
}

# Continue with implementation all the way to bitstream
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1

set i_status [get_property STATUS [get_runs impl_1]]
puts "impl_1 status: $i_status"
if {[string match "*failed*" [string tolower $i_status]]} {
    puts "ERROR: impl_1 failed."
    exit 2
}

puts "Done. Bitstream is typically in: <project>.runs/impl_1/*.bit"
exit 0
