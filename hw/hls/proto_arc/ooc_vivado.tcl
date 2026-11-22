# Single-process Vivado out-of-context synthesis + implementation of an HLS RTL directory (non-project
# flow), to keep memory low (the HLS export flow runs vitis_hls + two Vivado processes, > 7 GB here).
#   vivado -mode batch -source ooc_vivado.tcl -tclargs <hls sol/syn/verilog dir> <out dir> [period ns]
set vdir [file normalize [lindex $argv 0]]
set out  [file normalize [lindex $argv 1]]
set period [expr {[llength $argv] > 2 ? [lindex $argv 2] : 8.0}]
set part xck26-sfvc784-2LV-c
set top evaluateFitnessAndDetectLocalBest
file mkdir $out $out/ip
# synth_design otherwise spawns up to 7 child Vivado processes (synth.maxThreads), which exceeds the
# memory cap on this 13 GB PC; threads (general.maxThreads) share one process and are kept.
set_param synth.maxThreads 1
create_project -in_memory -part $part
cd $vdir   ;# the ROM models use $readmemh("./<name>.dat")
foreach t [lsort [glob -nocomplain *_ip.tcl]] {
    set fh [open $t]; set body [read $fh]; close $fh
    eval [string map [list "create_ip " "create_ip -dir $out/ip "] $body]
}
if {[llength [get_ips]] > 0} {
    # Synthesise the IP in-line with the top (no separate OOC IP runs, no extra processes).
    foreach ip [get_ips] { set_property GENERATE_SYNTH_CHECKPOINT false [get_files [get_property IP_FILE $ip]] }
    generate_target all [get_ips]
}
read_verilog [glob *.v]
set xdc $out/clock.xdc
set fh [open $xdc w]; puts $fh "create_clock -name ap_clk -period $period \[get_ports ap_clk\]"; close $fh
read_xdc -mode out_of_context $xdc
synth_design -top $top -part $part -mode out_of_context
report_utilization -file $out/util_synth.rpt
report_timing_summary -file $out/timing_synth.rpt
opt_design
place_design
route_design
report_utilization -file $out/util_route.rpt
report_timing_summary -max_paths 5 -file $out/timing_route.rpt
puts "OOC_DONE WNS=[get_property SLACK [get_timing_paths -max_paths 1 -setup]] period=$period"
