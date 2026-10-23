// axis_rand_fanout
// Shares one free-running random-number stream between initializeParticles and
// updateParticlesWithDuConstrains. The PSO FSM never runs both consumers at the
// same time, so an AXI4-Stream broadcaster (which waits for every output to be
// ready) deadlocks. Here the source advances when ANY consumer is ready; the idle
// consumer may see TDATA change without a handshake, which is harmless for a
// random source.
`timescale 1ns / 1ps

module axis_rand_fanout #(
    parameter integer DATA_WIDTH = 32
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m0_axis:m1_axis" *)
    input  wire                  aclk,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *)
    input  wire                  s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TREADY" *)
    output wire                  s_axis_tready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m0_axis TDATA" *)
    output wire [DATA_WIDTH-1:0] m0_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m0_axis TVALID" *)
    output wire                  m0_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m0_axis TREADY" *)
    input  wire                  m0_axis_tready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m1_axis TDATA" *)
    output wire [DATA_WIDTH-1:0] m1_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m1_axis TVALID" *)
    output wire                  m1_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m1_axis TREADY" *)
    input  wire                  m1_axis_tready
);

    assign m0_axis_tdata  = s_axis_tdata;
    assign m1_axis_tdata  = s_axis_tdata;
    assign m0_axis_tvalid = s_axis_tvalid;
    assign m1_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready  = m0_axis_tready | m1_axis_tready;

endmodule
