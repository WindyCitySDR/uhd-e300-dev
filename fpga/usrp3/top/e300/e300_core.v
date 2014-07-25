//
// Copyright 2013-2014 Ettus Research LLC
//

module e300_core
(
  // bus interfaces
  input             bus_clk,
  input             bus_rst,

  //axi fifo out from data mover
  input [63:0]      h2s_tdata,
  input             h2s_tlast,
  input             h2s_tvalid,
  output            h2s_tready,

  //axi fifo in to data mover
  output [63:0]     s2h_tdata,
  output            s2h_tlast,
  output            s2h_tvalid,
  input             s2h_tready,

  // radio interfaces
  input             radio_clk,
  input             radio_rst,
  input [31:0]      rx_data0,
  output [31:0]     tx_data0,
  input [31:0]      rx_data1,
  output [31:0]     tx_data1,

  // gpio controls
  output [31:0]     ctrl_out0,
  output [31:0]     ctrl_out1,

  // settings bus to control global registers
  input [31:0]      set_data,
  input [7:0]       set_addr,
  input             set_stb,

  output reg [31:0] rb_data,

  // pps signals -- muxing happens toplevel
  output [1:0]      pps_select,
  input             pps,

  // front panel (internal) gpio
  inout [5:0]       fp_gpio,


  output [31:0]     debug
);

  localparam SR_CORE_READBACK = 8'd0;
  localparam SR_CORE_PPS_SEL  = 8'd4;
  localparam SR_CORE_TEST     = 8'd28;

  localparam RB32_CORE_TEST     = 3'd0;
  localparam RB32_CORE_PPS_SEL  = 3'd1;
  localparam RB32_CORE_COMPAT   = 3'd2;
  localparam RB32_CORE_GITHASH  = 3'd3;

  wire [2:0] rb_addr;
  wire [31:0] rb_test;


  setting_reg
  #( .my_addr(SR_CORE_READBACK),
     .awidth(8), .width(3),
     .at_reset(2'h0)
  ) sr_readback_addr
  (
    .clk(bus_clk), .rst(bus_rst),
    .strobe(set_stb), .addr(set_addr),
    .in(set_data), .out(rb_addr),
    .changed()
  );

  setting_reg
  #( .my_addr(SR_CORE_TEST),
     .awidth(8), .width(32),
     .at_reset(32'h0)
  ) sr_test
  (
    .clk(bus_clk), .rst(bus_rst),
    .strobe(set_stb), .addr(set_addr),
    .in(set_data), .out(rb_test),
    .changed()
  );

  setting_reg
  #(
    .my_addr(SR_CORE_PPS_SEL),
    .awidth(8), .width(2),
    .at_reset(2'h0)
  ) sr_pps_sel
  (
    .clk(bus_clk), .rst(bus_rst),
    .strobe(set_stb), .addr(set_addr),
    .in(set_data), .out(pps_select),
    .changed()
  );

  always @(*)
    case(rb_addr)
      RB32_CORE_TEST    : rb_data <= rb_test;
      RB32_CORE_PPS_SEL : rb_data <= {30'd0, pps_select};
      RB32_CORE_COMPAT  : rb_data <= {8'hAC, 8'h0, 8'h0, 8'h1};
      RB32_CORE_GITHASH : rb_data <= 32'h`GIT_HASH;
      default           : rb_data <= 64'hdeadbeef;
    endcase

    ////////////////////////////////////////////////////////////////////
    // routing logic
    ////////////////////////////////////////////////////////////////////
    wire [63:0] ro_tdata [1:0]; wire ro_tlast [1:0]; wire ro_tvalid [1:0]; wire ro_tready [1:0];
    wire [63:0] ri_tdata [1:0]; wire ri_tlast [1:0]; wire ri_tvalid [1:0]; wire ri_tready [1:0];

    wire [63:0] vheader;
    wire [1:0] vdest = vheader[3:2]; //bottom 2 for radio dest, next 2 is which radio

    axi_demux4 #(.ACTIVE_CHAN(4'b0011), .WIDTH(64)) radio_demux
     (.clk(bus_clk), .reset(bus_rst), .clear(1'b0),
      .header(vheader), .dest(vdest),
      .i_tdata(h2s_tdata), .i_tlast(h2s_tlast), .i_tvalid(h2s_tvalid), .i_tready(h2s_tready),
      .o0_tdata(ro_tdata[0]), .o0_tlast(ro_tlast[0]), .o0_tvalid(ro_tvalid[0]), .o0_tready(ro_tready[0]),  //R0
      .o1_tdata(ro_tdata[1]), .o1_tlast(ro_tlast[1]), .o1_tvalid(ro_tvalid[1]), .o1_tready(ro_tready[1]),  //R1
      .o2_tdata(), .o2_tlast(), .o2_tvalid(), .o2_tready(1'b1),
      .o3_tdata(), .o3_tlast(), .o3_tvalid(), .o3_tready(1'b1));

    axi_mux4 #(.PRIO(0), .WIDTH(64), .BUFFER(1)) radio_mux
     (.clk(bus_clk), .reset(bus_rst), .clear(1'b0),
      .i0_tdata(ri_tdata[0]), .i0_tlast(ri_tlast[0]), .i0_tvalid(ri_tvalid[0]), .i0_tready(ri_tready[0]), //R0
      .i1_tdata(ri_tdata[1]), .i1_tlast(ri_tlast[1]), .i1_tvalid(ri_tvalid[1]), .i1_tready(ri_tready[1]), //R1
      .i2_tdata(), .i2_tlast(), .i2_tvalid(1'b0), .i2_tready(),
      .i3_tdata(), .i3_tlast(), .i3_tvalid(1'b0), .i3_tready(),
      .o_tdata(s2h_tdata), .o_tlast(s2h_tlast), .o_tvalid(s2h_tvalid), .o_tready(s2h_tready));

    ////////////////////////////////////////////////////////////////////
    // radio instantiation
    ////////////////////////////////////////////////////////////////////
    wire [63:0] tx_tdata_bo, tx_tdata_bi;
    wire tx_tlast_bo, tx_tvalid_bo, tx_tready_bo;
    wire tx_tlast_bi, tx_tvalid_bi, tx_tready_bi;
    wire [15:0] tx_xfer_occ;
    wire tx_can_xfer = (tx_xfer_occ == 16'b0);

    axi_fifo #(.WIDTH(65), .SIZE(11)) axi_fifo_tx_packet_buff
    (
        .clk(bus_clk), .reset(bus_rst), .clear(1'b0),
        .i_tdata({tx_tlast_bo, tx_tdata_bo}), .i_tvalid(tx_tvalid_bo), .i_tready(tx_tready_bo),
        .o_tdata({tx_tlast_bi, tx_tdata_bi}), .o_tvalid(tx_tvalid_bi), .o_tready(tx_tready_bi),
        .occupied(tx_xfer_occ)
    );

    radio radio0
    (
        //radio domain stuff
        .radio_clk(radio_clk), .radio_rst(radio_rst),

        //not connected
        .rx(rx_data0), .tx(tx_data0),
        .db_gpio(ctrl_out0),
        .db_gpio2(ctrl_out1),
        .fp_gpio(fp_gpio),
        .sen(), .sclk(), .mosi(), .miso(),
        .misc_outs(), .leds(),

        //bus clock domain and fifos
        .bus_clk(bus_clk), .bus_rst(bus_rst),
        .in_tdata(ro_tdata[0]), .in_tlast(ro_tlast[0]), .in_tvalid(ro_tvalid[0]), .in_tready(ro_tready[0]),
        .out_tdata(ri_tdata[0]), .out_tlast(ri_tlast[0]), .out_tvalid(ri_tvalid[0]), .out_tready(ri_tready[0]),

        //tx buffering -- used for insertion of axi_fifo_tx_packet_buff
        .tx_tdata_bo(tx_tdata_bo), .tx_tlast_bo(tx_tlast_bo), .tx_tvalid_bo(tx_tvalid_bo), .tx_tready_bo(tx_tready_bo),
        .tx_tdata_bi(tx_tdata_bi), .tx_tlast_bi(tx_tlast_bi), .tx_tvalid_bi(tx_tvalid_bi), .tx_tready_bi(tx_tready_bi),
        .pps(pps)
    );


endmodule // e300_core
