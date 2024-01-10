// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Ibex demo system top level for the Sonata board
module top_sonata (
  input              mainclk,

  output logic [7:0] userled,
  output logic       led_bootok,
  output logic       led_halted,
  output logic       led_cheri,
  output logic       led_legacy,
  output logic [8:0] cherierr,

  input  logic [4:0] nav_sw,
  input  logic [7:0] user_sw,

  output logic lcd_rst,
  output logic lcd_dc,
  output logic lcd_copi,
  output logic lcd_clk,
  output logic lcd_cs,

  output logic ser0_tx,
  input  logic ser0_rx,

  // Status input from USB transceiver
  input  logic  USRUSB_VBUSDETECT,

  // Control of USB transceiver
  output logic  USRUSB_SOFTCN,
  // Configure the USB transceiver for Full Speed operation.
  output logic  USRUSB_SPD,

  // Reception from USB host via transceiver
  input  logic  USRUSB_V_P,
  input  logic  USRUSB_V_N,
  input  logic  USRUSB_RCV,

  // Transmission to USB host via transceiver
  output logic  USRUSB_VPO,
  output logic  USRUSB_VMO,

  // Always driven configuration signals to the USB transceiver.
  output logic  USRUSB_OE,
  output logic  USRUSB_SUS,

  input  logic       tck_i,
  input  logic       tms_i,
  input  logic       td_i,
  output logic       td_o
);
  parameter int SysClkFreq = 50_000_000;
  parameter SRAMInitFile = "";

  logic top_rst_n;
  logic mainclk_buf;
  logic clk_sys, rst_sys_n;
  logic clk_usb, rst_usb_n;
  logic [7:0] reset_counter;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;

  initial begin
    reset_counter = 0;
  end

  always_ff @(posedge mainclk_buf) begin
    if (reset_counter != 8'hff) begin
      reset_counter <= reset_counter + 8'd1;
    end
  end

  assign top_rst_n = reset_counter < 8'd5   ? 1'b1 :
                     reset_counter < 8'd200 ? 1'b0 :
                                              1'b1;

  assign led_bootok = 1'b1;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~nav_sw;
  assign user_sw_n = ~user_sw;

  assign USRUSB_SPD = 1'b1;

  logic dp_en_d2p;
  logic rx_enable_d2p;
  assign USRUSB_OE  = !dp_en_d2p;  // Active low Output Enable
  assign USRUSB_SUS = !rx_enable_d2p;

  // No LCD backlight FPGA IO on v0.2 board, so leave this unconnected
  logic lcd_backlight;

  ibex_demo_system #(
    .SysClkFreq(SysClkFreq),
    .GpiWidth(13),
    .GpoWidth(12),
    .PwmWidth(12),
    .SRAMInitFile(SRAMInitFile)
  ) u_ibex_demo_system (
    //input
    .clk_sys_i    (clk_sys),
    .rst_sys_ni   (rst_sys_n),

    .clk_usb_i    (clk_usb),
    .rst_usb_ni   (rst_usb_n),

    .gp_i({user_sw_n, nav_sw_n[4:0]}),
    .gp_o({userled, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i(ser0_rx),
    .uart_tx_o(ser0_tx),

    .pwm_o({cherierr, led_legacy, led_cheri, led_halted}),

    .spi_rx_i(1'b0),
    .spi_tx_o(lcd_copi),
    .spi_sck_o(lcd_clk),

    // Reception from USB host via transceiver
    .usb_dp_i         (USRUSB_V_P),
    .usb_dn_i         (USRUSB_V_N),
    .usb_rx_d_i       (USRUSB_RCV),

    // Transmission to USB host via transceiver
    .usb_dp_o         (USRUSB_VPO),
    .usb_dp_en_o      (dp_en_d2p),
    .usb_dn_o         (USRUSB_VMO),
    .usb_dn_en_o      (),

    // Configuration and control of USB transceiver
    .usb_sense_i      (USRUSB_VBUSDETECT),
    .usb_dp_pullup_o  (USRUSB_SOFTCN),
    .usb_dn_pullup_o  (),
    .usb_rx_enable_o  (rx_enable_d2p),

    .trst_ni(rst_sys_n),
    .tms_i,
    .tck_i,
    .td_i,
    .td_o
  );

  // Produce the system clock and 48 MHz USB clock from 25 MHz Sonata board clock
  clkgen_sonata  #(
    .SysClkFreq(SysClkFreq)
  ) clkgen(
    .IO_CLK(mainclk),
    .IO_CLK_BUF(mainclk_buf),
    .IO_RST_N(top_rst_n),
    .clk_sys,
    .rst_sys_n,
    .clk_usb,
    .rst_usb_n
  );

endmodule
