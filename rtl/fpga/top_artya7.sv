// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is the top level SystemVerilog file that connects the IO on the board to the Ibex Demo System.
module top_artya7 (
  // These inputs are defined in data/pins_artya7.xdc
  input               IO_CLK,
  input               IO_RST_N,
  input  [ 3:0]       SW,
  input  [ 3:0]       BTN,
  output [ 3:0]       LED,
  output [11:0]       RGB_LED,
  output [ 3:0]       DISP_CTRL,
  input               UART_RX,
  output              UART_TX,
  input               SPI_RX,
  output              SPI_TX,
  output              SPI_SCK
/*
  ,
  inout               USB_DP,
  inout               USB_DN,
  output              USB_PULLUP_EN,
  input               USB_SENSE
*/
);
  parameter int SysClkFreq = 50_000_000;
  parameter SRAMInitFile = "";

  logic clk_sys, rst_sys_n;

  // No USB PMOD present
  wire clk_usb = 1'b0;
  wire rst_usb_n = 1'b0;
  wire USB_SENSE;
  wire USB_PULLUP_EN;

  wire usb_dp_i = 1'b0; //USB_DP;
  wire usb_dn_i = 1'b0; //USB_DN;

  wire usb_dp_o;
  wire usb_dn_o;
  wire usb_dp_en_o;
  wire usb_dn_en_o;

  assign USB_DP = usb_dp_en_o ? usb_dp_o : 1'bZ;
  assign USB_DN = usb_dn_en_o ? usb_dn_o : 1'bZ;

  // Instantiating the Ibex Demo System.
  ibex_demo_system #(
    .GpiWidth(8),
    .GpoWidth(8),
    .PwmWidth(12),
    .SRAMInitFile(SRAMInitFile)
  ) u_ibex_demo_system (
    //input
    .clk_sys_i(clk_sys),
    .rst_sys_ni(rst_sys_n),

    .clk_usb_i    (clk_usb_i),
    .rst_usb_ni   (rst_usb_ni),

    .gp_i({SW, BTN}),
    .uart_rx_i(UART_RX),

    //output
    .gp_o({LED, DISP_CTRL}),
    .pwm_o(RGB_LED),
    .uart_tx_o(UART_TX),

    .spi_rx_i(SPI_RX),
    .spi_tx_o(SPI_TX),
    .spi_sck_o(SPI_SCK),

    // Reception from USB host via transceiver
    .usb_dp_i         (usb_dp_i),
    .usb_dn_i         (usb_dn_i),
    .usb_rx_d_i       (usb_dp_i),  // We have no external transceiver

    // Transmission to USB host via transceiver
    .usb_dp_o         (usb_dp_o),
    .usb_dp_en_o      (dp_en_d2p),
    .usb_dn_o         (usb_dn_o),
    .usb_dn_en_o      (dn_en_d2p),

    // Configuration and control of USB transceiver
    .usb_sense_i      (USB_SENSE),
    .usb_dp_pullup_o  (USB_PULLUP_EN),
    .usb_dn_pullup_o  (),
    .usb_rx_enable_o  (),

    .trst_ni(1'b1),
    .tms_i(1'b0),
    .tck_i(1'b0),
    .td_i(1'b0),
    .td_o()
  );

  // Generating the system clock and reset for the FPGA.
  clkgen_xil7series clkgen(
    .IO_CLK,
    .IO_RST_N,
    .clk_sys,
    .rst_sys_n
  );

endmodule
