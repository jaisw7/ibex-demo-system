// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// TODO: drive I2C1 rather than I2C0?
`define DRIVE_I2C1
`define DRIVE_RPIHAT
//`define DRIVE_RPIID

// TODO: drive USBDEV traffic over PMOD rather than USRUSB?
//`define PMOD_USBDEV

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
  input  logic       USRUSB_VBUSDETECT,

  // Control of USB transceiver
  output logic       USRUSB_SOFTCN,
  // Configure the USB transceiver for Full Speed operation.
  output logic       USRUSB_SPD,

  // Reception from USB host via transceiver
  input  logic       USRUSB_V_P,
  input  logic       USRUSB_V_N,
  input  logic       USRUSB_RCV,

  // Transmission to USB host via transceiver
  output logic       USRUSB_VPO,
  output logic       USRUSB_VMO,

  // Always driven configuration signals to the USB transceiver.
  output logic       USRUSB_OE,
  output logic       USRUSB_SUS,

  // I2C buses
`ifdef DRIVE_RPIHAT
  output logic       SCL0,
  output logic       SDA0,
  output logic       SCL1,
  output logic       SDA1,
`else
  inout  logic       SCL0,  // Shared between Arduino and QWIIC (J1)
  inout  logic       SDA0,

  inout  logic       SCL1,  // QWIIC only (J7)
  inout  logic       SDA1,
`endif

  // PMOD interfaces

  // PMOD0 is driving a '7-segment LED' with 8 wires.
  output logic       PMOD0_1,
  output logic       PMOD0_2,
  output logic       PMOD0_3,
  output logic       PMOD0_4,
  output logic       PMOD0_5,
  output logic       PMOD0_6,
  output logic       PMOD0_7,
  output logic       PMOD0_8,

  // PMOD1 is (attempting to drive|driving) USB
  output logic       PMOD1_1,
  output logic       PMOD1_2,
  output logic       PMOD1_3,
  output logic       PMOD1_4,
  output logic       PMOD1_5,
  output logic       PMOD1_6,
  output logic       PMOD1_7,
  output logic       PMOD1_8,

  // RaspberryPi HAT
  inout  logic       RPH_G2_SDA,
  inout  logic       RPH_G3_SCL,

  inout  logic       RPH_G1,      // ID_SC - I2C for HAT ID EEPROM
  inout  logic       RPH_G0,      // ID_SD

  input  logic       tck_i,
  input  logic       tms_i,
  input  logic       td_i,
  output logic       td_o
);
  parameter int SysClkFreq = 50_000_000;
  parameter SRAMInitFile = "";

  logic top_rst_n;
  logic mainclk_buf;
  logic clk_sys,  rst_sys_n;
  logic clk_peri, rst_peri_n;
  logic clk_usb,  rst_usb_n;
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

  // Two digit 7-segment display
  logic [11:0] gp_o;
  assign PMOD0_8 = gp_o[7];  // CA selector.
  assign PMOD0_7 = gp_o[6] ? 1'b0 : 1'bZ;  // C
  assign PMOD0_6 = gp_o[5] ? 1'b0 : 1'bZ;  // TL
  assign PMOD0_5 = gp_o[4] ? 1'b0 : 1'bZ;  // BL
  assign PMOD0_4 = gp_o[3] ? 1'b0 : 1'bZ;  // B
  assign PMOD0_3 = gp_o[2] ? 1'b0 : 1'bZ;  // BR
  assign PMOD0_2 = gp_o[1] ? 1'b0 : 1'bZ;  // TR
  assign PMOD0_1 = gp_o[0] ? 1'b0 : 1'bZ;  // T

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

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;
  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Feed all I2C traffic to the other port as a pure output (always enabled) since this allows
  // us very easily to attach the logic analyser.
`ifdef DRIVE_I2C1
  // Drive I2C1 bus (J7) - mirror its traffic on J1 (I2C0)
  logic scl0_out;
  logic sda0_out;

// ID EPROM on HAT
`ifdef DRIVE_RPIID
  always_ff @(posedge clk_sys) begin
    scl0_out <= RPH_G1;
    sda0_out <= RPH_G0;
  end
  // Open Drain drivers onto I2C bus.
  assign RPH_G1 = scl1_oe ? scl1_o : 1'bZ;
  assign RPH_G0 = sda1_oe ? sda1_o : 1'bZ;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = RPH_G1;
  wire sda1_i = RPH_G0;
`else
// General I2C bus on HAT
`ifdef DRIVE_RPIHAT
  always_ff @(posedge clk_sys) begin
    scl0_out <= RPH_G3_SCL;
    sda0_out <= RPH_G2_SDA;
  end
  // Open Drain drivers onto I2C bus.
  assign RPH_G3_SCL = scl1_oe ? scl1_o : 1'bZ;
  assign RPH_G2_SDA = sda1_oe ? sda1_o : 1'bZ;

  // Make it clear who is driving the I2C bus
  assign SCL1 = scl1_oe;
  assign SDA1 = sda1_oe;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = RPH_G3_SCL;
  wire sda1_i = RPH_G2_SDA;
`else
  always_ff @(posedge clk_sys) begin
    scl0_out <= SCL1;
    sda0_out <= SDA1;
  end
  // Open Drain drivers onto I2C bus.
  assign SCL1 = scl1_oe ? scl1_o : 1'bZ;
  assign SDA1 = sda1_oe ? sda1_o : 1'bZ;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = SCL1;
  wire sda1_i = SDA1;
`endif
`endif
  // Output only, for logic analyser
  assign SCL0 = scl0_out;
  assign SDA0 = sda0_out;
`else
  logic scl1_out;
  logic sda1_out;

  // Driving I2C0
  always_ff @(posedge clk_sys) begin
    scl1_out <= SCL0;
    sda1_out <= SDA0;
  end
  // Open Drain drivers onto I2C bus.
  assign SCL0 = scl0_oe ? scl0_o : 1'bZ;
  assign SDA0 = sda0_oe ? sda0_o : 1'bZ;

  // Output only, for logic analyser
  assign SCL1 = scl1_out;
  assign SDA1 = sda1_out;

  wire scl0_i = SCL0;
  wire sda0_i = SDA0;
  wire scl1_i = SCL1;
  wire sda1_i = SDA1;
`endif

  assign {userled, lcd_backlight, lcd_dc, lcd_rst, lcd_cs} = 'b0;

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

    .clk_peri_i   (clk_peri),
    .rst_peri_ni  (rst_peri_n),

    .gp_i({user_sw_n, nav_sw_n[4:0]}),
//    .gp_o({userled, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),
    .gp_o(gp_o),

    .uart_rx_i(ser0_rx),
    .uart_tx_o(ser0_tx),

    .pwm_o({cherierr, led_legacy, led_cheri, led_halted}),

    .spi_rx_i(1'b0),
    .spi_tx_o(lcd_copi),
    .spi_sck_o(lcd_clk),

`ifdef PMOD_USBDEV
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
`else
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
`endif

    // I2C bus 0
    .i2c0_scl_i       (scl0_i),
    .i2c0_scl_o       (scl0_o),
    .i2c0_scl_en_o    (scl0_oe),
    .i2c0_sda_i       (sda0_i),
    .i2c0_sda_o       (sda0_o),
    .i2c0_sda_en_o    (sda0_oe),

    // I2C bus 1
    .i2c1_scl_i       (scl1_i),
    .i2c1_scl_o       (scl1_o),
    .i2c1_scl_en_o    (scl1_oe),
    .i2c1_sda_i       (sda1_i),
    .i2c1_sda_o       (sda1_o),
    .i2c1_sda_en_o    (sda1_oe),

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
    .rst_usb_n,
    .clk_peri,
    .rst_peri_n
  );

endmodule
