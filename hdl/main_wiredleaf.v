/*
 * main_wiredleaf.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: July 2013
 *
 * This top-level module contains only device-specific instantiations and
 * wiring. It connects central_core (the top-level functional module) with
 * specific device resources (DCMs, buffers, etc) and I/O Pads.
 *
 * This file is for use with the LeafLabs WiredLeaf board.
 *
 * Status LEDs, in order:
 *
 *    (barrel jack)
 *    0 blue    Main Clock (44.1MHz); should blink at 1Hz
 *    1 blue    "SATA Alive": solid if disk attached, blinks at 1Hz from
 *              sata_clk if not
 *    2 orange  Error Condition (any)
 *    3 orange  Lit if reset-after-power hasn't happened; then
 *              GPIO Output, register controlled (identical to ext[8])
 *    4 green   DAQ Acquire Enable
 *    5 green   "SATA Activity" (either read or write FIFO active)
 *    (buttons and SATA connectors)
 *
 * Buttons:
 *   (no buttons have any functional use)
 */

module main_wiredleaf (

    // Clocks and Reset button
    (* clock_signal = "yes"*) input wire clock_50mhz,
    input wire n_reset_button,

    // GPIO, LEDs, Buttons
    input wire [3:0] n_button,
    output wire [5:0] n_led,
    inout wire [15:0] ext,

    // USB FTDI Interface (eg, UART)
    inout [1:0] usb_d,

    // SATA
    input wire sata_clk_n, sata_clk_p,
    output wire sata1_tx_n, sata1_tx_p,
    input wire sata1_rx_n, sata1_rx_p,

    // Gigabit Ethernet
    output wire eth_n_phy_reset,
    output wire eth_gmii_gtx_clk,
    output wire eth_mdc,
    inout wire eth_mdio,
    output wire eth_gmii_tx_en,
    output wire [7:0] eth_gmii_txd,

    // MCU SPI Interface
    input wire mcu_sclk,
    input wire mcu_n_cs,
    input wire mcu_mosi,
    output wire mcu_miso,

    // Other MCU Lines
    //inout wire [1:0] mcu_ex, // DEBUG

    // Cross-Board Synchronization
    input wire sync_slave_n, sync_slave_p,
    output wire sync_master_n, sync_master_p,

    // HSMC high-density header
    inout wire[158:1] hsmc,

    // DDR3 HW Signals
    inout   [7:0]                    mcb5_dram_dq,
    output  [13:0]                   mcb5_dram_a,
    output  [2:0]                    mcb5_dram_ba,
    output                           mcb5_dram_ras_n,
    output                           mcb5_dram_cas_n,
    output                           mcb5_dram_we_n,
    output                           mcb5_dram_odt,
    output                           mcb5_dram_reset_n,
    output                           mcb5_dram_cke,
    output                           mcb5_dram_dm,
    inout                            mcb5_rzq,
    inout                            mcb5_zio,
    input                            memc_sys_clk,
    input                            memc_sys_rst_i,
    inout                            mcb5_dram_dqs,
    inout                            mcb5_dram_dqs_n,
    output                           mcb5_dram_ck,
    output                           mcb5_dram_ck_n
    );  // don't forget to remove trailing comma!

    /********** Parameters **********/
    // NOTE: these defaults overridden by Makefile
    parameter INTERFACE_BOARD_VERSION = 1;
    parameter GIT_COMMIT = 32'h0;
    parameter BUILD_UNIX_TIME = 32'h0;
    parameter NUM_CHIPS = 32;
    parameter DUMMY_DAQ = 0;
    parameter FORCE_ACQUISITION = 1;
    parameter MOSI_ON_FALLING = 1;
    parameter MOSI_ON_NEXT_RISING = 1;
    parameter SIGNED_ADC_VALUES = 1;

    /********** Device-specific Setup **********/
    // "global" reset, including clocks
    wire global_reset;
    assign global_reset = ~n_reset_button;
    // "functional" reset, of all sub-systems. see below for power-on-reset
    // implementation.

    // GPIO lines, LEDs, buttons, switches
    wire [5:0] gpio_led;
    assign n_led = ~gpio_led;
    wire [3:0] gpio_button;
    assign gpio_button = ~n_button;

    // USB FTDI (eg, UART)
    wire uart_rx;
    wire uart_tx;
    assign uart_rx = usb_d[0];
    assign usb_d[1] = uart_tx;

    // DAQ instatiation for actual HSMC header.
    wire spi_csbar;
    wire spi_sclk;
    wire spi_mosi;
    wire[31:0] spi_miso;
    // The CS (SPI Chip Select) lines and debug lines differ between interface
    // board revisions
    if (INTERFACE_BOARD_VERSION == 1) begin
        assign hsmc[  8] = spi_csbar;   // CS
        assign hsmc[ 96] = spi_csbar;   // CS
        assign hsmc[ 98] = spi_csbar;   // CS

        assign hsmc[  1] = spi_csbar;   // <debug>
        assign hsmc[  3] = spi_sclk;    // <debug>
        assign hsmc[  5] = spi_mosi;    // <debug>
        assign hsmc[  7] = spi_sclk;    // <debug>
    end else if (INTERFACE_BOARD_VERSION == 2) begin
        assign hsmc[  8] = spi_csbar;   // CS 1
        assign hsmc[ 21] = spi_csbar;   // CS 2
        assign hsmc[ 41] = spi_csbar;   // CS 3
        assign hsmc[ 67] = spi_csbar;   // CS 4
        assign hsmc[ 73] = spi_csbar;   // CS 5
        assign hsmc[ 98] = spi_csbar;   // CS 6
        assign hsmc[119] = spi_csbar;   // CS 7
        assign hsmc[121] = spi_csbar;   // CS 8
    end else begin
        // SHOULD NEVER GET HERE
        assign spi_csbar = spi_csbar;
    end

    // The rest of the outputs don't vary between board revisions
    assign hsmc[ 12] = spi_sclk;  // SCLK 1
    assign hsmc[ 24] = spi_sclk;  // SCLK 2
    assign hsmc[ 44] = spi_sclk;  // SCLK 3
    assign hsmc[ 62] = spi_sclk;  // SCLK 4
    assign hsmc[ 80] = spi_sclk;  // SCLK 5
    assign hsmc[104] = spi_sclk;  // SCLK 6
    assign hsmc[122] = spi_sclk;  // SCLK 7
    assign hsmc[140] = spi_sclk;  // SCLK 8

    assign hsmc[ 10] = spi_mosi;  // MOSI 1
    assign hsmc[ 22] = spi_mosi;  // MOSI 2
    assign hsmc[ 42] = spi_mosi;  // MOSI 3
    assign hsmc[ 60] = spi_mosi;  // MOSI 4
    assign hsmc[ 78] = spi_mosi;  // MOSI 5
    assign hsmc[102] = spi_mosi;  // MOSI 6
    assign hsmc[120] = spi_mosi;  // MOSI 7
    assign hsmc[138] = spi_mosi;  // MOSI 8

    // inputs:
    assign spi_miso[ 0] = hsmc[18];  // MISO A1
    assign spi_miso[ 1] = hsmc[20];  // MISO B1
    assign spi_miso[ 2] = hsmc[14];  // MISO C1
    assign spi_miso[ 3] = hsmc[16];  // MISO D1
    assign spi_miso[ 4] = hsmc[30];  // MISO A2
    assign spi_miso[ 5] = hsmc[32];
    assign spi_miso[ 6] = hsmc[26];
    assign spi_miso[ 7] = hsmc[28];
    assign spi_miso[ 8] = hsmc[54];  // MISO A3
    assign spi_miso[ 9] = hsmc[56];
    assign spi_miso[10] = hsmc[48];
    assign spi_miso[11] = hsmc[50];
    assign spi_miso[12] = hsmc[72];  // MISO A4
    assign spi_miso[13] = hsmc[74];
    assign spi_miso[14] = hsmc[66];
    assign spi_miso[15] = hsmc[68];
    assign spi_miso[16] = hsmc[90];  // MISO A5
    assign spi_miso[17] = hsmc[92];
    assign spi_miso[18] = hsmc[84];
    assign spi_miso[19] = hsmc[86];
    assign spi_miso[20] = hsmc[114]; // MISO A6
    assign spi_miso[21] = hsmc[116];
    assign spi_miso[22] = hsmc[108];
    assign spi_miso[23] = hsmc[110];
    assign spi_miso[24] = hsmc[132]; // MISO A7
    assign spi_miso[25] = hsmc[134];
    assign spi_miso[26] = hsmc[126];
    assign spi_miso[27] = hsmc[128];
    assign spi_miso[28] = hsmc[150]; // MISO A8
    assign spi_miso[29] = hsmc[152];
    assign spi_miso[30] = hsmc[144];
    assign spi_miso[31] = hsmc[146];

    // Buffer 50mhz clock input
    wire clock_50mhz_breakout;
    IBUFG clk50buf (
        .I(clock_50mhz),
        .O(clock_50mhz_breakout)
        );

    // Setup 44.1MHz and 125MHz clocks using DCM
    (* clock_signal = "yes"*) wire clock_44mhz;
    clk_wiz_50_44mhz dcm_44 (
        .CLK_IN_50(clock_50mhz_breakout),
        .CLK_OUT_44(clock_44mhz)
        );
    (* clock_signal = "yes"*) wire clock_125mhz;
    clk_wiz_50_125mhz dcm_125 (
        .CLK_IN_50(clock_50mhz_breakout),
        .CLK_OUT_125(clock_125mhz),
        .CLK_OUT_125_180()
        );

    // Send 125MHz clock to GIGE PHY
    ODDR2 oddr2_gige (
        .Q (eth_gmii_gtx_clk),
        .C0 (clock_125mhz),
        .C1 (~clock_125mhz),
        .D0 (1'b1),
        .D1 (1'b0),
        .R (),
        .CE(),
        .S()
        );

    // 150MHz differential clock for SATA
    (* clock_signal = "yes"*) wire sata_clk;
    IBUFDS ibufds_sata_clk (
        .I(sata_clk_p),
        .IB(sata_clk_n),
        .O(sata_clk)
        );

    // Cross-board synchronization differentials
    wire sync_slave; // input
    IBUFDS ibufds_sync_slave (
        .I(sync_slave_p),
        .IB(sync_slave_n),
        .O(sync_slave)
        );
    wire sync_master; // output
    assign sync_master_p = sync_master;
    assign sync_master_n = !sync_master;
/*
    OBUFDS obufds_sync_master (
        .O(sync_master_p),
        .OB(sync_master_n),
        .I(sync_master)
        );
*/
    STARTUP_SPARTAN6 STARTUP_SPARTAN6_i0 (
                                            .CLK(clock_44mhz),
                                            .GSR(global_reset)
                                            );

    // MCU SPI interface
    wire mcu_sclk_db, mcu_mosi_db, mcu_n_cs_db;
    // debounce all MCU SPI input lines
    debounce db_sclk (
                      .clock(clock_44mhz),
                      .i(mcu_sclk),
                      .o(mcu_sclk_db)
                      );

    debounce db_mosi (
                      .clock(clock_44mhz),
                      .i(mcu_mosi),
                      .o(mcu_mosi_db)
                      );

    debounce db_n_cs (
                      .clock(clock_44mhz),
                      .i(mcu_n_cs),
                      .o(mcu_n_cs_db)
                      );

    // MCU Status lines (FPGA out, MCU in)
    // whether there is any error condition at all
    wire mcu_error_line;
    //assign mcu_ex[0] = mcu_error_line; // DEBUG
    // whether the FPGA is alive at all
    //assign mcu_ex[1] = 1'b1;  // DEBUG

    wire [31:0] board_identifier;
    board_id_generator board_id_i0 (
        .clock(clock_44mhz),
        .board_identifier(board_identifier),
        .ready()
    );

   /******** Instatiate the DDR3 *********/
   wire         ddr3_reset; // module specific reset generated by central core.
   wire         ddr3_calib_done;
   wire         ddr3_wr_clk;
   wire         ddr3_wr_stb;
   wire [31:0]  ddr3_wr_data;
   wire         ddr3_wr_full;
   wire         ddr3_wr_empty;
   wire [6:0]   ddr3_wr_count;
   wire         ddr3_rd_clk;
   wire         ddr3_rd_stb;
   wire [31:0]  ddr3_rd_data;
   wire         ddr3_rd_empty;
   wire         ddr3_rd_full;
   wire [6:0]   ddr3_rd_count;

    ddr3_fifo_wrapper #(
        .IS_SIMULATION ("FALSE")
    ) ddr3_fifo_wrapper_i0 (
        .clk_50(clock_50mhz_breakout), // DDR core adds its own bufg, so we dont need the bufg version
        .reset(ddr3_reset), // generated by central core, not main reset.
        .calib_done(ddr3_calib_done),

        .wr_clk(ddr3_wr_clk),
        .wr_stb(ddr3_wr_stb),
        .wr_data(ddr3_wr_data), // 32
        .wr_full(ddr3_wr_full),
        .wr_empty(ddr3_wr_empty),
        .wr_count(ddr3_wr_count),

        .rd_clk(ddr3_rd_clk),
        .rd_stb(ddr3_rd_stb),
        .rd_data(ddr3_rd_data),  // 32
        .rd_empty(ddr3_rd_empty),
        .rd_full(ddr3_rd_full),
        .rd_count(ddr3_rd_count), // 7

        .mcb5_dram_dq(mcb5_dram_dq),              // in-out 7:0
        .mcb5_dram_a(mcb5_dram_a),               // output 13:0
        .mcb5_dram_ba(mcb5_dram_ba),              // output 2:0
        .mcb5_dram_ras_n(mcb5_dram_ras_n),           // output
        .mcb5_dram_cas_n(mcb5_dram_cas_n),           // output
        .mcb5_dram_we_n(mcb5_dram_we_n),            // output
        .mcb5_dram_odt(mcb5_dram_odt),             // output
        .mcb5_dram_reset_n(mcb5_dram_reset_n),         // output
        .mcb5_dram_cke(mcb5_dram_cke),     // output
        .mcb5_dram_dm(mcb5_dram_dm),              // output
        .mcb5_rzq(mcb5_rzq),                  // inout
        .mcb5_zio(mcb5_zio),                  // inout
        .mcb5_dram_dqs(mcb5_dram_dqs),             // inout
        .mcb5_dram_dqs_n(mcb5_dram_dqs_n),           // inout
        .mcb5_dram_ck(mcb5_dram_ck),              // output
        .mcb5_dram_ck_n(mcb5_dram_ck_n)             // output
    );

    /********** Instantiate Central Core **********/
    wire sata_alive;
    wire sata_activity;
    wire daq_acquiring;
    wire sata_io_ready; // DEBUG
    central_core #(
        .GIT_COMMIT(GIT_COMMIT),
        .BUILD_UNIX_TIME(BUILD_UNIX_TIME),
        .GPIO_READ_MASK(16'h00FF),
        .GPIO_WRITE_MASK(16'hFF00),
        .NUM_CHIPS(NUM_CHIPS),
        .DUMMY_DAQ(DUMMY_DAQ),
        .FORCE_ACQUISITION(FORCE_ACQUISITION),
        .MOSI_ON_FALLING(MOSI_ON_FALLING),
        .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING),
        .SIGNED_ADC_VALUES(SIGNED_ADC_VALUES)
    ) central_core_i0 (
        .clock_44mhz(clock_44mhz),
        .clock_125mhz(clock_125mhz),

        .gpio_pins(ext),

        .daq_spi_sclk(spi_sclk),
        .daq_spi_n_cs(spi_csbar),
        .daq_spi_mosi(spi_mosi),
        .daq_spi_miso(spi_miso[(NUM_CHIPS-1):0]),

        .daq_ss_master(sync_master),
        .daq_ss_slave(sync_slave),

        .board_identifier(board_identifier),

        .gige_reset(eth_n_phy_reset),
        .gige_txc_gtxclk(eth_gmii_gtx_clk),
        .gige_mdc(eth_mdc),
        .gige_mdio(eth_mdio),
        .gige_txctl_txen(eth_gmii_tx_en),
        .gige_tx_data(eth_gmii_txd),

        .mcu_spi_sclk(mcu_sclk_db),
        .mcu_spi_n_cs(mcu_n_cs_db),
        .mcu_spi_mosi(mcu_mosi_db),
        .mcu_spi_miso(mcu_miso),

        .uart_rx(uart_rx),
        .uart_tx(uart_tx),

        .sata_txdt_p0(sata1_tx_p),
        .sata_txdt_n0(sata1_tx_n),
        .sata_rxdt_p0(sata1_rx_p),
        .sata_rxdt_n0(sata1_rx_n),
        .sata_clk(sata_clk),
        .sata_io_ready(sata_io_ready), // DEBUG

        .sata_alive(sata_alive),
        .sata_activity(sata_activity),
        .daq_acquiring(daq_acquiring),
        .error_led(mcu_error_line),

        // DDR3
        .ddr3_reset(ddr3_reset),
        .ddr3_calib_done(ddr3_calib_done),

        .ddr3_wr_clk(ddr3_wr_clk),
        .ddr3_wr_stb(ddr3_wr_stb),
        .ddr3_wr_data(ddr3_wr_data),
        .ddr3_wr_full(ddr3_wr_full),
        .ddr3_wr_empty(ddr3_wr_empty),
        .ddr3_wr_count(ddr3_wr_count),

        .ddr3_rd_clk(ddr3_rd_clk),
        .ddr3_rd_stb(ddr3_rd_stb),
        .ddr3_rd_data(ddr3_rd_data),
        .ddr3_rd_empty(ddr3_rd_empty),
        .ddr3_rd_full(ddr3_rd_full),
        .ddr3_rd_count(ddr3_rd_count)
        );

    // these are here to squelch compile warnings
    //     gpio_led[0] = throb or buttons, below
    assign gpio_led[1] = sata_alive;
    assign gpio_led[2] = mcu_error_line;
    assign gpio_led[3] = ext[8];    // writable by register
    assign gpio_led[4] = daq_acquiring;
    assign gpio_led[5] = sata_activity;

    assign ext[0] = sata_io_ready; // DEBUG

    /************ Misc Logic ************/
    reg throb = 1'b1;
    assign gpio_led[0] = throb || gpio_button[3] || gpio_button[2] || gpio_button[1] || gpio_button[0];
    reg [27:0] throb_counter = 28'b0;
    always @(posedge clock_44mhz) begin
       throb <= (throb_counter > 28'd22_000_000);
       if (throb_counter >= 28'd44_100_000) begin
          throb_counter <= 28'd0;
       end
       else begin
          throb_counter <= throb_counter + 28'd1;
       end
    end

endmodule

