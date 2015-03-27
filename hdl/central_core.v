/*
 * central_core.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * This is the (hardware abstracted) top-level module for the SNG WiredLeaf
 * FPGA firmware. It mostly serves to interconnect all the function-specific
 * cores, though it does control a few system-wide configuration registers and
 * enforces some interfacing logic.
 *
 * TODO: more details as they come.
 *
 * Modules:
 *      0x00    Error
 *      0x01    Central
 *      0x02    SATA
 *      0x03    DAQ
 *      0x04    UDP
 *      0x05    GPIO
 *
 * Changes to the tables below should be included in the
 * "wiredleaf-registers.txt" file in the documentation repository.
 *
 * Register Map (module 0x01; Central Core):
 *
 *    0x00    R/0     Unused (3 B) | Central Core error flags (1 B)
 *    0x01    R/0     Unused (3 B) | Top Level Module State (UNIMPL) (1 B)
 *    0x02    R/W     Experiment cookie, bottom bytes (4 B)
 *    0x03    R/W     Experiment cookie, top bytes (4 B)
 *    0x04    R/O     Git commit of FPGA HDL firmware (4 B)
 *    0x05    R/O     FPGA HDL Parameters (4 B)
 *    0x06    R/O     FPGA HDL Build Unix Time (seconds) (4 B)
 *    0x07    R/O     Board Identifier (4 B)
 *
 * Error Flags (Central Core):
 *      0   Configuration Error
 *
 */

module central_core (
    // Clocks
    input wire reset,
    input wire clock_44mhz,
    input wire clock_125mhz,

    // DAQ SPI Interface
    output wire daq_spi_sclk,
    output wire daq_spi_n_cs,
    output wire daq_spi_mosi,
    input wire [(NUM_CHIPS-1):0] daq_spi_miso,

    // DAQ Cross-Board Synchronization
    output wire daq_ss_master,
    input wire daq_ss_slave,

    // Board Identifier
    input wire [31:0] board_identifier,

    // GPIO
    inout wire[15:0] gpio_pins,

    // Status Indicators
    output wire sata_alive,
    output wire sata_activity,
    output wire daq_acquiring,
    output wire error_led,
    output wire sata_io_ready,

    // GIGE Hardware I/O
    output wire gige_reset,
    output wire gige_txc_gtxclk,
    output wire gige_mdc,
    inout wire gige_mdio,
    output wire gige_txctl_txen,
    output wire[7:0] gige_tx_data,

    // MCU SPI (passthrough to cfg_master)
    input wire mcu_spi_sclk,
    input wire mcu_spi_n_cs,
    input wire mcu_spi_mosi,
    output wire mcu_spi_miso,

    // UART
    input wire uart_rx,
    output wire uart_tx,

    // SATA
    output wire sata_txdt_p0,
    output wire sata_txdt_n0,
    input wire sata_rxdt_p0,
    input wire sata_rxdt_n0,
    input wire sata_clk,

    // DDR3
    output             ddr3_reset,
    input              ddr3_calib_done,

    output             ddr3_wr_clk,
    output             ddr3_wr_stb,
    output [31:0]      ddr3_wr_data,
    input              ddr3_wr_full,
    input              ddr3_wr_empty,
    input [6:0]        ddr3_wr_count,

    output             ddr3_rd_clk,
    output             ddr3_rd_stb,
    input [31:0]       ddr3_rd_data,
    input              ddr3_rd_empty,
    input              ddr3_rd_full,
    input [6:0]        ddr3_rd_count
    );  // don't forget to remove trailing comma!

    // configurable parameters
    parameter GIT_COMMIT = 32'h0;
    parameter BUILD_UNIX_TIME = 32'h0;
    parameter GPIO_READ_MASK = 16'h00FF;
    parameter GPIO_WRITE_MASK = 16'hFF00;
    parameter NUM_CHIPS = 32;
    parameter DUMMY_DAQ = 1;
    parameter FORCE_ACQUISITION = 1;
    parameter MOSI_ON_FALLING = 1;
    parameter MOSI_ON_NEXT_RISING = 1;
    parameter SIGNED_ADC_VALUES = 1;

    parameter DEFAULT_EXPERIMENT_COOKIE = 64'h1234_5678_8765_4321;
    reg [63:0] experiment_cookie = DEFAULT_EXPERIMENT_COOKIE;

    wire [31:0] sata_start_write_index;

    wire any_error;

    // Tie off unused UART lines (for now)
    assign uart_tx = uart_rx;

    assign error_led = any_error;

    /********** Interface FIFOs **********/
    wire daq2udp_fifo_read_clock;
    wire daq2udp_fifo_read_en;
    wire [7:0] daq2udp_fifo_data;
    wire [12:0] daq2udp_fifo_read_count;
    wire [15:0] daq2udp_payload_len;

    wire [31:0] daq2sata_fifo_data;
    wire daq2sata_fifo_clock;
    wire daq2sata_fifo_read_en;
    wire daq2sata_fifo_empty;
    wire daq2sata_fifo_almost_empty;
    wire [14:0] daq2sata_fifo_data_count;

    wire sata2udp_fifo_read_clock;
    wire sata2udp_fifo_read_en;
    wire [7:0] sata2udp_fifo_data;
    wire [12:0] sata2udp_fifo_read_count;
    wire [15:0] sata2udp_payload_len;

    /********** Configuration register interfaces **********/
    /********** (also Error registers (module 0x00) **********/
    wire [5:0] error_flags;
    wire [5:0] cfg_sack;
    wire [5:0] cfg_mread_en;
    wire [5:0] cfg_mwrite_en;
    wire [31:0] cfg_data_mwrite;
    wire [31:0] cfg_data_mread0;
    wire [31:0] cfg_data_mread1;
    wire [31:0] cfg_data_mread2;
    wire [31:0] cfg_data_mread3;
    wire [31:0] cfg_data_mread4;
    wire [31:0] cfg_data_mread5;
    wire [7:0] cfg_addr;
    cfg_master cfg_master_i0 (
        .clock(clock_44mhz),
        .reset(reset),
        // Error registers
        // SPI interface (passthrough)
        .spi_sclk(mcu_spi_sclk),
        .spi_n_cs(mcu_spi_n_cs),
        .spi_mosi(mcu_spi_mosi),
        .spi_miso(mcu_spi_miso),
        // Configuration Register Interfaces
        .any_error(any_error),
        .cfg_sack(cfg_sack),
        .cfg_mread_en(cfg_mread_en),
        .cfg_mwrite_en(cfg_mwrite_en),
        .cfg_data_mwrite(cfg_data_mwrite),
        .cfg_data_mread0(cfg_data_mread0),
        .cfg_data_mread1(cfg_data_mread1),
        .cfg_data_mread2(cfg_data_mread2),
        .cfg_data_mread3(cfg_data_mread3),
        .cfg_data_mread4(cfg_data_mread4),
        .cfg_data_mread5(cfg_data_mread5),
        .cfg_addr(cfg_addr)
        );


    /********** SATA Core (module 0x02) **********/
    sata_core sata_core_i0 (
        .clock(clock_44mhz),
        .reset(reset),

        .alive_led(sata_alive),
        .activity_led(sata_activity),
        .sata_io_ready(sata_io_ready),

        .start_write_index(sata_start_write_index),

        // UDP FIFO
        .udp_fifo_data(sata2udp_fifo_data),
        .udp_fifo_clock(sata2udp_fifo_read_clock),
        .udp_fifo_read_en(sata2udp_fifo_read_en),
        .udp_fifo_data_count(sata2udp_fifo_read_count),
        .udp_fifo_payload_len(sata2udp_payload_len),

        // DAQ FIFO
        .daq_fifo_data(daq2sata_fifo_data),
        .daq_fifo_clock(daq2sata_fifo_clock),
        .daq_fifo_read_en(daq2sata_fifo_read_en),
        .daq_fifo_empty(daq2sata_fifo_empty),
        .daq_fifo_data_count(daq2sata_fifo_data_count),

        // generic register cfg interface
        .cfg_data_mwrite(cfg_data_mwrite),
        .cfg_data_mread(cfg_data_mread2),
        .cfg_addr(cfg_addr),
        .cfg_mread_en(cfg_mread_en[2]),
        .cfg_mwrite_en(cfg_mwrite_en[2]),
        .cfg_sack(cfg_sack[2]),
        .error_flag(error_flags[2]),
        .any_error_flag(any_error),

        .sata_txdt_p0(sata_txdt_p0),
        .sata_txdt_n0(sata_txdt_n0),
        .sata_rxdt_p0(sata_rxdt_p0),
        .sata_rxdt_n0(sata_rxdt_n0),
        .sata_clk(sata_clk)
        );


    /********** DAQ Core (module 0x03) **********/
    daq_core #(
        .NUM_CHIPS(NUM_CHIPS),
        .DUMMY_DAQ(DUMMY_DAQ),
        .FORCE_ACQUISITION(FORCE_ACQUISITION),
        .MOSI_ON_FALLING(MOSI_ON_FALLING),
        .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING),
        .SIGNED_ADC_VALUES(SIGNED_ADC_VALUES)
    ) daq_core_i0 (
        .clock(clock_44mhz),
        .reset(reset),
        // misc i/o
        .experiment_cookie(experiment_cookie),
        .gpio_pins(gpio_pins),
        .any_error_flag(any_error),
        .board_identifier(board_identifier),
        .acquire_status(daq_acquiring),
        .sata_start_index(sata_start_write_index),
        // Cross-board synchronization (TODO)
        .ss_master(daq_ss_master),
        .ss_slave(daq_ss_slave),
        // DAQ SPI interface
        .spi_csbar(daq_spi_n_cs),
        .spi_sclk(daq_spi_sclk),
        .spi_mosi(daq_spi_mosi),
        .spi_miso(daq_spi_miso),
        // UDP interface
        .udp_fifo_data(daq2udp_fifo_data),
        .udp_fifo_clock(daq2udp_fifo_read_clock),
        .udp_fifo_read_en(daq2udp_fifo_read_en),
        .udp_fifo_data_count(daq2udp_fifo_read_count),
        .udp_fifo_payload_len(daq2udp_payload_len),
        .udp_fifo_empty(),
        // SATA interface
        .sata_fifo_data(daq2sata_fifo_data),
        .sata_fifo_clock(daq2sata_fifo_clock),
        .sata_fifo_read_en(daq2sata_fifo_read_en),
        .sata_fifo_empty(daq2sata_fifo_empty),
        .sata_fifo_almost_empty(daq2sata_fifo_almost_empty),
        .sata_fifo_data_count(daq2sata_fifo_data_count),
        // generic register cfg interface
        .cfg_data_mwrite(cfg_data_mwrite),
        .cfg_data_mread(cfg_data_mread3),
        .cfg_addr(cfg_addr),
        .cfg_mread_en(cfg_mread_en[3]),
        .cfg_mwrite_en(cfg_mwrite_en[3]),
        .cfg_sack(cfg_sack[3]),
        .error_flag(error_flags[3]),
         // DDR3 FIFO interface
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

    /********** UDP Core (module 0x04) **********/
    udp_core udp_core_i0 (
        .clock_44mhz(clock_44mhz),
        .clock_125mhz(clock_125mhz),
        .reset(reset),
        // GIGE PHY interface
        .gige_reset(gige_reset),
        .gige_txc_gtxclk(gige_txc_gtxclk),
        .gige_mdc(gige_mdc),
        .gige_mdio(gige_mdio),
        .gige_txctl_txen(gige_txctl_txen),
        .gige_tx_data(gige_tx_data),
        // DAQ interface
        .daq_fifo_payload_len(daq2udp_payload_len),
        .daq_fifo_count(daq2udp_fifo_read_count),
        .daq_fifo_data(daq2udp_fifo_data),
        .daq_fifo_clock(daq2udp_fifo_read_clock),
        .daq_fifo_read_en(daq2udp_fifo_read_en),
        // SATA interface
        .sata_fifo_payload_len(sata2udp_payload_len),
        .sata_fifo_count(sata2udp_fifo_read_count),
        .sata_fifo_data(sata2udp_fifo_data),
        .sata_fifo_clock(sata2udp_fifo_read_clock),
        .sata_fifo_read_en(sata2udp_fifo_read_en),
        // generic register cfg interface
        .cfg_data_mwrite(cfg_data_mwrite),
        .cfg_data_mread(cfg_data_mread4),
        .cfg_addr(cfg_addr),
        .cfg_mread_en(cfg_mread_en[4]),
        .cfg_mwrite_en(cfg_mwrite_en[4]),
        .cfg_sack(cfg_sack[4]),
        .error_flag(error_flags[4])
        );


    /********** GPIO Core (module 0x05) **********/
    gpio_core #(
        .GPIO_READ_MASK(GPIO_READ_MASK),
        .GPIO_WRITE_MASK(GPIO_WRITE_MASK)
    ) gpio_core_i0 (
        .clock(clock_44mhz),
        .reset(reset),
        .gpio_pins(gpio_pins),
        // generic register cfg interface
        .cfg_data_mwrite(cfg_data_mwrite),
        .cfg_data_mread(cfg_data_mread5),
        .cfg_addr(cfg_addr),
        .cfg_mread_en(cfg_mread_en[5]),
        .cfg_mwrite_en(cfg_mwrite_en[5]),
        .cfg_sack(cfg_sack[5]),
        .error_flag(error_flags[5])
        );


    /********** Error and Central Cores (modules 0x00, 0x01)  **********/
    reg ecore_cfg_sack = 0;
    reg [5:0] error_flags_buf = 6'd0;
    assign error_flags[0] = 1'b0;
    // ecore_cfg_sack is used as a state bit
    assign cfg_sack[0] = ecore_cfg_sack;
    assign cfg_data_mread0 = {26'd0, error_flags_buf};

    reg [31:0] central_cfg_mread = 32'd0;
    wire central_cfg_mread_en;
    assign central_cfg_mread_en = cfg_mread_en[1];
    wire central_cfg_mwrite_en;
    assign central_cfg_mwrite_en = cfg_mwrite_en[1];
    assign cfg_data_mread1 = central_cfg_mread;
    reg central_cfg_sack = 0;
    assign cfg_sack[1] = central_cfg_sack;
    wire central_error_flag;
    reg central_error_cfg = 0;
    assign central_error_flag = central_error_cfg;
    assign error_flags[1] = central_error_flag;

    assign any_error = error_flags_buf[0] || error_flags_buf[1]
        || error_flags_buf[2] || error_flags_buf[3] || error_flags_buf[4]
        || error_flags_buf[5];

    always @(posedge clock_44mhz) begin
        if (reset) begin
            ecore_cfg_sack <= 1'b0;
            error_flags_buf <= 6'd0;

            experiment_cookie <= DEFAULT_EXPERIMENT_COOKIE;
            central_cfg_mread <= 32'd0;
            central_cfg_sack <= 1'b0;
            central_error_cfg <= 1'b0;
        end else begin
            //// Error Core
            // update error_flags_buf in all conditional paths
            if (ecore_cfg_sack) begin
                ecore_cfg_sack <= 1'b0;
                error_flags_buf <= error_flags_buf | error_flags;
            end else if (cfg_mread_en[0]) begin
                // cfg_data_mread0 is constantly updated, so just need to set
                // sack
                ecore_cfg_sack <= 1'b1;
                error_flags_buf <= error_flags_buf | error_flags;
            end else if (cfg_mwrite_en[0]) begin
                ecore_cfg_sack <= 1'b1;
                // overwrite error_flags_buf on write
                error_flags_buf <= (error_flags_buf & cfg_data_mwrite[5:0]) | error_flags;
            end else begin
                // all other cases, just update error flag buffers
                error_flags_buf <= error_flags_buf | error_flags;
            end

            //// Central Core
            if (central_cfg_mread_en || central_cfg_mwrite_en) begin
                central_cfg_sack <= 1'b1;
                case (cfg_addr)
                8'h00: begin  // Unused (3 B) | Central Core Error Flags (1 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= cfg_data_mwrite[0];
                        central_cfg_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        central_cfg_mread <= {31'd0, central_error_cfg};
                    end
                end
                8'h01: begin  // Unused (3 B) | Top Level Module State (1 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= 1'b1;
                    end
                    central_cfg_mread <= 32'd0;
                end
                8'h02: begin  // Experiment cookie, top bytes (4 B)
                    if (central_cfg_mwrite_en) begin
                        experiment_cookie[63:32] <= cfg_data_mwrite;
                        central_cfg_mread <= cfg_data_mwrite;
                    end else begin
                        central_cfg_mread <= experiment_cookie[63:32];
                    end
                end
                8'h03: begin  // Experiment cookie, bottom bytes (4 B)
                    if (central_cfg_mwrite_en) begin
                        experiment_cookie[31:0] <= cfg_data_mwrite;
                        central_cfg_mread <= cfg_data_mwrite;
                    end else begin
                        central_cfg_mread <= experiment_cookie[31:0];
                    end
                end
                8'h04: begin  // Git commit of FPGA HDL firmware (4 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= 1'b1;
                    end
                    central_cfg_mread <= GIT_COMMIT;
                end
                8'h05: begin  // FPGA HDL Parameters (4 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= 1'b1;
                    end
                    central_cfg_mread <= {27'd0,
                        MOSI_ON_FALLING ? 1'b1 : 1'b0,        // 4
                        MOSI_ON_NEXT_RISING ? 1'b1 : 1'b0,    // 3
                        SIGNED_ADC_VALUES ? 1'b1 : 1'b0,      // 2
                        FORCE_ACQUISITION ? 1'b1 : 1'b0,      // 1
                        DUMMY_DAQ ? 1'b1 : 1'b0               // 0
                        };
                end
                8'h06: begin  // FPGA HDL Build Unix Time (seconds) (4 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= 1'b1;
                    end
                    central_cfg_mread <= BUILD_UNIX_TIME;
                end
                8'h07: begin  // Board Identifier (4 B)
                    if (central_cfg_mwrite_en) begin
                        central_error_cfg <= 1'b1;
                    end
                    central_cfg_mread <= board_identifier;
                end
                default: begin
                    // configuration address error
                    central_error_cfg <= 1'b1;
                    central_cfg_mread <= 32'd0;
                end
                endcase
            end else begin
                central_cfg_sack <= 1'b0;
            end
        end
    end

endmodule

