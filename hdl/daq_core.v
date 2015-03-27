/*
 * daq_core.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * Changes to the tables below should be included in the
 * "wiredleaf-registers.txt" file in the documentation repository.
 *
 * Register map (module 0x03):
 *
 *      0x00    R/W     DAQ Module Error Flags
 *      0x01    R/W     Unused (3 B) | DAQ Module Acquire Enable (1 B)
 *      0x02    R/W     Desired Start Board Sample Number (4 B)
 *      0x03    R/O     Current Board Sample Number (4 B)
 *      0x04    R/O     Chip alive bitmask (4 B)
 *      0x05    R/W     CMD Write Enable (1 B) | CMD Chip Address (1 B) | CMD Command (2 B)
 *      0x06    R/W     Synchronous Sampling (???)
 *      0x07    R/O     DAQ-UDP FIFO read count ("bytes in FIFO")
 *      0x08    R/W     DAQ-UDP FIFO flags (TBD, but bit[0] is reset line)
 *      0x09    R/W     Unused (3 B) | DAQ-UDP Output Enable (1 B)
 *      0x0A    R/W     Unused (3 B) | DAQ-UDP Output Mode (1 B)
 *      0x0B    R/W     Unused (3 B) | DAQ-SATA Output Enable (1 B)
 *      0x0C    R/O     DAQ-SATA FIFO read count ("words in FIFO")
 *      0x0D    R/W     DAQ-SATA FIFO flags (TBD, but bit[0] is reset line)
 *      0x0E    R/W     Unused (3 B) | DAQ Intan Sample Extra Channels (1 B)
 *      0x80    R/W     Unused (2 B) | Sub-Sample #0 Chip (1 B) | Sub-Sample #0 Chan (1 B)
 *                      [...]
 *      0x9F    R/W     Unused (2 B) | Sub-Sample #31 Chip (1 B) | Sub-Sample #31 Chan (1 B)
 *
 * Error Flags:
 *      0   Module Configuration Error
 *      1   DAQ-UDP FIFO Underflow
 *      2   DAQ-UDP FIFO Overflow
 *      3   DAQ-SATA FIFO Underflow
 *      4   DAQ-SATA FIFO Overflow
 *
 */

module daq_core (
    input clock,    // 44.1MHz
    input reset,

    input [63:0] experiment_cookie,
    input [15:0] gpio_pins,
    input wire any_error_flag,  // used for UDP packet construction
    input [31:0] board_identifier,
    output wire acquire_status,
    input [31:0] sata_start_index,

    //// spi interface
    output wire spi_csbar,
    output wire spi_sclk,
    output wire spi_mosi,
    input wire[(NUM_CHIPS-1):0] spi_miso,

    //// synchronous sampling lines
    output ss_master,
    input ss_slave,

    //// UDP FIFO interface
    output wire [15:0] udp_fifo_payload_len,
    output wire [7:0] udp_fifo_data,
    input wire udp_fifo_clock,
    input wire udp_fifo_read_en,
    output wire [12:0] udp_fifo_data_count,
    output wire udp_fifo_empty,

    //// SATA FIFO interface
    output wire [31:0] sata_fifo_data,
    input wire sata_fifo_clock,
    input wire sata_fifo_read_en,
    output wire sata_fifo_empty,
    output wire sata_fifo_almost_empty,
    output wire [14:0] sata_fifo_data_count,

    //// generic register configuration interface
    input [31:0] cfg_data_mwrite,
    output reg [31:0] cfg_data_mread = 32'b0,
    input [7:0] cfg_addr,
    input cfg_mread_en,
    input cfg_mwrite_en,
    output reg cfg_sack = 0,
    output wire error_flag,

    ///// DDR3 FIFO Interface
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
    );

    // configurable parameters
    parameter NUM_CHIPS = 32;
    parameter DUMMY_DAQ = 0;
    parameter FORCE_ACQUISITION = 1;
    parameter MOSI_ON_FALLING = 1;
    parameter MOSI_ON_NEXT_RISING = 1;
    parameter SIGNED_ADC_VALUES = 1;
    parameter DEFAULT_SAMPLE_EXTRA = 0;

    reg acquire_en = 0;
    assign acquire_status = acquire_en;
    reg sata_en = 0;
    reg sata_align_start_en = 0;
    reg udp_en = 0;
    reg udp_full_mode = 0;

    reg [31:0] desired_bsn = 0;
    wire [31:0] chips_alive_mask;

    reg cmd_write_en = 0;
    reg [7:0] cmd_chip_addr = 0;
    reg [15:0] cmd_command = 0;

    reg sample_extra_en = DEFAULT_SAMPLE_EXTRA;


    reg error_cfg = 0;
    reg [3:0] error_buf = 0;
    assign error_flag = error_buf[0] || error_buf[1] ||
                        error_buf[2] || error_buf[3] ||
                        error_cfg;

    wire [159:0] subsample_chip_indexes;     // 5bits x 32chips
    wire [159:0] subsample_channel_indexes;  // 5bits x 32channels
    reg [4:0] sschipi [0:31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // chip indexes
    reg [4:0] sschani [0:31] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};   // channel index
    genvar cntr;
    for (cntr = 0; cntr < 32; cntr = cntr + 1) begin : subsample_index_loop
        assign subsample_chip_indexes[(cntr*5)+4:(cntr*5)] = sschipi[cntr];
        assign subsample_channel_indexes[(cntr*5+4):(cntr*5)] = sschani[cntr];
    end

    wire [31:0] sngdaq_data;
    wire [31:0] sngdaq_bsi;
    wire sngdaq_ready;
    reg sngdaq_reset_bsi = 0;
    wire [9:0] sngdaq_addr;
    wire sngdaq_read_en;
    wire [7:0] dac_state;
    if (DUMMY_DAQ) begin
        sngdaq_dummy sngdaq_i0 (
            .clk_i(clock),           // 44.1 MHz clock
            .reset_i(reset),
            .enable_acq(acquire_en),

            .spi_csbar_o(spi_csbar),
            .spi_sclk_o(spi_sclk),
            .spi_mosi_o(spi_mosi),
            .spi_miso_i(spi_miso),

            .data_clk_i(clock), // BRAM read clock
            .data_en_i(sngdaq_read_en), // BRAM read enable
            .data_addr_i(sngdaq_addr), // BRAM read address
            .data_o(sngdaq_data), // BRAM data
            .data_ready_o(sngdaq_ready), // "ok to read from BRAM"

            .extra_cmd_i(cmd_command),
            .extra_cmd_chip_i(cmd_chip_addr[5:0]),
            .extra_cmd_en_i(cmd_write_en),
            .sample_extra_en_i(sample_extra_en),
            .gpio_pins(gpio_pins),

            .dac_state_o(dac_state),
            .spi_ready_o(chips_alive_mask), // status of all SPI drivers
            .board_sample_index_o(sngdaq_bsi), // board sample number of BRAM contents
            .initial_board_sample_index_i(desired_bsn),
            .reset_board_sample_index_i(sngdaq_reset_bsi)
            );
    end else begin
        sngdaq #(
            .NUM_CHIPS(NUM_CHIPS),
            .FORCE_ACQUISITION(FORCE_ACQUISITION),
            .MOSI_ON_FALLING(MOSI_ON_FALLING),
            .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING),
            .SIGNED_ADC_VALUES(SIGNED_ADC_VALUES)
        ) sngdaq_i0 (
            .clk_i(clock),           // 44.1 MHz clock
            .reset_i(reset),
            .enable_acq(acquire_en),

            .spi_csbar_o(spi_csbar),
            .spi_sclk_o(spi_sclk),
            .spi_mosi_o(spi_mosi),
            .spi_miso_i(spi_miso),

            .data_clk_i(clock), // BRAM read clock
            .data_en_i(sngdaq_read_en), // BRAM read enable
            .data_addr_i(sngdaq_addr), // BRAM read address
            .data_o(sngdaq_data), // BRAM data
            .data_ready_o(sngdaq_ready), // "ok to read from BRAM"

            .extra_cmd_i(cmd_command),
            .extra_cmd_chip_i(cmd_chip_addr[5:0]),
            .extra_cmd_en_i(cmd_write_en),
            .sample_extra_en_i(sample_extra_en),
            .gpio_pins(gpio_pins),

            .dac_state_o(dac_state),
            .spi_ready_o(chips_alive_mask), // status of all SPI drivers
            .board_sample_index_o(sngdaq_bsi), // board sample number of BRAM contents
            .initial_board_sample_index_i(desired_bsn),
            .reset_board_sample_index_i(sngdaq_reset_bsi)
            );
    end

    wire udp_fifo_write_en;
    wire udp_fifo_full;
    wire udp_fifo_valid;

    wire sata_fifo_write_en;
    wire sata_fifo_full;

    reg udp_fifo_reset = 0;
    reg sata_fifo_reset = 0;
    wire sata_fifo_almost_full;
    wire error_udp_overflow;
    wire error_udp_underflow;
    wire error_sata_overflow;
    wire error_sata_underflow;
    daq_interfaces daq_interfaces_i0 (
        .clock(clock),
        .reset(reset),

        .udp_enable(udp_en),
        .udp_full_mode(udp_full_mode),
        .sata_enable(sata_en),
        .sata_align_start_en(sata_align_start_en),
        .sata_start_index(sata_start_index),
        .experiment_cookie(experiment_cookie),
        .gpio_pins(gpio_pins),
        .any_error_flag(any_error_flag),
        .board_identifier(board_identifier),

        .subsample_chip_indexes(subsample_chip_indexes),
        .subsample_channel_indexes(subsample_channel_indexes),

        .daq_bram_clock(), // just "clock"
        .daq_bram_addr(sngdaq_addr),
        .daq_bram_data(sngdaq_data),
        .daq_bram_read_en(sngdaq_read_en),
        .daq_data_ready(sngdaq_ready),
        .daq_board_sample_index(sngdaq_bsi),
        .daq_chip_alive_mask(chips_alive_mask),
        .dac_state(dac_state),

        .udp_fifo_payload_len(udp_fifo_payload_len),
        .udp_fifo_data(udp_fifo_data),
        .udp_fifo_clock(udp_fifo_clock),
        .udp_fifo_read_en(udp_fifo_read_en),
        .udp_fifo_data_count(udp_fifo_data_count),
        .udp_fifo_empty(udp_fifo_empty),
        .udp_fifo_full(udp_fifo_full),
        .udp_fifo_valid(udp_fifo_valid),
        .udp_fifo_write_en(udp_fifo_write_en),
        .udp_fifo_reset(udp_fifo_reset),

        .sata_fifo_data(sata_fifo_data),
        .sata_fifo_clock(sata_fifo_clock),
        .sata_fifo_read_en(sata_fifo_read_en),
        .sata_fifo_write_en(sata_fifo_write_en),
        .sata_fifo_data_count(sata_fifo_data_count),
        .sata_fifo_empty(sata_fifo_empty),
        .sata_fifo_almost_empty(sata_fifo_almost_empty),
        .sata_fifo_full(sata_fifo_full),
        .sata_fifo_reset(sata_fifo_reset),
        .sata_fifo_almost_full(sata_fifo_almost_full),

        .error_sata_overflow(error_sata_overflow),
        .error_sata_underflow(error_sata_underflow),
        .error_udp_overflow(error_udp_overflow),
        .error_udp_underflow(error_udp_underflow),

        ///// DDR3 FIFO Interface
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

    always @(posedge clock) begin
        if (reset) begin
            acquire_en <= 1'b0;
            sata_en <= 1'b0;
            sata_align_start_en <= 1'b0;
            udp_en <= 1'b0;
            udp_full_mode <= 1'b0;
            cmd_write_en <= 1'b0;
            cmd_chip_addr <= 8'd0;
            cmd_command <= 16'd0;
            error_cfg <= 1'b0;
            udp_fifo_reset <= 1'b0;
            sngdaq_reset_bsi <= 1'b0;
            reset_subsample_indexes;
            error_buf <= 4'd0;
            sample_extra_en <= DEFAULT_SAMPLE_EXTRA;
        end else begin
            // --------- Main Logic ---------
            // [...]
            // --------- CFG Interface ---------
            if (cfg_mread_en || cfg_mwrite_en) begin
                cfg_sack <= 1'b1;
                case (cfg_addr)
                8'h00: begin    // DAQ Module Error Flags
                    if (cfg_mwrite_en) begin
                        // corner case: write to error_cfg must return exact
                        // write value, regardless of other error flags
                        error_buf[3:0] <= cfg_data_mwrite[4:1];
                        error_cfg <= cfg_data_mwrite[0];
                        cfg_data_mread <= {27'd0, cfg_data_mwrite[4:0]};
                    end else begin
                        cfg_data_mread <= {27'd0,
                            error_buf[3],           // 4    sata_overflow
                            error_buf[2],           // 3    sata_underflow
                            error_buf[1],           // 2    udp_overflow
                            error_buf[0],           // 1    udp_underflow
                            error_cfg};             // 0
                    end
                end
                8'h01: begin    // Unused (3 B) | DAQ Module Acquire Enable (1 B)
                    if (cfg_mwrite_en) begin
                        acquire_en <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, acquire_en};
                    end
                end
                8'h02: begin    // Desired Start Board Sample Number (4 B)
                    if (cfg_mwrite_en) begin
                        desired_bsn <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                        sngdaq_reset_bsi <= 1'b1;
                    end else begin
                        cfg_data_mread <= desired_bsn;
                    end
                end
                8'h03: begin    // Current Board Sample Number (4 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= sngdaq_bsi;
                end
                8'h04: begin    // Chip alive bitmask (4 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= chips_alive_mask;
                end
                8'h05: begin    // CMD Write Enable (1 B) | CMD Chip Address (1 B) | CMD Command (2 B)
                    if (cfg_mwrite_en) begin
                        cmd_write_en <= cfg_data_mwrite[24];
                        cmd_chip_addr <= cfg_data_mwrite[23:16];
                        cmd_command <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= {7'd0, cfg_data_mwrite[24:0]};
                    end else begin
                        cfg_data_mread <= {7'd0, cmd_write_en, cmd_chip_addr, cmd_command};
                    end
                end
                8'h06: begin    // Synchronous Sampling (TODO)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= 32'd0;
                end
                8'h07: begin    // UDP FIFO count (bytes)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {19'd0, udp_fifo_data_count};
                end
                8'h08: begin    // UDP FIFO status
                    if (cfg_mwrite_en) begin
                        // This is one of those weird read/write corner cases.
                        udp_fifo_reset <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {24'd0,
                            udp_fifo_valid,         // 7
                            error_udp_overflow,     // 6
                            error_udp_underflow,    // 5
                            udp_fifo_full,          // 4
                            udp_fifo_empty,         // 3
                            udp_fifo_write_en,      // 2
                            udp_fifo_read_en,       // 1
                            udp_fifo_reset};        // 0
                    end
                end
                8'h09: begin    // Unused (3 B) | DAQ UDP Output Enable (1 B)
                    if (cfg_mwrite_en) begin
                        udp_en <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, udp_en};
                    end
                end
                8'h0A: begin    // Unused (3 B) | DAQ UDP Output Mode (1 B)
                    if (cfg_mwrite_en) begin
                        udp_full_mode <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, udp_full_mode};
                    end
                end
                8'h0B: begin    // Unused (3 B) | DAQ SATA Output Enable (1 B)
                    if (cfg_mwrite_en) begin
                        sata_en <= cfg_data_mwrite[0];
                        sata_align_start_en <= cfg_data_mwrite[1];
                        cfg_data_mread <= {30'd0, cfg_data_mwrite[1:0]};
                    end else begin
                        cfg_data_mread <= {30'd0, sata_align_start_en, sata_en};
                    end
                end
                8'h0C: begin    // DAQ SATA FIFO count (bytes)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {17'd0, sata_fifo_data_count};
                end
                8'h0D: begin    // DAQ SATA FIFO status
                    if (cfg_mwrite_en) begin
                        // This is one of those weird read/write corner cases.
                        sata_fifo_reset <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {23'd0,
                            sata_fifo_almost_full,  // 8
                            sata_fifo_almost_empty, // 7
                            error_sata_overflow,    // 6
                            error_sata_underflow,   // 5
                            sata_fifo_full,         // 4
                            sata_fifo_empty,        // 3
                            sata_fifo_write_en,     // 2
                            sata_fifo_read_en,      // 1
                            sata_fifo_reset};       // 0
                    end
                end
                8'h0E: begin    // Unused (3 B) | DAQ Intan Sample Extra Channels (1 B)
                    if (cfg_mwrite_en) begin
                        sample_extra_en <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, sample_extra_en};
                    end
                end
                default: begin
                    if (cfg_addr >= 8'd128 && cfg_addr < 8'd160) begin
                        // Sub-sample indexing
                        if (cfg_mwrite_en) begin
                            sschipi[cfg_addr[4:0]] <= cfg_data_mwrite[12:8];
                            sschani[cfg_addr[4:0]] <= cfg_data_mwrite[4:0];
                            cfg_data_mread <= {16'd0, 3'd0, cfg_data_mwrite[12:8],
                                                      3'd0, cfg_data_mwrite[4:0]};
                        end else begin
                            cfg_data_mread <= {16'd0, 3'd0, sschipi[cfg_addr[4:0]],
                                                      3'd0, sschani[cfg_addr[4:0]]};
                        end
                    end else begin
                        // configuration address error
                        error_cfg <= 1'b1;
                        cfg_data_mread <= 32'd0;
                    end
                end
                endcase
            end else begin
                cfg_sack <= 1'b0;
                // extra potentially conflicting logic goes here...
                sngdaq_reset_bsi <= 1'b0;
                error_buf[3] <= error_buf[3] || error_sata_overflow || sata_fifo_almost_full;
                error_buf[2] <= error_buf[2] || error_sata_underflow;
                error_buf[1] <= error_buf[1] || error_udp_overflow;
                error_buf[0] <= error_buf[0] || error_udp_underflow;
            end
        end
    end

    initial begin
        reset_subsample_indexes;
    end

    task reset_subsample_indexes;
        begin
        sschipi[ 0] <= 5'd00;  sschani[ 0] <= 5'd00;
        sschipi[ 1] <= 5'd00;  sschani[ 1] <= 5'd01;
        sschipi[ 2] <= 5'd00;  sschani[ 2] <= 5'd02;
        sschipi[ 3] <= 5'd00;  sschani[ 3] <= 5'd03;
        sschipi[ 4] <= 5'd00;  sschani[ 4] <= 5'd04;
        sschipi[ 5] <= 5'd00;  sschani[ 5] <= 5'd05;
        sschipi[ 6] <= 5'd00;  sschani[ 6] <= 5'd06;
        sschipi[ 7] <= 5'd00;  sschani[ 7] <= 5'd07;
        sschipi[ 8] <= 5'd00;  sschani[ 8] <= 5'd08;
        sschipi[ 9] <= 5'd00;  sschani[ 9] <= 5'd09;
        sschipi[10] <= 5'd00;  sschani[10] <= 5'd10;
        sschipi[11] <= 5'd00;  sschani[11] <= 5'd11;
        sschipi[12] <= 5'd00;  sschani[12] <= 5'd12;
        sschipi[13] <= 5'd00;  sschani[13] <= 5'd13;
        sschipi[14] <= 5'd00;  sschani[14] <= 5'd14;
        sschipi[15] <= 5'd00;  sschani[15] <= 5'd15;
        sschipi[16] <= 5'd00;  sschani[16] <= 5'd16;
        sschipi[17] <= 5'd00;  sschani[17] <= 5'd17;
        sschipi[18] <= 5'd00;  sschani[18] <= 5'd18;
        sschipi[19] <= 5'd00;  sschani[19] <= 5'd19;
        sschipi[20] <= 5'd00;  sschani[20] <= 5'd20;
        sschipi[21] <= 5'd00;  sschani[21] <= 5'd21;
        sschipi[22] <= 5'd00;  sschani[22] <= 5'd22;
        sschipi[23] <= 5'd00;  sschani[23] <= 5'd23;
        sschipi[24] <= 5'd00;  sschani[24] <= 5'd24;
        sschipi[25] <= 5'd00;  sschani[25] <= 5'd25;
        sschipi[26] <= 5'd00;  sschani[26] <= 5'd26;
        sschipi[27] <= 5'd00;  sschani[27] <= 5'd27;
        sschipi[28] <= 5'd00;  sschani[28] <= 5'd28;
        sschipi[29] <= 5'd00;  sschani[29] <= 5'd29;
        sschipi[30] <= 5'd00;  sschani[30] <= 5'd30;
        sschipi[31] <= 5'd00;  sschani[31] <= 5'd31;
        end
    endtask

endmodule

