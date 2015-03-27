/*
 * sata_core.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * Known Issues:
 *   - A read with length 0 will read a single sample
 *   - is_last flag needs testing
 *
 * Changes to the tables below should be included in the
 * "wiredleaf-registers.txt" file in the documentation repository.
 *
 * Register Map:
 *
 *      0x00     R/W     SATA Module Error Flags
 *      0x01     R/O     Unused (3 B) | SATA Module Mode (1 B)
 *      0x02     R/O     Unused (1 B) | SATA Module Status (3 B)
 *      0x03      -      Unused
 *      0x04      -      Unused
 *      0x05     R/W     Next Read Index (4 B)
 *      0x06     R/W     Read Length (4 B)
 *      0x07     R/0     Last Write Index (4 B)
 *      0x08     R/W     SATA Read FIFO Reset
 *      0x09     R/O     SATA Read FIFO Status
 *      0x0A     R/O     SATA Read FIFO Count
 *      0x0B     R/W     UDP FIFO Reset
 *      0x0C     R/W     UDP FIFO Status
 *      0x0D     R/W     UDP FIFO Count
 *      0x0E     R/O     Unused (2 B) | Current Disk Sector (High Bytes; 2 B)
 *      0x0F     R/O     Current Disk Sector (Low Bytes; 4 B)
 *      0x10     R/O     Delay Clock Freq in Hz (4 B)
 *      0x11     R/W     SATA Read Slowdown in Clock Cycles (4 B)
 *      0x12     R/W     Unused (1 B) | SATA Write Delay Cycles (3 B)
 *      0x13     R/O     Unused (2 B) | DAQ-SATA FIFO Count for Feedback (2 B)
 *      0x14     R/W     Write Starting Sample Index (4 B)
 *      0x15     R/W     Unused (2 B) | SATA Read Sector Count (512 byte sectors) (2 B)
 *
 *
 * Error Flags:
 *      0   Configuration Error
 *      1   Disk Not Ready
 *      2   SATA-UDP FIFO Underflow
 *      3   SATA-UDP FIFO Overflow
 *      4   SATA Read FIFO Underflow
 *      5   SATA Read FIFO Overflow
 *      6   Disk Removed
 *      7   Other error
 *      8   Low-Level SATA Error (see bits 23:16)
 *      23:16 SATA Error Code
 */

module sata_core (
    input clock,    // 44.1 MHZ
    input reset,

    output wire alive_led,
    output wire activity_led,
    output wire sata_io_ready,  // DEBUG

    output reg [31:0] start_write_index = 0,

    //// UDP FIFO interface
    output wire [15:0] udp_fifo_payload_len,
    output wire [7:0] udp_fifo_data,
    input wire udp_fifo_clock,
    input wire udp_fifo_read_en,
    output wire [12:0] udp_fifo_data_count,

    //// DAQ FIFO interface
    output wire daq_fifo_clock,
    output wire daq_fifo_read_en,
    input wire daq_fifo_empty,
    input wire [31:0] daq_fifo_data,
    input wire [14:0] daq_fifo_data_count,

    //// generic register configuration interface
    input [31:0] cfg_data_mwrite,
    output reg [31:0] cfg_data_mread = 32'b0,
    input [7:0] cfg_addr,
    input cfg_mread_en,
    input cfg_mwrite_en,
    output reg cfg_sack = 0,
    output wire error_flag,
    input wire any_error_flag, // "any error in any module"

    // GTP I/O
    output wire sata_txdt_p0,
    output wire sata_txdt_n0,
    input wire sata_rxdt_p0,
    input wire sata_rxdt_n0,

    // clocks
    input wire sata_clk
    );

    assign activity_led = sata_read_fifo_write_en || daq_fifo_read_en;
    assign sata_io_ready = sata_ready_flag;

    parameter CLOCK_FREQ_HZ = 32'd44_100_000;
    parameter DEFAULT_SLOWDOWN_CYCLES = 1470; // for < 30KHz packet frequency
    reg [31:0] slowdown_cycles = DEFAULT_SLOWDOWN_CYCLES;
    reg [31:0] slowdown_counter = 0;

    // error flags
    reg error_cfg = 0;
    reg error_other = 0;
    reg error_not_ready = 0;
    wire error_udp_underflow;
    wire error_udp_overflow;
    wire error_read_underflow;
    wire error_read_overflow;
    reg error_udp_underflow_buf = 0;
    reg error_udp_overflow_buf = 0;
    reg error_read_underflow_buf = 0;
    reg error_read_overflow_buf = 0;
    wire error_sata_lowlevel;
    wire [7:0] sata_error_code;
    reg [7:0] sata_error_code_buf = 0;
    reg error_disk_removed = 0;
    reg error_sata_lowlevel_buf = 0;
    assign error_flag = error_cfg || error_other || error_not_ready || error_udp_underflow_buf || error_udp_overflow_buf || error_sata_lowlevel_buf || error_disk_removed || error_read_underflow_buf || error_read_overflow_buf;

    reg [27:0] throb_counter = 0;

    localparam SAMPLE_WORDS = 565;  // len of raw board sample in 32bit words

    assign udp_fifo_payload_len = 16'd2264;     // 1x board sample + header
    reg [31:0] udp_fifo_data_in = 32'b0;

    wire udp_fifo_empty;
    wire udp_fifo_valid;
    reg udp_fifo_write_en = 0;
    wire sata2udp_fifo_reset;
    reg udp_fifo_reset = 0;
    assign sata2udp_fifo_reset = udp_fifo_reset || reset;
    sata2udp_fifo sata2udp_fifo_i0 (
        .rst(sata2udp_fifo_reset),
        .wr_clk(clock),
        .rd_clk(udp_fifo_clock),
        .din(udp_fifo_data_in),
        .wr_en(udp_fifo_write_en),
        .rd_en(udp_fifo_read_en),
        .dout(udp_fifo_data),
        .full(udp_fifo_full),
        .overflow(error_udp_overflow),
        .empty(udp_fifo_empty),
        .valid(udp_fifo_valid),
        .underflow(error_udp_underflow),
        .rd_data_count(udp_fifo_data_count),
        .wr_data_count()
        );

    /* Unimplemented
    wire [31:0] disk_identifier;
    wire [31:0] disk_io_parameters;
    */

    // read mode parameters
    reg [31:0] next_read_index = 0;
    reg [31:0] read_length = 0;
    reg [31:0] read_progress = 0;
    reg [31:0] last_write_index = 0;

    localparam SATA_MODE_WAIT = 0;
    localparam SATA_MODE_READ = 1;
    localparam SATA_MODE_WRITE = 2;

    localparam READ_STATE_START = 0;
    localparam READ_STATE_CONTINUE = 1;
    localparam READ_STATE_UNDERFLOW = 2;
    localparam READ_STATE_PADDING = 3;
    localparam READ_STATE_SLOWDOWN = 4;
    localparam READ_STATE_COMPLETE = 5;

    reg [1:0] sata_mode = SATA_MODE_WAIT;
    reg [2:0] read_state = READ_STATE_START;

    // 386.666us in 37.5MHz ticks
    //parameter DEFAULT_WRITE_DELAY_CYCLES = 14499;
    // about 66% of the above calculated value
    parameter DEFAULT_WRITE_DELAY_CYCLES = 0;
    reg [23:0] write_delay_cycles = DEFAULT_WRITE_DELAY_CYCLES;
    reg [14:0] feedback_count = 15'b0;

    parameter DEFAULT_SATA_READ_SECTOR_COUNT = 8;

    wire sata_ready_flag;
    wire phy_ready_flag;
    wire [3:0] sata_state;
    reg [3:0] sata_state_buf = 4'b0;
    reg sata_read_fifo_read_en = 0;
    reg sata_read_fifo_reset = 0;
    wire sata_read_fifo_write_en;
    wire [31:0] sata_read_fifo_data;
    wire sata_read_fifo_empty;
    wire sata_read_fifo_full;
    wire [9:0] sata_read_fifo_count;
    wire sata_clkout;
    wire [47:0] current_sector;
    reg [47:0] current_sector_buf = 48'b0;
    wire feedback_count_strobe;
    reg [15:0] sata_read_sector_count = DEFAULT_SATA_READ_SECTOR_COUNT;
    sata_top #( .SERDES_TYPE("SPARTAN6") ) sata_top_inst (

        .write_enable(sata_mode[1]),
        .read_enable(sata_mode[0]),
        .zero_enable(1'b0),

        .read_sectorcount(sata_read_sector_count),
        .write_sectorcount(17'd8192),
        .read_sectoraddress({13'd0, next_read_index, 3'd0}),
        .write_sectoraddress({13'd0, start_write_index, 3'd0}),
        .current_sectoraddress(current_sector),

        .write_delay_cycles(write_delay_cycles),

        .sata_txdt_p0(sata_txdt_p0),
        .sata_txdt_n0(sata_txdt_n0),
        .sata_rxdt_p0(sata_rxdt_p0),
        .sata_rxdt_n0(sata_rxdt_n0),

        .sata_clk(sata_clk),

        .RESET(reset),

        .clkout(sata_clkout),

        .sata_read_fifo_rd_clk(clock),
        .sata_read_fifo_empty(sata_read_fifo_empty),
        .sata_read_fifo_full(sata_read_fifo_full),
        .sata_read_fifo_write_en(sata_read_fifo_write_en),
        .sata_read_fifo_read_en(sata_read_fifo_read_en),
        .sata_read_fifo_overflow(error_read_overflow),
        .sata_read_fifo_underflow(error_read_underflow),
        .sata_read_fifo_reset(sata_read_fifo_reset),
        // SATA little-endian conversion
        .sata_read_fifo_dout({sata_read_fifo_data[7:0],
                              sata_read_fifo_data[15:8],
                              sata_read_fifo_data[23:16],
                              sata_read_fifo_data[31:24]}),
        .sata_read_fifo_rd_data_count(sata_read_fifo_count),

        .daq_fifo_clock(daq_fifo_clock),
        .daq_fifo_read_en(daq_fifo_read_en),
        .daq_fifo_empty(daq_fifo_empty),
        .daq_fifo_data(daq_fifo_data),
        .daq_fifo_feedback_count_strobe(feedback_count_strobe),

        .sata_ready(sata_ready_flag),
        .phy_ready(phy_ready_flag),
        .error_sata_lowlevel(error_sata_lowlevel),
        .sata_error_code(sata_error_code),
        .sata_state(sata_state)
        );

    reg [9:0] read_word_count = 0;
    reg is_last_flag = 0;

    always @(posedge clock) begin
        if (reset) begin
            error_cfg <= 1'b0;
            error_other <= 1'b0;
            error_not_ready <= 1'b0;
            error_disk_removed <= 1'b0;
            error_sata_lowlevel_buf <= 1'b0;
            error_udp_underflow_buf <= 1'b0;
            error_udp_overflow_buf <= 1'b0;
            error_read_underflow_buf <= 1'b0;
            error_read_overflow_buf <= 1'b0;
            sata_error_code_buf <= 8'd0;
            sata_mode <= SATA_MODE_WAIT;
            read_state <= READ_STATE_START;
            slowdown_cycles <= DEFAULT_SLOWDOWN_CYCLES;
            slowdown_counter <= 32'd0;
            last_write_index <= 32'd0;
            next_read_index <= 32'd0;
            start_write_index <= 32'd0;
            read_length <= 32'd0;
            read_progress <= 32'd0;
            sata_read_fifo_read_en <= 1'b0;
            sata_read_fifo_reset <= 1'b0;
            udp_fifo_write_en <= 1'b0;
            udp_fifo_data_in <= 32'd0;
            udp_fifo_reset <= 1'b0;
            read_word_count <= 10'd0;
            is_last_flag <= 1'b0;
            current_sector_buf <= 48'd0;
            sata_state_buf <= 4'd0;
            write_delay_cycles <= DEFAULT_WRITE_DELAY_CYCLES;
            sata_read_sector_count <= DEFAULT_SATA_READ_SECTOR_COUNT;
        end else begin
            current_sector_buf <= current_sector;
            sata_state_buf <= sata_state;
            /* ----------- Main Logic ---------- */
            case (sata_mode)
            SATA_MODE_WAIT: begin
                // clear out read FIFO/transaction
                read_state <= READ_STATE_START;
                read_progress <= 32'd0;
                udp_fifo_write_en <= 1'b0;
                is_last_flag <= 1'b0;
                sata_read_fifo_read_en <= 1'b0;
            end
            SATA_MODE_READ: begin
                case (read_state)
                READ_STATE_START: begin
                    if (sata_read_fifo_count > (SAMPLE_WORDS/2)) begin
                        // half a board sample is in the SATA read FIFO; ready
                        // to start copying to UDP. Will enter
                        // READ_STATE_CONTINUE on the second clock cycle.
                        sata_read_fifo_read_en <= 1'b1;
                        if (sata_read_fifo_read_en) begin
                            read_state <= READ_STATE_CONTINUE;
                            udp_fifo_write_en <= 1'b1;
                            read_word_count <= 10'd1;
                        end else begin
                            udp_fifo_write_en <= 1'b0;
                            read_word_count <= 10'd1;
                        end
                    end else begin
                        read_word_count <= 10'd0;
                        sata_read_fifo_read_en <= 1'b0;
                        udp_fifo_write_en <= 1'b0;
                    end
                    // UDP header word:
                    udp_fifo_data_in <= {
                        8'h5A,      // Magic number
                        8'h00,      // Proto Version
                        8'h81,      // Message Type: full samples
                        any_error_flag,     // error flag
                        1'b0,               // is_live: false
                        is_last_flag,       // is_last_flag
                        5'd0                // (unused flags)
                        };
                end
                READ_STATE_UNDERFLOW: begin
                    udp_fifo_write_en <= 1'b0;
                    if (sata_read_fifo_empty) begin
                        sata_read_fifo_read_en <= 1'b0;
                    end else begin
                        sata_read_fifo_read_en <= 1'b1;
                        if (sata_read_fifo_read_en) begin
                            read_state <= READ_STATE_CONTINUE;
                        end
                    end

                end
                READ_STATE_CONTINUE: begin
                    // continuation of a read transaction.
                    if (sata_read_fifo_empty) begin
                        read_state <= READ_STATE_UNDERFLOW;
                        udp_fifo_write_en <= 1'b0;
                        sata_read_fifo_read_en <= 1'b0;
                    end else begin
                        sata_read_fifo_read_en <= 1'b1;
                        udp_fifo_write_en <= 1'b1;
                        udp_fifo_data_in <= sata_read_fifo_data;
                        if (sata_read_fifo_read_en) begin
                            read_word_count <= read_word_count + 10'd1;
                            if (read_word_count >= SAMPLE_WORDS) begin
                                // "+ 1" so that udp_fifo_write_en is held
                                // high an extra cycle
                                // NB: udp_fifo_write_en etc are left high an
                                // extra cycle in this condition
                                read_state <= READ_STATE_PADDING;
                            end
                        end
                    end
                end
                READ_STATE_PADDING: begin
                    // condition: read_word_count >= SAMPLE_LENGTH
                    // end of UDP packet; read from SATA to skip inter-sample
                    udp_fifo_write_en <= 1'b0;
                    if (sata_read_fifo_empty) begin
                        sata_read_fifo_read_en <= 1'b0;
                    end else begin
                        sata_read_fifo_read_en <= 1'b1;
                        if (sata_read_fifo_read_en) begin
                            read_word_count <= read_word_count + 10'd1;
                            if (read_word_count == 10'd1022) begin
                                // NB: jump a clock cycle early as
                                // sata_read_fifo_read_en is left high.
                                slowdown_counter <= 32'd0;
                                read_state <= READ_STATE_SLOWDOWN;
                                read_progress <= read_progress + 32'd1;
                            end
                        end
                    end
                end
                READ_STATE_SLOWDOWN: begin
                    // this is the programmable delay period
                    read_word_count <= 10'd0;
                    udp_fifo_write_en <= 1'b0;
                    sata_read_fifo_read_en <= 1'b0;
                    slowdown_counter <= slowdown_counter + 32'd1;
                    if (read_progress + 32'd1 == read_length) begin
                        is_last_flag <= 1'b1;
                    end

                    if (read_progress >= read_length) begin
                        // done with the read!
                        read_state <= READ_STATE_COMPLETE;
                    end else begin
                        // pause before jumping to START
                        if (slowdown_counter >= slowdown_cycles) begin
                            read_state <= READ_STATE_START;
                        end
                    end
                end
                READ_STATE_COMPLETE: begin
                    // we're done, just hang out and wait for the logic at the
                    // end to put us back in WAIT
                    udp_fifo_write_en <= 1'b0;
                    sata_read_fifo_read_en <= 1'b0;
                    is_last_flag <= 1'b0;
                end
                default: begin
                    error_other <= 1'b1;
                    read_state <= READ_STATE_START;
                end
                endcase
            end
            SATA_MODE_WRITE: begin
                // clear out read FIFO/transaction
                udp_fifo_write_en <= 1'b0;
                read_state <= READ_STATE_START;
                last_write_index <= current_sector_buf[34:3];
                if (feedback_count_strobe) begin
                    feedback_count <= daq_fifo_data_count;
                    // TODO: automatically apply delay feedback correction
                    // here
                end
                sata_read_fifo_read_en <= 1'b0;
            end
            default: begin
                error_other <= 1'b1;
                sata_mode <= SATA_MODE_WAIT;
            end
            endcase
            /* ----------- Register Configuration Interface ---------- */
            if (cfg_mread_en || cfg_mwrite_en) begin
                cfg_sack <= 1'b1;
                case (cfg_addr)
                8'h00: begin  // SATA Module Error Flags
                    if (cfg_mwrite_en) begin
                        error_cfg <= cfg_data_mwrite[0];
                        error_not_ready <= cfg_data_mwrite[1];
                        error_udp_underflow_buf <= cfg_data_mwrite[2];
                        error_udp_overflow_buf <= cfg_data_mwrite[3];
                        error_read_underflow_buf <= cfg_data_mwrite[4];
                        error_read_overflow_buf <= cfg_data_mwrite[5];
                        error_disk_removed <= cfg_data_mwrite[6];
                        error_other <= cfg_data_mwrite[7];
                        error_sata_lowlevel_buf <= cfg_data_mwrite[8];
                        sata_error_code_buf <= cfg_data_mwrite[23:16];
                        cfg_data_mread <= {8'd0, cfg_data_mwrite[23:0]};
                    end else begin
                        cfg_data_mread <= {8'd0,
                            sata_error_code_buf,// 23:16; only if error_sata_lowlevel
                            7'd0,
                            error_sata_lowlevel,        // 8
                            error_other,                // 7
                            error_disk_removed,         // 6
                            error_read_overflow_buf,    // 5
                            error_read_underflow_buf,   // 4
                            error_udp_overflow_buf,     // 3
                            error_udp_underflow_buf,    // 2
                            error_not_ready,            // 1
                            error_cfg};                 // 0
                    end
                end
                8'h01: begin  // Unused (3 B) | SATA Module Mode (1 B)
                    if (cfg_mwrite_en) begin
                        sata_mode <= cfg_data_mwrite[1:0];
                        cfg_data_mread <= {30'd0, cfg_data_mwrite[1:0]};
                        if (!sata_ready_flag &&
                            (cfg_data_mwrite[1:0] != 2'b00)) begin
                            // read or write without device ready
                            error_not_ready <= 1'b1;
                        end
                    end else begin
                        cfg_data_mread <= {30'd0, sata_mode};
                    end
                end
                8'h02: begin  // Unused (1 B) | SATA Module Status (3 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end else begin
                        cfg_data_mread <= {8'd0,
                                           4'd0,
                                           read_state,
                                           3'd0,
                                           sata_state_buf,
                                           6'd0,
                                           sata_ready_flag,
                                           phy_ready_flag};
                    end
                end
                8'h03: begin  // Unused
                /* Would be "Disk Identifier", but this isn't passed through
                   from SATA module.

                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= disk_identifier;
                */
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end else begin
                        cfg_data_mread <= 32'd0;
                    end
                end
                8'h04: begin  // Unused
                /* Would be "Disk I/O parameters", but this isn't passed
                   through from SATA module.

                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= disk_io_parameters;
                */
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end else begin
                        cfg_data_mread <= 32'd0;
                    end
                end
                8'h05: begin // Next Read Index (4 B)
                    if (cfg_mwrite_en) begin
                        next_read_index <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= next_read_index;
                    end
                end
                8'h06: begin // Read Length (4 B)
                    if (cfg_mwrite_en) begin
                        read_length <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= read_length;
                    end
                end
                8'h07: begin  // Last Write Index (4 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= last_write_index;
                end
                8'h08: begin // SATA Read FIFO Reset
                    if (cfg_mwrite_en) begin
                        sata_read_fifo_reset <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, sata_read_fifo_reset};
                    end
                end
                8'h09: begin  // SATA Read FIFO Status
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    // This is weird, because it's trying to mimick the UDP
                    // status lines
                    cfg_data_mread <= {24'd0, 1'd0, 1'd0,
                        sata_read_fifo_full,    // 6
                        sata_read_fifo_empty,   // 5
                        error_read_overflow,    // 4
                        error_read_underflow,   // 3
                        sata_read_fifo_write_en,// 2
                        sata_read_fifo_read_en, // 1
                        sata_read_fifo_reset};  // 0
                end
                8'h0A: begin  // SATA Read FIFO Count
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {22'd0, sata_read_fifo_count};
                end
                8'h0B: begin  // UDP FIFO Reset
                    if (cfg_mwrite_en) begin
                        udp_fifo_reset <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, udp_fifo_reset};
                    end
                end
                8'h0C: begin  // UDP FIFO Status
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {24'd0,
                        udp_fifo_valid,         // 7
                        udp_fifo_full,          // 6
                        udp_fifo_empty,         // 5
                        error_udp_overflow,     // 4
                        error_udp_underflow,    // 3
                        udp_fifo_write_en,      // 2
                        udp_fifo_read_en,       // 1
                        udp_fifo_reset};        // 0
                end
                8'h0D: begin  // UDP FIFO Count
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {19'd0, udp_fifo_data_count};
                end
                8'h0E: begin  // Current Disk Sector (Upper Bytes)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, current_sector_buf[47:32]};
                end
                8'h0F: begin  // Current Disk Sector (Lower Bytes)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= current_sector[31:0];
                end
                8'h10: begin  // Clock Freq in Hz
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= CLOCK_FREQ_HZ;
                end
                8'h11: begin  // SATA Read Slowdown in Clock Cycles
                    if (cfg_mwrite_en) begin
                        slowdown_cycles <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= slowdown_cycles;
                    end
                end
                8'h12: begin  // Unused (1 B) | SATA Write Delay Cycles (3 B)
                    if (cfg_mwrite_en) begin
                        write_delay_cycles <= cfg_data_mwrite[23:0];
                        cfg_data_mread <= {8'd0, cfg_data_mwrite[23:0]};
                    end else begin
                        cfg_data_mread <= {8'd0, write_delay_cycles};
                    end
                end
                8'h13: begin  // Unused (2 B) | DAQ-SATA FIFO Count for Feedback (2 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {17'd0, feedback_count};
                end
                8'h14: begin  // Write Starting Sample Index (4 B)
                    if (cfg_mwrite_en) begin
                        start_write_index <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= start_write_index;
                    end
                end
                8'h15: begin  // SATA Read Sector Count (512 byte sectors) (2 B)
                    if (cfg_mwrite_en) begin
                        sata_read_sector_count <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= {16'd0, sata_read_sector_count};
                    end
                end
                default: begin
                    // invalid register address
                    error_cfg <= 1'b1;
                end
                endcase
            end else begin
                cfg_sack <= 1'b0;
                // cleanup logic to safely change sata_mode
                if ((sata_mode == SATA_MODE_READ)
                    && (read_state == READ_STATE_COMPLETE)) begin
                    sata_mode <= SATA_MODE_WAIT;
                end
                // check for error conditions
                if (sata_mode == 2'b11) begin
                    error_cfg <= 1'b1;
                end
                if (!phy_ready_flag && (sata_mode != 2'b00)) begin
                    error_disk_removed <= 1'b1;
                end
                if (error_sata_lowlevel && !error_sata_lowlevel_buf) begin
                    // fresh rising edge of error_sata_lowlevel
                    error_sata_lowlevel_buf <= 1'b1;
                    sata_error_code_buf <= sata_error_code;
                end
                error_udp_underflow_buf <= error_udp_underflow_buf || error_udp_underflow;
                error_udp_overflow_buf <= error_udp_overflow_buf || error_udp_overflow;
                error_read_underflow_buf <= error_read_underflow_buf || error_read_underflow;
                error_read_overflow_buf <= error_read_overflow_buf || error_read_overflow;
            end
        end
    end

    // setup a throbber LED which either blinks once a second (based on
    // sata_clkout @ 37.5MHz) or is on solid (based on device ready)
    reg throb_led = 1'b0;
    assign alive_led = throb_led || sata_ready_flag;
    always @(posedge sata_clkout) begin
        if (reset) begin
            throb_counter <= 28'd0;
            throb_led <= 1'b0;
        end else begin
            if (throb_counter <= 37_500_000/2) begin
                throb_counter <= throb_counter + 28'd1;
            end else begin
                throb_led <= ~throb_led;
                throb_counter <= 0;
            end
        end
    end

endmodule

