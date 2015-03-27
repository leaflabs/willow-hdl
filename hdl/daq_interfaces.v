/*
 * daq_interfaces.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: June 2013
 *
 * TODO:
 *  - interface docs up here
 *  - functional docs up here
 *
 * Quirks:
 *  - sata_started flag is used to enforce writes always starting with
 *    board_sample_index == sata_start_index. This is controlled by the
 *    sata_align_start_en register flag.
 *
 *  SIMULATION NOTE: 79539ns
 */
module daq_interfaces (
    input clock,
    input reset,

    input udp_enable,
    input udp_full_mode,
    input sata_enable,
    input sata_align_start_en,
    input [31:0] sata_start_index,
    input [63:0] experiment_cookie,
    input [15:0] gpio_pins,
    input wire any_error_flag,
    input [31:0] board_identifier,

    output wire error_udp_overflow,
    output wire error_udp_underflow,
    output reg         error_sata_overflow = 0,
    output reg         error_sata_underflow = 0,

    input [159:0] subsample_chip_indexes,
    input [159:0] subsample_channel_indexes,

    input wire [7:0] dac_state,

    //// DAQ BRAM interface
    output wire daq_bram_clock,
    output reg [9:0] daq_bram_addr = 0,
    input [31:0] daq_bram_data,
    output reg daq_bram_read_en = 0,
    input daq_data_ready,
    input [31:0] daq_board_sample_index,
    input [31:0] daq_chip_alive_mask,

    //// UDP FIFO interface
    output reg [15:0] udp_fifo_payload_len = (PARTIAL_SAMPLE_LENGTH + 1) * 4,
    output wire [7:0] udp_fifo_data,
    input wire udp_fifo_clock,
    input wire udp_fifo_read_en,
    output reg udp_fifo_write_en = 0,
    output wire [12:0] udp_fifo_data_count,
    output wire udp_fifo_empty,
    output wire udp_fifo_full,
    output wire udp_fifo_valid,
    input wire udp_fifo_reset,

    //// SATA FIFO interface
    output wire [31:0] sata_fifo_data,
    input wire sata_fifo_clock,
    input wire sata_fifo_read_en,
    output reg sata_fifo_write_en = 0,
    output wire [14:0] sata_fifo_data_count,
    output wire sata_fifo_empty,
    output wire sata_fifo_almost_empty,
    output wire sata_fifo_almost_full,
    output wire sata_fifo_full,
    input wire         sata_fifo_reset,

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

assign daq_bram_clock = clock;

wire [4:0] sschipi [31:0];    // chip indexes
wire [3:0] sschanih [31:0];   // channel index, minus last bit ("high")
wire [31:0] sschanip;         // last bit of channel index ("parity")
genvar cntr;
for (cntr = 0; cntr < 32; cntr = cntr + 1) begin : subsample_index_loop
    assign sschipi[cntr] = subsample_chip_indexes[(cntr*5)+4:(cntr*5)];
    assign sschanih[cntr] = subsample_channel_indexes[(cntr*5)+4:(cntr*5)+1];
    assign sschanip[cntr] = subsample_channel_indexes[(cntr*5)];
end

// UDP wire format adds an extra 4 bytes (1 word) to the below
parameter FULL_SAMPLE_LENGTH = 565; // 2260 bytes, 565 32bit words
parameter PARTIAL_SAMPLE_LENGTH = 38; // 152 bytes, 38 32bit words

reg [31:0] udp_fifo_data_in = 0;
wire daq2udp_fifo_reset;
assign daq2udp_fifo_reset = udp_fifo_reset || reset;
daq2udp_fifo daq2udp_fifo_i0 (
  .rst(daq2udp_fifo_reset),
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

reg [31:0] sata_fifo_data_in = 0;
wire daq2sata_fifo_reset;
assign daq2sata_fifo_reset = sata_fifo_reset || reset;
assign ddr3_reset = daq2sata_fifo_reset;

assign ddr3_wr_clk  = clock;
assign ddr3_wr_stb  = sata_fifo_write_en;

assign ddr3_wr_data = {sata_fifo_data_in[7:0],
        sata_fifo_data_in[15:8],
        sata_fifo_data_in[23:16],
                       sata_fifo_data_in[31:24]};

assign sata_fifo_full = ddr3_wr_full;
assign sata_fifo_data_count = ddr3_wr_count;

assign ddr3_rd_clk     = sata_fifo_clock;
assign ddr3_rd_stb     = sata_fifo_read_en;
assign sata_fifo_data  = ddr3_rd_data;
assign sata_fifo_empty = ddr3_rd_empty;

// no idea if this is right, why are they separate signals in the first place? TODO
always @ (posedge ddr3_wr_clk) begin
   if (daq2sata_fifo_reset) begin
      error_sata_overflow <= 1'b0;
   end else begin
      if (sata_fifo_full && sata_fifo_write_en)
        error_sata_overflow <= 1'b1;
      else
        error_sata_overflow <= 1'b0;
   end
end

always @ (posedge ddr3_rd_clk) begin
  if (daq2sata_fifo_reset) begin
     error_sata_underflow <= 1'b0;
  end else begin
     if (sata_fifo_empty && sata_fifo_read_en)
       error_sata_underflow <= 1'b1;
     else
       error_sata_underflow <= 1'b0;
  end
end

assign sata_fifo_almost_empty = sata_fifo_empty;
assign sata_fifo_almost_full = sata_fifo_full; // This signal is not needed with the DDR approach. In both DDR and BRAM FIFO instances, however, *any* signalling of either almost_full or full indicates a loss of data. Almost_full simply allows the loss to be boundary aligned/recoverable

localparam STATE_WAIT = 0;
localparam STATE_READY = 1;
localparam STATE_UDP = 2;
localparam STATE_UDP_FULL = 3;
localparam STATE_SATA = 4;

reg [3:0] state = STATE_WAIT;
reg [31:0] last_board_sample_index = 0;
reg bram_flipped = 0;
reg [9:0] word_count = 0;

reg [5:0] udp_ptr = 0;

reg [15:0] dac_waveform = 0;

reg sata_started = 0;

// Packet Header Flags
wire is_live_flag;
assign is_live_flag = 1'b1;
wire is_last_flag;
assign is_last_flag = 1'b0;

always @(posedge clock) begin
    if (reset) begin
        state <= STATE_WAIT;
        last_board_sample_index <= daq_board_sample_index;
        word_count <= 0;
        udp_fifo_data_in <= 0;
        daq_bram_read_en <= 1'b0;
        daq_bram_addr <= 0;
        udp_fifo_write_en <= 1'b0;
        udp_fifo_payload_len <= (PARTIAL_SAMPLE_LENGTH + 1) * 4;
        udp_ptr <= 6'd0;
        sata_fifo_write_en <= 1'b0;
        dac_waveform <= 16'd0;
        sata_started <= 1'b0;
    end else begin
        if (udp_full_mode) begin
            udp_fifo_payload_len <= (FULL_SAMPLE_LENGTH + 1) * 4;
        end else begin
            udp_fifo_payload_len <= (PARTIAL_SAMPLE_LENGTH + 1) * 4;
        end
        last_board_sample_index <= daq_board_sample_index;
        bram_flipped <= (last_board_sample_index != daq_board_sample_index);
        case (state)
        STATE_WAIT: begin
            udp_fifo_write_en <= 1'b0;
            sata_fifo_write_en <= 1'b0;
            sata_started <= 1'b0;
            if (udp_enable || sata_enable) begin
                // wait for gap between board samples
                state <= STATE_READY;
            end
        end
        STATE_READY: begin
            word_count <= 10'd0;
            udp_fifo_write_en <= 1'b0;
            sata_fifo_write_en <= 1'b0;
            daq_bram_addr <= 0;
            daq_bram_read_en <= 1'b0;
            if (!(udp_enable || sata_enable)) begin
                state <= STATE_WAIT;
            end else if (bram_flipped && daq_data_ready) begin
                if (sata_enable && !sata_fifo_almost_full) begin
                    state <= STATE_SATA;
                end else if (udp_enable) begin
                    if (udp_full_mode) begin
                        state <= STATE_UDP_FULL;
                    end else begin
                        state <= STATE_UDP;
                    end
                end else begin
                    state <= STATE_READY;
                end
            end else begin
                state <= STATE_READY;
            end
        end
        STATE_UDP: begin
            sata_fifo_write_en <= 1'b0;
            case (word_count)
            0: begin    // wired leaf header
                udp_fifo_write_en <= 1'b1;
                word_count <= word_count + 10'd1;
                // Magic number | Proto Version | Message Type | Status Flags
                // Status flags:
                //      bit 7   error
                //      bit 6   is_live
                //      bit 5   is_last_sample
                //      others  unused
                // 8'h80 message type for sub-samples
                udp_fifo_data_in <= {8'h5A, 8'h00, 8'h80,
                    any_error_flag, is_live_flag, is_last_flag, 5'd0};
            end
            1: begin    // experiment cookie upper word
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= experiment_cookie[63:32];
            end
            2: begin    // experiment cookie lower word
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= experiment_cookie[31:0];
                // read dac waveform during header; takes 2 cycles
                // DEBUG: read from chip #3, not chip #0
                daq_bram_addr <= 10'd0512 + 10'd4;
                daq_bram_read_en <= 1'b1;
            end
            3: begin    // board identifier
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= board_identifier;
            end
            4: begin    // sample index
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= daq_board_sample_index;
                // finish reading dac waveform, and state byte
                // DEBUG: read from chip #3, not chip #0
                dac_waveform <= {dac_state, daq_bram_data[23:16]};
                daq_bram_read_en <= 1'b0;
            end
            5: begin    // chip live status
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= daq_chip_alive_mask;
            end
            6,7,8,9,10,11,12,13,14,15,16,17,18,19,20: begin
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= {3'd0, sschipi[(word_count-6)*2],
                                     3'd0, sschanih[(word_count-6)*2], sschanip[(word_count-6)*2],
                                     3'd0, sschipi[(word_count-6)*2+1],
                                     3'd0, sschanih[(word_count-6)*2+1], sschanip[(word_count-6)*2+1]};
            end
            21: begin // last of channel indexes
                word_count <= word_count + 10'd1;
                udp_fifo_data_in <= {3'd0, sschipi[30],
                                     3'd0, sschanih[30], sschanip[30],
                                     3'd0, sschipi[31],
                                     3'd0, sschanih[31], sschanip[31]};
                udp_ptr <= 6'd0;
                daq_bram_addr <= {1'b0, sschipi[0], sschanih[0]};
                daq_bram_read_en <= 1'b1;
            end
            39: begin // last word: DAC and GPIO
                word_count <= word_count + 10'd1;
                udp_fifo_write_en <= 1'b1;
                udp_fifo_data_in <= {gpio_pins, dac_waveform};
            end
            40: begin
                // nothing; handled in if statement below
            end
            default: begin // data from DAQ BRAM
                // note: in BRAM, the "left" word is higher index
                //       in FIFO, the "right" word is higher index
                udp_ptr <= udp_ptr + 6'd1;
                daq_bram_addr <= {1'b0, sschipi[udp_ptr[4:0]+5'd1], sschanih[udp_ptr[4:0]+5'd1]};
                // There is an extra cycle of delay for BRAM data fetch, so
                // base logic on (udp_ptr-1)
                if (udp_ptr[0]) begin // evens (low bit of udp_ptr - 1)
                    word_count <= word_count + 10'd1;
                    udp_fifo_write_en <= 1'b0;
                    if (sschanip[udp_ptr[4:0]-5'd1]) begin
                        udp_fifo_data_in[31:16] <= daq_bram_data[31:16];
                    end else begin
                        udp_fifo_data_in[31:16] <= daq_bram_data[15:0];
                    end
                end else begin // odds (low bit of udp_ptr - 1)
                    if (udp_ptr > 0) begin
                        udp_fifo_write_en <= 1'b1;
                    end else begin
                        udp_fifo_write_en <= 1'b0;
                    end
                    if (sschanip[udp_ptr[4:0]-5'd1]) begin
                        udp_fifo_data_in[15:0] <= daq_bram_data[31:16];
                    end else begin
                        udp_fifo_data_in[15:0] <= daq_bram_data[15:0];
                    end
                end
            end
            endcase
            if (error_udp_overflow) begin
                state <= STATE_WAIT;
            end else if (word_count > PARTIAL_SAMPLE_LENGTH+1) begin
                // this was the last word
                word_count <= 0;
                daq_bram_read_en <= 1'b0;
                udp_fifo_write_en <= 1'b0;
                state <= STATE_READY;
            end
        end
        STATE_UDP_FULL: begin
            sata_fifo_write_en <= 1'b0;
            case (word_count)
            0: begin    // wired leaf header
                udp_fifo_write_en <= 1'b1;
                // Status flags:
                //      bit 7   error
                //      bit 6   is_live
                //      bit 5   is_last_sample
                //      others  unused
                // 8'h81 message type for full samples
                udp_fifo_data_in <= {8'h5A, 8'h00, 8'h81,
                    any_error_flag, is_live_flag, is_last_flag, 5'd0};
            end
            1: begin    // experiment cookie upper word
                udp_fifo_data_in <= experiment_cookie[63:32];
            end
            2: begin    // experiment cookie lower word
                udp_fifo_data_in <= experiment_cookie[31:0];
            end
            3: begin    // board identifier
                udp_fifo_data_in <= board_identifier;
            end
            4: begin    // sample index
                udp_fifo_data_in <= daq_board_sample_index;
                daq_bram_addr <= 10'd0; // needs to be a clock ahead
                daq_bram_read_en <= 1'b1;
            end
            5: begin    // chip live status
                udp_fifo_data_in <= daq_chip_alive_mask;
                daq_bram_addr <= 10'd1; // needs to be a clock ahead
            end
            default: begin // data from DAQ BRAM
                udp_fifo_data_in <= {daq_bram_data[15:0], daq_bram_data[31:16]};
                if (daq_bram_addr < 559) begin
                    daq_bram_addr <= daq_bram_addr + 10'd1;
                end
            end
            endcase
            if (error_udp_overflow) begin
                state <= STATE_WAIT;
            end else if (word_count <= (FULL_SAMPLE_LENGTH-2) + 1) begin
                // "+ 1" is for the extra 4-byte wire format header
                word_count <= word_count + 10'd1;
            end else begin
                // this is the last word
                word_count <= 0;
                state <= STATE_READY;
            end
        end
        STATE_SATA: begin
            udp_fifo_write_en <= 1'b0;
            case (word_count)
            0: begin
                // This first case block (word_count == 0) does not do much
                // heavy lifting, which is inefficient. This is intentional,
                // to keep the overall timing consistent with STATE_UDP_FULL,
                // to make debugging and copy-pasta easier between the two
                // (mostly identical) blocks of code.
                if ((sata_align_start_en && (!sata_started))
                        && (daq_board_sample_index != sata_start_index)) begin
                    state <= STATE_READY;
                end
            end
            1: begin    // experiment cookie upper word
                sata_started <= 1'b1;
                sata_fifo_write_en <= 1'b1;
                sata_fifo_data_in <= experiment_cookie[63:32];
            end
            2: begin    // experiment cookie lower word
                sata_fifo_data_in <= experiment_cookie[31:0];
            end
            3: begin    // board identifier
                sata_fifo_data_in <= board_identifier;
            end
            4: begin    // sample index
                sata_fifo_data_in <= daq_board_sample_index;
                daq_bram_addr <= 10'd0; // needs to be a clock ahead
                daq_bram_read_en <= 1'b1;
            end
            5: begin    // chip live status
                sata_fifo_data_in <= daq_chip_alive_mask;
                daq_bram_addr <= 10'd1; // needs to be a clock ahead
            end
            default: begin // data from DAQ BRAM
                sata_fifo_data_in <= {daq_bram_data[15:0], daq_bram_data[31:16]};
                if (daq_bram_addr < 559) begin
                    daq_bram_addr <= daq_bram_addr + 10'd1;
                end
            end
            endcase
            if (error_sata_overflow) begin
                state <= STATE_WAIT;
            end else if (word_count <= (FULL_SAMPLE_LENGTH-2) + 1) begin
                // "+ 1" is for consistency with STATE_UDP_FULL
                word_count <= word_count + 10'd1;
            end else begin
                // this is the last word
                word_count <= 0;
                if (udp_enable) begin
                    if (udp_full_mode) begin
                        state <= STATE_UDP_FULL;
                    end else begin
                        state <= STATE_UDP;
                    end
                end else begin
                    state <= STATE_READY;
                end
            end
        end
        default: begin
            state <= STATE_WAIT;
        end
        endcase
    end
end

endmodule
