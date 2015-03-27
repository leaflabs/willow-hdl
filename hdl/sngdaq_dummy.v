/*
 * sngdaq_dummy.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: June 2013
 *
 * A dummy (synthetic data, no SPI) version of the sngdaq module.
 *
 * The full sngdaq interface is duplicated so that daq_core.v will be
 * compatible with either sngdaq or sngdaq_dummy.
 *
 * Write scheme:
 *      chip_index loops fastest (44.1MHz; clk_i)
 *      then spi_slot (at 1.05MHz; spi_clock)
 *      then board sample index (at 30KHz; board_clock)
 *
 * Output board sample:
 *
 *  TODO:
 *
 * TO-DO/BUGS:
 * - extra CMD simulation is borked
 * - handle enable_acq falling edge properly (pseudo-reset?)
 * - this file should simulate IDENT
 * - a parameter to this file should allow marking some chips at not-alive
 */

module sngdaq_dummy (
    input wire clk_i, // 44.1 MHz clock
    input wire reset_i,
    input wire enable_acq,
    //// SPI I/O lines
    output wire spi_csbar_o,
    output wire spi_sclk_o,
    output wire spi_mosi_o,
    input wire[31:0] spi_miso_i,
    //// BRAM interface
    input wire data_clk_i, // BRAM read clock
    input wire data_en_i, // BRAM read enable
    input wire[9:0] data_addr_i, // BRAM read address
    output wire[31:0] data_o, // BRAM data
    output wire data_ready_o, // "ok to read from BRAM"
    //// metadata and other ports
    output reg[31:0] board_sample_index_o = 32'b0, // board sample number of BRAM contents
    output wire[(NUM_CHIPS-1):0] spi_ready_o, // status of all SPI drivers
    input wire[15:0] extra_cmd_i,
    input wire[5:0] extra_cmd_chip_i,
    input wire extra_cmd_en_i,
    input wire sample_extra_en_i,
    input wire [15:0] gpio_pins,
    output wire [7:0] dac_state_o,
    input wire[31:0] initial_board_sample_index_i,
    input wire reset_board_sample_index_i
    );

    localparam NUM_CHIPS = 32; // copy-pasta
    parameter MOSI_ON_FALLING = 1;  // ignored
    parameter MOSI_ON_NEXT_RISING = 1; // ignored

    // we are a dummy!
    assign spi_csbar_o = 1'b0;
    assign spi_sclk_o = 1'b0;
    assign spi_mosi_o = 1'b0;
    assign spi_ready_o = 32'hFFFF_FFFF;

    reg reset_board_sample_index = 0; // confusing buffer variable

    reg data_ready = 0;
    assign data_ready_o = data_ready;

    wire bram_flip;
    assign bram_flip = board_sample_index_o[0];
    reg bram_in_we = 0;
    reg[10:0] bram_in_addr = 0;
    reg[15:0] bram_in_data = 0;

    wire bram0_in_we;
    assign bram0_in_we = bram_flip && bram_in_we;
    wire bram0_out_re;
    assign bram0_out_re = (!bram_flip) && data_en_i;
    wire[31:0] bram0_out_data;
    daq_bram inst_daq_bram0(
        .clka(clk_i),
        .wea(bram0_in_we),
        .addra(bram_in_addr),
        .dina(bram_in_data),
        .clkb(data_clk_i),
        .enb(bram0_out_re),
        .addrb(data_addr_i),
        .doutb(bram0_out_data)
        );
    wire bram1_in_we;
    assign bram1_in_we = (!bram_flip) && bram_in_we;
    wire bram1_out_re;
    assign bram1_out_re = bram_flip && data_en_i;
    wire[31:0] bram1_out_data;
    daq_bram inst_daq_bram1(
        .clka(clk_i),
        .wea(bram1_in_we),
        .addra(bram_in_addr),
        .dina(bram_in_data),
        .clkb(data_clk_i),
        .enb(bram1_out_re),
        .addrb(data_addr_i),
        .doutb(bram1_out_data)
        );
    assign data_o = bram_flip ? bram1_out_data : bram0_out_data;

    // DAC waveform from a sine wave lookup table
    // in general, dac_waveform_index = ((board_sample_index_o + 8'd2) % 30)
    wire [7:0] dac_waveform_value;
    reg [4:0] dac_waveform_index = 5'd1;
    sine_table inst_sine_table (
        .clk_i(clk_i),
        // take the "next" sample index, modulo 30, as the sine index
        .addr_i(dac_waveform_index),
        .value_o(dac_waveform_value)
        );
    // A second DAC waveform, with an off-by-three phase
    wire [7:0] dac_fazed_value;
    reg [4:0] dac_fazed_index = 5'd5;
    sine_table inst_faze_table (
        .clk_i(clk_i),
        .addr_i(dac_fazed_index),
        .value_o(dac_fazed_value)
        );

    reg [4:0] dac_select = 5'd0;
    reg dac_power = 0;
    reg dac_enable = 0;
    reg [1:0] dac_scale = 0;
    assign dac_state_o = {(dac_power && dac_enable), dac_scale, dac_select};

    reg[2:0] state = STATE_WAIT;
    localparam STATE_WAIT = 0;
    localparam STATE_LOOP_READ = 1;
    localparam STATE_LOOP_WAIT = 2;

    reg[4:0] chip_index = 0;
    reg[5:0] last_slot = 0;
    reg [5:0] spi_slot = 0;
    reg [5:0] spi_slot_delay = 0; // dummy-specific counter

    // t*(42&t>>10); (t*9&t>>4|t*5&t>>7|t*3&t/1024)-1; (t>>6)&(2*t)&(t>>1)
    reg [31:0] work_four_one = 0;
    reg [31:0] work_four_two = 0;
    reg [31:0] work_four_thr = 0;

    always @(posedge clk_i) begin
        if (reset_i) begin
            state <= STATE_WAIT;
            bram_in_we <= 1'b0;
            data_ready <= 1'b0;
            chip_index <= 5'd0;
            last_slot <= 6'd0;
            board_sample_index_o <= 32'd0;
            dac_waveform_index <= 5'd1;
            dac_fazed_index <= 5'd5;
            reset_board_sample_index <= 0;
            spi_slot <= 6'd0;
            spi_slot_delay <= 6'd0;
            dac_select <= 5'd0;
            dac_enable <= 0;
            dac_power <= 0;
            dac_scale <= 2'd0;
        end else begin
            // ------- board sample reset stuff ---------
            if (reset_board_sample_index_i && !reset_board_sample_index) begin
                // is the index already set correctly?
                if (initial_board_sample_index_i != board_sample_index_o) begin
                    // need to reset index as soon as possible
                    reset_board_sample_index <= 1'b1;
                end
            end
            // ------- increment/loop spi_slot ---------
            if (spi_slot_delay == 41) begin
                spi_slot_delay <= 6'd0;
                if (spi_slot >= 34) begin
                    spi_slot <= 0;
                end else begin
                    spi_slot <= spi_slot + 6'd1;
                end
            end else begin
                spi_slot_delay <= spi_slot_delay + 6'd1;
            end
            // ------- main state machine ---------
            case (state)
            STATE_WAIT: begin
                if (enable_acq) begin
                    state <= STATE_LOOP_READ;
                    chip_index <= 5'd0;
                end
            end
            STATE_LOOP_READ: begin
                if (enable_acq) begin
                    if (board_sample_index_o > 32'd1) begin
                        // did we write to BRAM yet?
                        data_ready <= 1'b1;
                    end
                    if (chip_index == (NUM_CHIPS-1)) begin
                        // this is the last chip for this spi_slot
                        state <= STATE_LOOP_WAIT;
                        if (spi_slot == 34) begin
                            // also, end of board sample
                            if (dac_waveform_index >= 5'd29) begin
                                dac_waveform_index <= 5'd0;
                            end else begin
                                dac_waveform_index <= dac_waveform_index + 5'd1;
                            end
                            if (dac_fazed_index >= 5'd29) begin
                                dac_fazed_index <= 5'd0;
                            end else begin
                                dac_fazed_index <= dac_fazed_index + 5'd1;
                            end
                            // XXX: this code *should* check for sample
                            // indexes modulo 30, but does not because the
                            // division is too slow (~14MHz max).
                            // For now it is the responsibility of the user to
                            // only ever change the board sample number to
                            // something modulo 30.
                            // Else data will be corrupt.
                            // Bad.
                            if (reset_board_sample_index && (dac_waveform_index == 5'd1)) begin
                                board_sample_index_o <= initial_board_sample_index_i;
                                reset_board_sample_index <= 1'b0;
                            end else begin
                                board_sample_index_o <= board_sample_index_o + 32'd1;
                            end
                        end
                    end
                    last_slot <= spi_slot;
                    chip_index <= chip_index + 5'd1;
                    bram_in_we <= 1'b1;
                    case (spi_slot)
                    0: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= board_sample_index_o[15:0];
                        work_four_two <= {10'd0, board_sample_index_o[31:10]};
                        work_four_one <= board_sample_index_o * 32'd9;
                    end
                    1: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= board_sample_index_o[16:1];
                        work_four_two <= work_four_two & 32'd42;
                        work_four_thr <= board_sample_index_o * 32'd3;
                    end
                    2: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= board_sample_index_o[17:2];
                        work_four_two <= board_sample_index_o * work_four_two;
                        work_four_one <= work_four_one & {4'd0, board_sample_index_o[31:4]};
                    end
                    3: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= work_four_two[15:0];
                        work_four_thr <= work_four_thr & {10'd0, board_sample_index_o[31:10]};
                        work_four_two <= board_sample_index_o * 32'd5;
                    end
                    4: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= board_sample_index_o[19:4];
                        work_four_two <= work_four_two & {7'd0, board_sample_index_o[31:7]};
                        work_four_one <= work_four_one | work_four_thr;
                        work_four_thr <= {1'd0, board_sample_index_o[31:1]};
                    end
                    5: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= board_sample_index_o[20:5];
                        work_four_one <= work_four_one | work_four_two;
                        work_four_thr <= work_four_thr & {6'd0, board_sample_index_o[31:6]};
                    end
                    6: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= {5'd0, board_sample_index_o[31:21]};
                        work_four_one <= work_four_one - 32'd1;
                        work_four_thr <= work_four_thr & {board_sample_index_o[30:1], 1'd0};
                    end
                    7: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= work_four_one[15:0];
                    end
                    8: begin
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= work_four_thr[15:0];
                    end
                    16: begin
                        // Impedance testing: just the waveform (small)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {8'd0, dac_waveform_value};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    17: begin
                        // Impedance testing: just the waveform (large)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {dac_waveform_value, 8'd0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    18: begin
                        // Impedance testing: just the waveform (middle sized)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {4'd0, dac_waveform_value, 4'd0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    19: begin
                        // Impedance testing: just the waveform (centered)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {4'h8, dac_waveform_value, 4'd0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    20: begin
                        // Impedance testing: phase offset waveform (small)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {8'd0, dac_fazed_value};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    21: begin
                        // Impedance testing: phase offset waveform (large)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {dac_fazed_value, 8'd0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    22: begin
                        // Impedance testing: phase offset waveform (middle sized)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {4'd0, dac_fazed_value, 4'd0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    23: begin
                        // Impedance testing: phase offset waveform (centered)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {4'h8, dac_fazed_value, 4'h0};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    24: begin
                        // Impedance testing: phase offset waveform (smaller)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {10'd0, dac_fazed_value[5:0]};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    25: begin
                        // Impedance testing: phase offset waveform (smaller)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {12'd0, dac_fazed_value[3:0]};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    26: begin
                        // Impedance testing: phase offset waveform (smallest)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        if ((dac_select == spi_slot[4:0]) && dac_enable) begin
                            bram_in_data <= {14'd0, dac_fazed_value[1:0]};
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        end
                    end
                    32: begin // Extra slot 0; DAC waveform
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + (spi_slot - 11'd32);
                        bram_in_data <= {8'd0, dac_waveform_value};
                    end
                    33: begin // Extra slot 1
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + (spi_slot - 11'd32);
                        bram_in_data <= {3'd0, chip_index, 8'b1010_0001}; // 0x??A1
                    end
                    34: begin // Extra slot 2
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + (spi_slot - 11'd32);
                        // arbitrary CMD passback
                        if (dac_waveform_value == 5'd0 || dac_waveform_value == 5'd15) begin
                            if (((extra_cmd_chip_i[5] == 1'b1)
                                    || (extra_cmd_chip_i[4:0] == chip_index[4:0]))
                                  && extra_cmd_en_i) begin
                                bram_in_data <= extra_cmd_i;
                                if (extra_cmd_i[15:8] == {2'b10, 6'd5}) begin
                                    dac_enable <= extra_cmd_i[0];
                                    dac_power <= extra_cmd_i[6];
                                    dac_scale <= extra_cmd_i[4:3];
                                end else if (extra_cmd_i[15:8] == {2'b10, 6'd7}) begin
                                    dac_select <= extra_cmd_i[4:0];
                                end
                            end else begin
                                bram_in_data <= 16'd0;
                            end
                        end else begin
                            bram_in_data <= {3'd0, chip_index, 8'b1010_0010}; // 0x??A2
                        end
                    end
                    default: begin // regular channel slot (0 through 31)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                        bram_in_data <= {3'd0, chip_index, 2'd0, spi_slot};
                        /*
                        // Note: used in DEBUG
                        if (spi_slot > 34) begin
                            $finish("FAIL");
                        end
                        */
                    end
                    endcase
                end else begin
                    bram_in_we <= 1'b0;
                    state <= STATE_WAIT;
                end
            end
            STATE_LOOP_WAIT: begin
                // spi_slot gets incremented regardless of STATE
                if (enable_acq) begin
                    bram_in_we <= 1'b0;
                    if (spi_slot != last_slot) begin
                        // new data must be ready
                        state <= STATE_LOOP_READ;
                        chip_index <= 5'd0;
                    end
                end else begin
                    bram_in_we <= 1'b0;
                    state <= STATE_WAIT;
                end
            end
            default:
                state <= STATE_WAIT;
        endcase
        end
    end
endmodule

