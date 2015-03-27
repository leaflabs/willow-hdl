/*
 * rhd2000_spi.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: The MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: April 2013
 *
 * This simple module implements SPI communications with an Intan
 * RHD2000-series chip (specificly the 2132, though the 2216 would not require
 * many modifications). It is simply a 16-bit shift register at a ~22MHz SCLK
 * frequency.
 *
 * See page 14 of the RHD2000-series datasheet for a detailed timing diagram.
 *
 * Build-time Parameters (defined in Makefile):
 *
 *      MOSI_ON_FALLING and MOSI_ON_NEXT_RISING:
 *          These are crude offsets used to calibrate for (relatively) long
 *          cable and buffering delays between the FPGA and the Intan chips.
 *          MOSI_ON_NEXT_RISING overrides MOSI_ON_FALLING.
 *
 */
module rhd2000_spi (
    output wire mosi_o,
    output reg sclk_o = 1'b0,
    output reg csbar_o = 1'b1,
    output reg[15:0] reply_o = 16'b0,
    input wire[15:0] cmd_i,
    input wire miso_i,
    input wire clk_i,            // expect a 44.1MHz clock -> 22.05MHz SCLK
    input wire reset_i,
    input wire sync_i
    );

    reg[15:0] ioreg = 0;
    reg[3:0] shift_ind = 0;
    reg[2:0] state = STATE_IDLE;
    reg miso_buf = 0;

    // Set to 1 to (partially) calibrate for cable delays, etc.
    parameter MOSI_ON_FALLING = 1;
    // Set to 1 to (more strongly) calibrate for cable delays, etc. Overrides
    // MOSI_ON_FALLING.
    parameter MOSI_ON_NEXT_RISING = 1;

    // see Intan datasheet and doc/rhd2000_timing.ods for SPI timing details
    localparam STATE_IDLE = 0;
    localparam STATE_CS1 = 1;
    localparam STATE_SHIFT_WAIT = 2;
    localparam STATE_SHIFT_DATA = 3;
    localparam STATE_CS2 = 4;

    assign mosi_o = ioreg[15];

    always @(posedge clk_i) begin
        if (reset_i == 1'b1) begin
            csbar_o <= 1'b1;
            state <= STATE_IDLE;
            sclk_o <= 1'b0;
            ioreg[15:0] <= 16'd0;
            shift_ind <= 4'd0;
            miso_buf <= 1'b0;
            reply_o <= 16'd0;
        end else begin
            case (state)
            STATE_IDLE: begin
                shift_ind <= 4'd0;
                if (sync_i) begin
                    // start sampling!
                    csbar_o <= 1'b0;
                    state <= STATE_CS1;
                    sclk_o <= 1'b0;
                    ioreg[15:0] <= cmd_i[15:0];
                end else begin
                    state <= STATE_IDLE;
                end
            end
            STATE_CS1: begin
                // this is just waiting for the next SCLK sync
                state <= STATE_SHIFT_WAIT;
            end
            STATE_SHIFT_WAIT: begin
                sclk_o <= 1;
                miso_buf <= miso_i;
                state <= STATE_SHIFT_DATA;
                if (MOSI_ON_NEXT_RISING && (shift_ind != 0)) begin
                    ioreg[0] <= miso_i;
                end
            end
            STATE_SHIFT_DATA: begin
                sclk_o <= 1'b0;
                shift_ind <= shift_ind + 4'd1;
                if (MOSI_ON_FALLING) begin
                    ioreg <= {ioreg[14:0], miso_i};
                end else begin
                    ioreg <= {ioreg[14:0], miso_buf};
                end
                if (shift_ind == 15)
                    state <= STATE_CS2;
                else begin
                    state <= STATE_SHIFT_WAIT;
                end
            end
            STATE_CS2: begin
                csbar_o <= 1;
                state <= STATE_IDLE;
                if (MOSI_ON_NEXT_RISING) begin
                    reply_o <= {ioreg[15:1], miso_i};
                end else begin
                    reply_o <= ioreg;
                end
                sclk_o <= 1'b0;
            end
            default:
                state <= STATE_IDLE;
            endcase
        end
    end
endmodule

