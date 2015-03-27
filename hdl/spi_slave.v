/*
 * spi_slave.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * This is a simple non-parametric SPI Slave implementation. Does what it says
 * on the tin.
 *
 * This module is synchronous with 'clock', as opposed to triggering off SCLK.
 *
 * Interface:
 *
 *      clock, reset
 *          This module is synchronous off 'clock' (including reset), as
 *          opposed to triggering off SCLK. clock should probably be at least
 *          ~2-4x faster than the expected SCLK rate.
 *
 *      mosi, miso, sclk, n_cs
 *          Regular old SPI interface lines. Should probably be debounced at
 *          a device-specific layer.
 *
 *      done
 *          Goes high for a single clock cycle at the end of a SPI
 *          transaction. technically, the SPI transaction isn't "done" because
 *          n_cs hasn't gone high yet, but all data has been read after the
 *          last rising edge of SCLK.
 *
 *      data_i
 *          The 96-bit value to be shifted out (on MISO, from this module to
 *          the master) on the next falling edge of n_cs. This should get set
 *          up rapidly after a 'done' pulse.
 *
 *      data_o
 *          The 96-bit value most recently read. Valid when 'done' is pulsed,
 *          and will persist until the next 'done' pulse.
 *
 */

module spi_slave (
    input clock,
    input reset,
    // spi lines
    input mosi,
    output reg miso = 0,
    input sclk,
    input n_cs,
    // data i/o lines
    output reg done = 0,
    input [95:0] data_i,
    output reg [95:0] data_o = 0
    );

    reg mosi_buf = 0;
    reg [7:0] bit_count = 0;
    reg [95:0] data = 0;

    parameter STATE_LOW = 0;
    parameter STATE_HIGH = 1;
    parameter STATE_DONE = 2;
    reg [1:0] state = STATE_LOW;

    always @(posedge clock) begin
        if (reset) begin
            data_o <= 96'd0;
            data <= 96'd0;
            mosi_buf <= 1'b0;
            miso <= 1'b0;
            state <= STATE_LOW;
            bit_count <= 8'd0;
            done <= 1'b0;
        end else if (n_cs) begin
            mosi_buf <= 1'b0;
            miso <= 1'b0;
            state <= STATE_LOW;
            data <= data_i;
            bit_count <= 8'd0;
            done <= 1'b0;
        end else begin
            case (state)
            STATE_LOW: begin
                miso <= data[95];
                if (sclk) begin
                    // posedge sclk
                    mosi_buf <= mosi;
                    state <= STATE_HIGH;
                    bit_count <= bit_count + 8'd1;
                end
            end
            STATE_HIGH: begin
                if (~sclk) begin
                    // negedge sclk
                    if (bit_count == 96) begin
                        // all done
                        done <= 1'b1;
                        state <= STATE_DONE;
                        data_o <= {data[94:0], mosi_buf};
                    end else begin
                        data <= {data[94:0], mosi_buf};
                        state <= STATE_LOW;
                    end
                end
            end
            STATE_DONE: begin
                done <= 1'b0;
            end
            endcase

        end
    end

endmodule
