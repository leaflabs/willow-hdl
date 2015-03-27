/*
 * debounce.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * Calling this a "debouncer" is maybe a bit of an overstatement. It's a 2-bit
 * depth shift register for de-glitching 3.3v digital signaling, introduced
 * after SPI data integrity problems at low speeds (~250KHz SCLK) interfacing
 * to the LeafLabs Maple Mini with both the SP605 and WiredLeaf hardware.
 *
 * Introduces several clock cycles of delay betwen 'i' and 'o'. YMMV.
 */

module debounce (
    input clock,
    input reset,
    input i,        // raw input line
    output reg o    // de-glitched output line
    );

    reg [1:0] ibuf = 2'b0;

    always @(posedge clock) begin
        if (reset) begin
            ibuf <= 2'b00;
        end else begin
            if (i == ibuf[0]) begin
                ibuf[1] <= ibuf[0];
                if (ibuf[0] == ibuf[1]) o <= ibuf[1];
            end else begin
                ibuf[0] <= i;
            end
        end
    end

endmodule
