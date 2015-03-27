/*
 * sine_table.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: The MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: April 2013
 *
 * A simple static sine wave lookup table.
 *
 * 30 values at 8 bits, centered at half max.
 */
module sine_table (
    input wire clk_i,           // 44.1 MHz clock
    input wire[4:0] addr_i,
    output wire[7:0] value_o
    );

    reg[7:0] outval = 8'b0;
    assign value_o = outval;

    wire[7:0] lookup_table [0:29];
    // see bottom for python code to generate this list
    assign lookup_table[0] = 8'd128;
    assign lookup_table[1] = 8'd154;
    assign lookup_table[2] = 8'd180;
    assign lookup_table[3] = 8'd203;
    assign lookup_table[4] = 8'd223;
    assign lookup_table[5] = 8'd238;
    assign lookup_table[6] = 8'd249;
    assign lookup_table[7] = 8'd255;
    assign lookup_table[8] = 8'd255;
    assign lookup_table[9] = 8'd249;
    assign lookup_table[10] = 8'd238;
    assign lookup_table[11] = 8'd223;
    assign lookup_table[12] = 8'd203;
    assign lookup_table[13] = 8'd180;
    assign lookup_table[14] = 8'd154;
    assign lookup_table[15] = 8'd128;
    assign lookup_table[16] = 8'd101;
    assign lookup_table[17] = 8'd75;
    assign lookup_table[18] = 8'd52;
    assign lookup_table[19] = 8'd32;
    assign lookup_table[20] = 8'd17;
    assign lookup_table[21] = 8'd6;
    assign lookup_table[22] = 8'd0;
    assign lookup_table[23] = 8'd0;
    assign lookup_table[24] = 8'd6;
    assign lookup_table[25] = 8'd17;
    assign lookup_table[26] = 8'd32;
    assign lookup_table[27] = 8'd52;
    assign lookup_table[28] = 8'd75;
    assign lookup_table[29] = 8'd101;

    always @(posedge clk_i) begin
        if (addr_i < 30) begin
            outval <= lookup_table[addr_i];
        end
    end

endmodule

/* Python code to generate sine table

import math
for i in range(30):
    v = 2**7 + 2**7 * math.sin(math.pi / 15. * i)
    print "assign lookup_table[%d] = 8'd%d;" % (i, v)

*/
