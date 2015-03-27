/*
 * board_identifier.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: August 2013
 *
 * This is a simple core which wraps the built-in Xilinx "DNA" unique per-chip
 * identifier shift register as a wide value.
 *
 * The bottom 32 bits of the 57 bit identifier are returned.
 *
 * "DIN" is tied to "DOUT" of the shift register, so the ID is written back
 * over itself while reading (as recommended by the Xilinx Spartan-6
 * Configuration User Guide, UG380).
 *
 * This peripheral can only be run at 2MHz (?!?!), so a factor of ~64 slow down
 * compared to 44MHz is used (more than twice as slow as necessary).
 *
 * WARNING: this core may not be robust to rapid toggling (bounces) of the reset
 * line, which may result in the shift register being partially read.
 *
 * Interface:
 *      ready indicates whether the board_identifier is valid yet.
 *      board_identifier is the bottom 32 bits of the 57-bit identifier.
 */

module board_id_generator (
    input wire clock,
    input wire reset,

    output reg ready = 1'b0,
    output reg [31:0] board_identifier = 32'b0
);

    wire dout;
    reg read_en = 1'b0;
    reg shift_en = 1'b0;
    reg [56:0] shift_data = 57'b0;
    reg [5:0] shift_count = 6'b0;
    reg [5:0] slow_counter = 6'b0;

    DNA_PORT #(
        .SIM_DNA_VALUE(57'h12345678_9ABCDE)
    ) dna_port_inst (
        .DOUT(dout),
        .DIN(dout),
        .READ(read_en),
        .SHIFT(shift_en),
        .CLK(slow_counter[5])
    );

    always @(posedge clock) begin
        if (reset) begin
            ready <= 0;
            read_en <= 0;
            shift_en <= 0;
            shift_data <= 0;
            shift_count <= 0;
            board_identifier <= 0;
            slow_counter <= 0;
        end else begin
            slow_counter <= slow_counter + 6'd1;
            if (slow_counter == 0) begin
                if (!(read_en || shift_en || ready)) begin
                    read_en <= 1;
                    ready <= 0;
                end else begin
                    if (shift_count == 6'd0) begin
                        read_en <= 0;
                        shift_en <= 1;
                        shift_count <= shift_count + 6'd1;
                    end else if (shift_count <= 6'd56) begin
                        shift_count <= shift_count + 6'd1;
                        shift_data <= {shift_data[55:0], dout};
                    end else begin
                        read_en <= 0;
                        shift_en <= 0;
                        ready <= 1;
                        board_identifier <= shift_data[31:0];
                    end
                end
            end
        end
    end

endmodule
