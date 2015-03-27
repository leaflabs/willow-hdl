/*
 * gpio_core.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * This is a simple core to expose configurable GPIO pins over a standard
 * register interface (which is subsequently exposed over a network protocol).
 *
 * 16 GPIO pins is assumed, but the configuration of which are "readable"
 * (input) vs. "writable" (output) is configurable at compile time using the
 * GPIO_READ_MASK and GPIO_WRITE_MASK parameters.
 *
 * The "Error" register of this module is writable, which makes it possible to
 * experiment with higher-level error handling.
 *
 * Interface:
 *
 *      clock, reset
 *          This module is synchronous, including reset.
 *
 *      gpio_pins
 *          Routed to the physical I/O pads; I/O direction should be
 *          consistent with GPIO_READ_MASK and GPIO_WRITE_MASK.
 *
 *      cfg_data_mwrite, cfg_data_mread, cfg_addr, cfg_mread_en,
 *      cfg_mwrite_en, cfg_sack, error_flag
 *          Generic register configuration interface. See ./cfg_master.v for
 *          details.
 *
 * Changes to the tables below should be included in the
 * "wiredleaf-registers.txt" file in the documentation repository.
 *
 * Register map (module 0x05):
 *
 *      0x00     R/W     GPIO Module Error Flags
 *      0x01      -      (no state machine for GPIO pins)
 *      0x02     R/O     GPIO read mask
 *      0x03     R/O     GPIO write mask
 *      0x04     R/W     GPIO status
 *
 * Error Flags:
 *      0   Configuration Error
 */

module gpio_core (
    input clock,
    input reset,

    inout[15:0] gpio_pins, // run out to the acutual pads

    // generic register configuration interface
    input [31:0] cfg_data_mwrite,
    output reg [31:0] cfg_data_mread = 0,
    input [7:0] cfg_addr,
    input cfg_mread_en,
    input cfg_mwrite_en,
    output reg cfg_sack = 0,
    output wire error_flag
    );

    // configurable parameters
    parameter GPIO_READ_MASK = 16'h00FF;
    parameter GPIO_WRITE_MASK = 16'hFF00;

    // error flags
    reg error_cfg = 0;
    assign error_flag = error_cfg;

    wire [15:0] gpio_read_mask;
    wire [15:0] gpio_write_mask;
    assign gpio_read_mask = GPIO_READ_MASK;
    assign gpio_write_mask = GPIO_WRITE_MASK;

    if ((GPIO_READ_MASK & GPIO_WRITE_MASK) != 16'h0000) begin
        //$display("GPIO_READ_MASK and GPIO_WRITE_MASK overlap");
        //$assert(0'd0);
        //$finish();
    end

    wire [15:0] gpio_readable;
    reg [15:0] gpio_writeable = 16'b0;
    genvar PINNUM;
    for (PINNUM = 0; PINNUM < 16; PINNUM = PINNUM + 1) begin : pin_block
        if (GPIO_WRITE_MASK[PINNUM] == 1'b1) begin
            assign gpio_pins[PINNUM] = gpio_writeable[PINNUM];
            assign gpio_readable[PINNUM] = 1'b0;
        end else if (GPIO_READ_MASK[PINNUM] == 1'b1) begin
            // safe to ignore gpio_writeable[PINNUM] warnings
            assign gpio_readable[PINNUM] = gpio_pins[PINNUM];
        end else begin
            // safe to ignore gpio_writeable[PINNUM] warnings
            assign gpio_pins[PINNUM] = 1'bZ;
            assign gpio_readable[PINNUM] = 1'b0;
        end
    end

    always @(posedge clock) begin
        if (reset) begin
            error_cfg <= 1'b0;
            gpio_writeable <= 16'd0;
            cfg_data_mread <= 32'd0;
            cfg_sack <= 1'b0;
        end else begin
            if (cfg_mread_en || cfg_mwrite_en) begin
                cfg_sack <= 1'b1;
                case (cfg_addr)
                8'h00: begin  // GPIO Module Error Flags
                    if (cfg_mwrite_en) begin
                        error_cfg <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, error_cfg};
                    end
                end
                8'h01: begin  // (no state machine for GPIO pins)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= 32'd0;
                end
                8'h02: begin  // Unused (2 B) | Available GPIO mask (2 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, gpio_read_mask};
                end
                8'h03: begin  // Unused (2 B) | GPIO state (2 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, gpio_write_mask};
                end
                8'h04: begin
                    if (cfg_mwrite_en) begin
                        gpio_writeable <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= {16'd0, cfg_data_mwrite[15:0] & GPIO_WRITE_MASK};
                    end else begin
                        // read
                        cfg_data_mread <= {16'd0, gpio_pins};
                    end
                end
                default: begin
                    // configuration address error
                    error_cfg <= 1'b1;
                    cfg_data_mread <= 32'd0;
                end
                endcase
            end else begin
                cfg_sack <= 1'b0;
            end
        end
    end

endmodule

