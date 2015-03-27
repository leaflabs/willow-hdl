/*
 * cfg_master.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * This module processes "command socket" packets, exposing traditional
 * chip peripheral configuration registers over a simple network protocol. An
 * external microcontroller (MCU) handles the network-layer protocols (in this
 * case TCP over Ethernet), and passes raw data payloads over a 96bit SPI
 * interface to this module, which muxes out read/write requests to specific
 * system modules.
 *
 * This module acts as the SPI slave, with the MCU as master. The master
 * writes a packet (request) in a single SPI transaction, and then reads out
 * the reply in the next transaction. It is expected that the master will
 * normally write a dummy packet during "read" transactions, so that there are
 * two SPI transactions per request/reply packet, but it is possible for the
 * master to constantly send valid requests, resulting in a single SPI
 * transaction per request/reply (interleaved).
 *
 * The external modules may be in separate timing domains; the
 * "mread_en/mwrite_en" request valid lines and "sack" reply data valid lines
 * allow safe clock domain crossing. There is no explicit limit on how long
 * a module has to reply (indicated by "sack" going high), but it is expected
 * to be a handful of cfg_master clock cycles at most.
 *
 * Only a single mread or mwrite transaction with a single slave module will
 * happen at a time, so the cfg_addr and cfg_data_mwrite ports are shared by
 * all slave modules.
 *
 * See wired-leaf-docs:net_protocol/datanode-net-protocol-notes.txt for
 * details on the command protocol.
 *
 * Interface:
 *
 *      spi_sclk, spi_n_cs, spi_mosi, spi_miso
 *          Traditional SPI interface to MCU.
 *
 *      clock, reset
 *          The entire module is synchronous, including reset.
 *
 *      any_error
 *          Single combined 1 bit error status line for all modules. Could go
 *          high at any time.
 *
 *      cfg_sack
 *          Configuration slave acknowledge. Pulled high by a "slave" module
 *          when data is valid on cfg_data_mreadN (for either read or write
 *          requests)
 *
 *      cfg_mread_en
 *          Configuration master read enable. Pulled high by this module for
 *          a single slave module when cfg_addr is valid. Held high until the
 *          corresponding sack goes high.
 *
 *      cfg_mwrite_en
 *          Configuration master write enabled. Pulled high by this module for
 *          a single slave module when cfg_addr and cfg_data_mwrite is valid.
 *          Held high until the corresponding sack goes high. cfg_mwrite_en
 *          and cfg_mread_en are mutually exclusive.
 *
 *      cfg_data_mwrite
 *          32bit word to be written in an mwrite transaction.
 *
 *      cfg_data_mread
 *          Slave module's response to either an mread or mwrite transaction.
 *
 *      cfg_addr
 *          8-bit register address.
 *
 */

module cfg_master (
    input clock,
    input reset,

    // MCU SPI
    input wire spi_sclk,
    input wire spi_n_cs,
    input wire spi_mosi,
    output wire spi_miso,

    // Configuration Register Interfaces
    input wire any_error,
    input wire [5:0] cfg_sack,
    output reg [5:0] cfg_mread_en = 6'b0,
    output reg [5:0] cfg_mwrite_en = 6'b0,
    output reg [31:0] cfg_data_mwrite = 32'b0,
    input wire [31:0] cfg_data_mread0,
    input wire [31:0] cfg_data_mread1,
    input wire [31:0] cfg_data_mread2,
    input wire [31:0] cfg_data_mread3,
    input wire [31:0] cfg_data_mread4,
    input wire [31:0] cfg_data_mread5,
    output reg [7:0] cfg_addr = 8'b0

    );

    // why is this necessary?
    // ERROR:HDLCompiler:251 "Cannot access memory cfg_data_mread directly"
    wire [31:0] cfg_data_mread [0:5];
    assign cfg_data_mread[0] = cfg_data_mread0;
    assign cfg_data_mread[1] = cfg_data_mread1;
    assign cfg_data_mread[2] = cfg_data_mread2;
    assign cfg_data_mread[3] = cfg_data_mread3;
    assign cfg_data_mread[4] = cfg_data_mread4;
    assign cfg_data_mread[5] = cfg_data_mread5;

    // MCU SPI interface
    // NB: pins were debounced in main_*.v module
    wire[95:0] spi_data_i;
    wire spi_done;
    reg[95:0] spi_data_o = 96'd0;
    spi_slave spi_slave_inst (
        .clock(clock),
        .reset(reset),
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .n_cs(spi_n_cs),
        .miso(spi_miso),
        .data_i(spi_data_o),
        .data_o(spi_data_i),
        .done(spi_done)
        );

    // packet aliases
    wire [7:0] packet_magic;
    assign packet_magic = spi_data_i[95:88];
    wire [7:0] packet_version;
    assign packet_version = spi_data_i[87:80];
    wire [7:0] packet_type;
    assign packet_type = spi_data_i[79:72];
    wire [7:0] packet_flags;
    assign packet_flags = spi_data_i[71:64];
    wire request_dir;
    assign request_dir = packet_flags[0];
    wire [15:0] request_id;
    assign request_id = spi_data_i[63:48];
    wire [7:0] request_module;
    assign request_module = spi_data_i[47:40];
    wire [7:0] request_addr;
    assign request_addr = spi_data_i[39:32];
    wire [31:0] request_data;
    assign request_data = spi_data_i[31:0];

    // don't need all 8bits to index actual modules
    wire [2:0] module_index;
    assign module_index = request_module[2:0];

    parameter ERROR_PACKET = {8'h5A, 8'd0, 8'h7F, 8'h80, 64'd0};

    reg [7:0] state = STATE_WAIT;
    parameter STATE_WAIT = 0;
    parameter STATE_REPLY = 1;
    always @(posedge clock) begin
        if (reset) begin
            state <= STATE_WAIT;
            spi_data_o <= 96'd0;
            cfg_data_mwrite <= 32'd0;
        end else begin
            case (state)
            STATE_WAIT: begin
                if (spi_done) begin
                    // start fetching packet
                    if (packet_magic == 8'h5A &&     // magic number
                        packet_type == 8'h01 &&      // REQUEST
                        request_module <= 8'd5) begin // valid module
                        state <= STATE_REPLY;
                        cfg_addr <= request_addr;
                        if (request_dir == 1'b1) begin // WRITE
                            cfg_data_mwrite <= request_data;
                            cfg_mwrite_en[module_index] <= 1'b1;
                        end else begin // READ
                            cfg_mread_en[module_index] <= 1'b1;
                        end
                    end else begin
                        spi_data_o <= ERROR_PACKET;
                    end
                end
            end
            STATE_REPLY: begin
                if (cfg_sack[module_index]) begin
                    cfg_mread_en[module_index] <= 1'b0;
                    cfg_mwrite_en[module_index] <= 1'b0;
                    state <= STATE_WAIT;
                    spi_data_o <= {packet_magic, packet_version, 8'h02,
                                        any_error, packet_flags[6:0],
                                   request_id, request_module, request_addr,
                                   cfg_data_mread[module_index]};
                end
            end
            endcase
        end
    end

endmodule
