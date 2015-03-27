/*
 * udp_core.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: May-June 2013
 *
 * Changes to the tables below should be included in the
 * "wiredleaf-registers.txt" file in the documentation repository.
 *
 * Register Map (module 0x04):
 *
 *    0x00    R/W     UDP Module Error Flags
 *    0x01    R/W     Unused (3 B) | UDP Module Enable (1 B)
 *    0x02    R/O     Unused (2 B) | Source MAC-48 Address Top bytes (2 B)
 *    0x03    R/O     Source MAC-48 Address Bottom bytes (4 B)
 *    0x04    R/W     Unused (2 B) | Destination MAC-48 Address Top bytes (2B)
 *    0x05    R/W     Destination MAC-48 Address Bottom bytes (4 B)
 *    0x06    R/W     Source IPv4 Address (4 B)
 *    0x07    R/W     Destination IPv4 Address (4 B)
 *    0x08    R/W     Unused (2 B) | Source IPv4 Port (2 B)
 *    0x09    R/W     Unused (2 B) | Destination IPv4 Port (2 B)
 *    0x0A    R/O     Unused (2 B) | Packet Count
 *    0x0B    R/O     Unused (2 B) | Packet Length (2 B)
 *    0x0C    R/O     Unused (2 B) | Payload Length (2 B)
 *    0x0D    R/W     Unused (3 B) | UDP Module Mode (0=daq, 1=sata) (1 B)
 *    0x0E    R/O     Unused (3 B) | GigE Status (1 B)
 *    0x0F    R/W     Unused (3 B) | Unused (3 B) | GigE PHY MIIM Enable (1 bit)
 *    0x10    R/W     Unused (3 B) | Unused (3 B) | GigE PHY MIIM Address (5 bits)
 *    0x11    R/O     Unused (3 B) | Unused (1 B) | GigE PHY MIIM Data (17 bit)
 *
 * Error Flags:
 *      0   Configuration Error
 *      7   Other error
 */

module udp_core (
    input clock_44mhz,
    input clock_125mhz,
    input reset,

    // DAQ interface
    input [15:0] daq_fifo_payload_len,
    input [12:0] daq_fifo_count,
    input [7:0] daq_fifo_data,
    output daq_fifo_clock,
    output daq_fifo_read_en,

    // GIGE Hardware I/O
    output wire gige_reset,
    output wire gige_txc_gtxclk,
    output wire gige_mdc,
    inout wire gige_mdio,
    output wire gige_txctl_txen,
    output wire[7:0] gige_tx_data,

    // SATA interface
    input [15:0] sata_fifo_payload_len,
    input [12:0] sata_fifo_count,
    input [7:0] sata_fifo_data,
    output sata_fifo_clock,
    output sata_fifo_read_en,

    // generic register configuration interface
    input [31:0] cfg_data_mwrite,
    output reg [31:0] cfg_data_mread = 32'b0,
    input [7:0] cfg_addr,
    input cfg_mread_en,
    input cfg_mwrite_en,
    output reg cfg_sack = 0,
    output wire error_flag
    );

    // default configuration parameters. these could be overridden by
    // a higher-level defparam.
    parameter UDP_DEFAULT_DEST_MAC48 = 48'h3C97_0E76_7019; // bnewbold's Thinkpad
    parameter UDP_DEFAULT_SRC_IPV4 = {8'd192, 8'd168, 8'd1, 8'd20};
    parameter UDP_DEFAULT_DEST_IPV4 = {8'd192, 8'd168, 8'd1, 8'd99};
    parameter UDP_DEFAULT_SRC_PORT = 16'd1370;
    parameter UDP_DEFAULT_DEST_PORT = 16'd1370;

    reg udp_mode = MODE_DAQ;
    localparam MODE_DAQ = 0;
    localparam MODE_SATA = 1;

    reg udp_enable = 0;

    // error flags
    reg error_cfg = 0;
    reg error_other = 0;
    assign error_flag = error_cfg || error_other;

    reg [47:0] destination_mac48 = UDP_DEFAULT_DEST_MAC48;
    reg [31:0] source_ipv4 = UDP_DEFAULT_SRC_IPV4;
    reg [31:0] destination_ipv4 = UDP_DEFAULT_DEST_IPV4;
    reg [15:0] source_port = UDP_DEFAULT_SRC_PORT;
    reg [15:0] destination_port = UDP_DEFAULT_DEST_PORT;

    wire [47:0] source_mac48;
    wire [31:0] packet_count;
    wire [16:0] packet_length;
    wire [3:0] gige_state;

    wire fifo_clock;
    reg [15:0] fifo_payload_len = 0;

    reg miim_probe_en = 0;
    reg [4:0] miim_addr = 5'd1;
    wire [16:0] miim_data;

    //// Instantiate actual UDP core
    udp udp_i0 (
        .clock(clock_44mhz),
        .clk125(clock_125mhz),
        .RESET(reset),

        .udp_en(udp_enable),
        .miim_probe_en(miim_probe_en),

        .miim_data(miim_data),
        .miim_addr(miim_addr),

        .tx_outen(gige_txctl_txen),
        .tx_out(gige_tx_data),
        .phy_reset(gige_reset),
        .phy_mdc(gige_mdc),
        .PHY_MDIO(gige_mdio),

        .fifo_clock(fifo_clock),
        .payload_length_in(fifo_payload_len),
        .fifo_select(udp_mode),

        .fifo0_data_count(daq_fifo_count),
        .fifo0_data(daq_fifo_data),
        .fifo0_read_en(daq_fifo_read_en),
        .fifo1_data_count(sata_fifo_count),
        .fifo1_data(sata_fifo_data),
        .fifo1_read_en(sata_fifo_read_en),

        .packet_count(packet_count),
        .gige_state(gige_state),
        .udp_packet_length(packet_length),
        .udp_source_mac(source_mac48),

        .mac_destination_in(destination_mac48),
        .ip_source_in(source_ipv4),
        .ip_destination_in(destination_ipv4),
        .port_source_in(source_port),
        .port_destination_in(destination_port),

        .data_received(),
        .led_clock_throb()
        );

    assign daq_fifo_clock = fifo_clock;
    assign sata_fifo_clock = fifo_clock;

    always @(posedge clock_44mhz) begin
        if (reset) begin
            error_cfg <= 1'b0;
            error_other <= 1'b0;
            destination_mac48 <= UDP_DEFAULT_DEST_MAC48;
            source_ipv4 <= UDP_DEFAULT_SRC_IPV4;
            destination_ipv4 <= UDP_DEFAULT_DEST_IPV4;
            source_port <= UDP_DEFAULT_SRC_PORT;
            destination_port <= UDP_DEFAULT_DEST_PORT;
            udp_enable <= 1'b0;
            udp_mode <= MODE_DAQ;
            fifo_payload_len <= 16'd0;
            miim_probe_en <= 0;
            miim_addr <= 5'd1;
        end else begin
            /* ----------- Main Logic ------------ */
            if (udp_mode == MODE_DAQ) begin
                fifo_payload_len <= daq_fifo_payload_len;
            end else begin
                fifo_payload_len <= sata_fifo_payload_len;
            end
            /* ----------- Configuration Interface ----------- */
            if (cfg_mread_en || cfg_mwrite_en) begin
                cfg_sack <= 1'b1;
                case (cfg_addr)
                8'h00: begin  // UDP Module Error Flags
                    if (cfg_mwrite_en) begin
                        error_cfg <= cfg_data_mwrite[0];
                        error_other <= cfg_data_mwrite[7];
                        cfg_data_mread <= {24'd0, cfg_data_mwrite[7:0]};
                    end else begin
                        cfg_data_mread <= {24'd0,
                            error_other,        // 7
                            6'd0,               // ...
                            error_cfg};         // 0
                    end
                end
                8'h01: begin  // Unused (3 B) | UDP Module Enable (1 B)
                    if (cfg_mwrite_en) begin
                        udp_enable <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, udp_enable};
                    end
                end
                8'h02: begin  // Unused (2 B) | Source MAC-48 Address Top bytes (2 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, source_mac48[47:32]};
                end
                8'h03: begin  // Source MAC-48 Address Bottom bytes (4 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= source_mac48[31:0];
                end
                8'h04: begin // Unused (2 B) | Destination MAC-48 Address Top bytes (2 B)
                    if (cfg_mwrite_en) begin
                        destination_mac48[47:32] <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= {16'd0, cfg_data_mwrite[15:0]};
                    end else begin
                        cfg_data_mread <= {16'd0, destination_mac48[47:32]};
                    end
                end
                8'h05: begin // Destination MAC-48 Address Bottom bytes (4 B)
                    if (cfg_mwrite_en) begin
                        destination_mac48[31:0] <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= destination_mac48[31:0];
                    end
                end
                8'h06: begin // Source IPv4 Address (4 B)
                    if (cfg_mwrite_en) begin
                        source_ipv4 <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= source_ipv4;
                    end
                end
                8'h07: begin // Destination IPv4 Address (4 B)
                    if (cfg_mwrite_en) begin
                        destination_ipv4 <= cfg_data_mwrite;
                        cfg_data_mread <= cfg_data_mwrite;
                    end else begin
                        cfg_data_mread <= destination_ipv4;
                    end
                end
                8'h08: begin // Unused (2 B) | Source IPv4 Port (2 B)
                    if (cfg_mwrite_en) begin
                        source_port[15:0] <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= {16'd0, cfg_data_mwrite[15:0]};
                    end else begin
                        cfg_data_mread <= {16'd0, source_port};
                    end
                end
                8'h09: begin // Unused (2 B) | Destination IPv4 Port (2 B)
                    if (cfg_mwrite_en) begin
                        destination_port[15:0] <= cfg_data_mwrite[15:0];
                        cfg_data_mread <= {16'd0, cfg_data_mwrite[15:0]};
                    end else begin
                        cfg_data_mread <= {16'd0, destination_port};
                    end
                end
                8'h0A: begin // Packet Count (4 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= packet_count;
                end
                8'h0B: begin // Unused (1 B) | Packet Length (3 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, packet_length[15:0]};
                end
                8'h0C: begin // Unused (1 B) | Payload Length (3 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {16'd0, fifo_payload_len};
                end
                8'h0D: begin  // Unused (3 B) | UDP Module Mode (1 B)
                    if (cfg_mwrite_en) begin
                        udp_mode <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, udp_mode};
                    end
                end
                8'h0E: begin  // Unused (3 B) | GigE Status (1 B)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {28'd0, gige_state};
                end
                8'h0F: begin  // Unused (3 B) | GigE PHY MIIM Enable (1 bit)
                    if (cfg_mwrite_en) begin
                        miim_probe_en <= cfg_data_mwrite[0];
                        cfg_data_mread <= {31'd0, cfg_data_mwrite[0]};
                    end else begin
                        cfg_data_mread <= {31'd0, miim_probe_en};
                    end
                end
                8'h10: begin  // Unused (3 B) | GigE PHY MIIM Address (5 bits)
                    if (cfg_mwrite_en) begin
                        miim_addr <= cfg_data_mwrite[4:0];
                        cfg_data_mread <= {27'd0, cfg_data_mwrite[4:0]};
                    end else begin
                        cfg_data_mread <= {27'd0, miim_addr};
                    end
                end
                8'h11: begin  // Unused (1 B) | GigE PHY MIIM Data (17 bit)
                    if (cfg_mwrite_en) begin
                        error_cfg <= 1'b1;
                    end
                    cfg_data_mread <= {15'd0, miim_data};
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

