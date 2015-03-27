// Copyright (C) 2013 LeafLabs
// Distributed under the MIT licence.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

module udp (
            input              clock,
            input              clk125,
            input              RESET,

            input              miim_probe_en,
            input              udp_en,

            input [4:0]        miim_addr,
            output [16:0]      miim_data,

            input [15:0]       payload_length_in,
            input [47:0]       mac_destination_in,
            input [31:0]       ip_source_in,
            input [31:0]       ip_destination_in,
            input [15:0]       port_source_in,
            input [15:0]       port_destination_in,

            output reg         tx_outen = 0,
            output reg [7:0]   tx_out = 0,
            output reg         phy_reset = 0,
            output             phy_mdc,
            inout wire         PHY_MDIO,

            output wire        fifo_clock,
            input wire         fifo_select,
            input [12:0]       fifo0_data_count,
            output reg         fifo0_read_en = 0,
            input [7:0]        fifo0_data,
            input [12:0]       fifo1_data_count,
            output reg         fifo1_read_en = 0,
            input [7:0]        fifo1_data,

            output reg [31:0]  packet_count = 32'b0,
            output wire [3:0]  gige_state,

            output wire [16:0] udp_packet_length,
            output wire [47:0] udp_source_mac,

            output reg         data_received = 0,
            output wire        led_clock_throb
            );

   // Buffered (cross-clock-domain) inputs
   reg                         udp_en_buf = 0;
   reg                         miim_probe_en_buf = 0;

   // MIIM signals

   // MIIM states
   parameter [3:0]
     MIIM_IDLE = 0,
     MIIM_SHIFT_OUT_0 = 1,
     MIIM_SHIFT_OUT_1 = 2,
     MIIM_SHIFT_OUT_2 = 3,
     MIIM_SHIFT_IN_0 = 4,
     MIIM_SHIFT_IN_1 = 5,
     MIIM_UART_OUT_0 = 6,
     MIIM_UART_OUT_1 = 7,
     MIIM_WAIT = 8;

   parameter [9:0] MDC_WAIT = 270;
   parameter [13:0] DEFAULT_MIIM_ADDR = 14'b01100011100001; // read from reg1
   // Low 5 bits of MIIM_ADDR is reg to be read; don't touch high 9
   // bits

   reg [13:0]                  mdio_data_o = DEFAULT_MIIM_ADDR;
   reg [16:0]                  mdio_data_i = 17'b0;
   reg                         mdc = 1'b1;
   reg                         mdio = 1'b1;
   reg [3:0]                   miim_state = MIIM_IDLE;
   reg [9:0]                   mdc_count = 0;
   reg [4:0]                   mdc_initialize = 0;
   reg [4:0]                   mdio_count = 0;
   assign miim_data = mdio_data_i;

   // CRC32 signals

   wire [31:0]                 crc_reg;
   wire [7:0]                  crc_out;
   wire [7:0]                  crc_byte;
   reg [31:0]                  crc_reg_shift = 0;
   reg [7:0]                   crc_d = 0;
   reg                         crc_calc = 0;
   reg                         crc_d_valid = 0;
   reg                         crc_clear = 0;

   // Transmit signals

   // GigE Transmit states
   parameter [3:0]
     TX_IDLE = 0,
     TX_SHIFT_OUT_0 = 1,
     TX_SHIFT_OUT_1 = 2,
     TX_CRC_OUT_0 = 3,
     TX_WAIT = 4,
     TX_PACKET_CONSTRUCT_0 = 5,
     TX_PACKET_CONSTRUCT_1 = 6,
     TX_PACKET_CONSTRUCT_2 = 7,
     TX_PACKET_CONSTRUCT_3 = 8,
     TX_PACKET_CONSTRUCT_4 = 9,
     TX_PACKET_CONSTRUCT_5 = 10,
     TX_PACKET_CONSTRUCT_6 = 11,
     TX_PACKET_CONSTRUCT_7 = 12,
     TX_PACKET_CONSTRUCT_8 = 13;

   reg [3:0]                   tx_state = TX_IDLE;
   assign gige_state = tx_state;
   reg [16:0]                  tx_count = 0;
   reg [7:0]                   tx_delay_count = 0;
   (* keep = "true" *) reg [23:0]                tx_led = 0;

   assign fifo_clock = clk125;


   parameter [5:0] header_length = 50; // in bytes
   parameter [63:0] ethernet_preamble = 64'h55555555555555D5;
   parameter [47:0] mac_source = 48'h001234567890;
   parameter [15:0] ethernet_type = 16'h0800;
   parameter [15:0] udp_checksum = 16'h0000;
   //for delivery

    // default configuration parameters. these could be overridden by
    // a higher-level defparam.
    parameter UDP_DEFAULT_DEST_MAC48 = 48'h3C97_0E76_7019; // bnewbold's Thinkpad
    parameter UDP_DEFAULT_SRC_IPV4 = {8'd192, 8'd168, 8'd1, 8'd20};
    parameter UDP_DEFAULT_DEST_IPV4 = {8'd192, 8'd168, 8'd1, 8'd99};
    parameter UDP_DEFAULT_SRC_PORT = 16'd1370;
    parameter UDP_DEFAULT_DEST_PORT = 16'd1370;

   // initialized on every write
   reg [15:0]                  payload_length = 16'b0;
   reg [47:0]                  mac_destination = UDP_DEFAULT_DEST_MAC48;
   reg [31:0]                  ip_source = UDP_DEFAULT_SRC_IPV4;
   reg [31:0]                  ip_destination = UDP_DEFAULT_DEST_IPV4;
   reg [15:0]                  port_source = UDP_DEFAULT_SRC_PORT;
   reg [15:0]                  port_destination = UDP_DEFAULT_DEST_PORT;

   reg [16:0]                  packet_length = 0;
   reg [79:0]                  ip_header = 0;
   reg [47:0]                  udp_header = 0;

   assign udp_packet_length = packet_length;
   assign udp_source_mac = mac_source;

   reg [7:0]                   payload_byte = 8'hff;
   reg [7:0]                   payload_byte_next = 8'hff;

   // calculate the IP checksum, big-endian style
   reg [31:0]                  ip_checksum_constant_0 = 0;
   reg [31:0]                  ip_checksum_constant_1 = 0;
   reg [31:0]                  ip_checksum_calc_0 = 0;
   reg [31:0]                  ip_checksum_calc_1 = 0;
   reg [31:0]                  ip_checksum_calc_2 = 0;
   reg [15:0]                  ip_checksum = 0;

   reg [((header_length*8)-1):0] packet_header = 0;

   crc crc_i (
              .clk (~clk125),
              .reset (RESET),
              .init (crc_clear),
              .d (crc_d),
              .calc (crc_calc),
              .d_valid (crc_d_valid),
              .crc_reg (crc_reg),
              .crc (crc_out)
              );

   // CRC32 logic

   // Each ethernet packet must have a 32-bit CRC at the end. This CRC
   // is computed by a module called "crc". The logic in this section
   // ensures that the appropriate data is passed into the CRC module
   // at the appropriate times. Packet data must be passed into the
   // CRC module one clock cycle before it goes out over the TX lines
   // to ensure the CRC has been fulled computed in time.

   always @ (negedge clk125) begin
      // The preamble bits (55555555555555D5) are not used to compute
      // the CRC, but the rest of the header is.
      if ((tx_count > 6) && (tx_count < (header_length-1))) begin
         crc_d <= packet_header[((header_length*8)-9):((header_length*8)-16)];
         crc_calc <= 1;
      end
      // Once the header has been passed in, pass in the payload data.
      else if ((tx_count >= (header_length-1)) && (tx_count < (packet_length-1))) begin
         crc_d <= payload_byte_next;
         crc_calc <= 1;
      end
      else begin
         crc_d <= 0;
         crc_calc <= 0;
      end
      if ((tx_count > 6) && (tx_count < (packet_length+2))) begin
         crc_d_valid <= 1;
      end
      else begin
         crc_d_valid <= 0;
      end
   end

   // MV88E1111 Reset Logic

   // On startup, the MV88E1111 phy chip reset line must be pulled
   // high for 5 ms, then low for 5 ms, before the chip is
   // functional. This logic takes care of toggling the reset line
   // appropriately.

   reg [20:0]                    reset_counter = 0;
   reg                           reset_done = 0;

   always @ (posedge clk125) begin
      if (RESET == 1) begin
         reset_counter <= 0;
         phy_reset <= 0;
         reset_done <= 0;
      end
      else begin
         if (reset_done == 0) reset_counter <= reset_counter+21'd1;
         if (reset_counter < 21'd625_000) phy_reset <= 1;
         else phy_reset <= 0;
         if (reset_counter == 21'd1_250_000) begin
            reset_done <= 1;
            reset_counter <= 0;
         end
      end
   end

   // Transmit Logic
   reg [12:0] fifo_data_count = 0;

   // On an external trigger (switch 1 being flipped), this state
   // machine begins outputting packets to the MV88E1111 phy chip,
   // which then puts them on the wire. Packets are continually output
   // until the switch is turned off. Each packet is comprised of 50
   // bytes of header data, 2500 payload bytes, and 4 CRC
   // bytes. Currently, this state machine just outputs a packet
   // counter into each payload byte, but the code should be
   // structured to make it trivial to output from a BRAM or other
   // memory element instead. More instructions for doing so inline.

   always @ (negedge clk125) begin
      if (RESET) begin
         tx_state <= TX_IDLE;
         packet_count <= 32'd0;
         tx_outen <= 0;
         tx_count <= 0;
         tx_delay_count <= 0;
         fifo0_read_en <= 0;
         fifo1_read_en <= 0;
         tx_led <= 24'd0;
         payload_length <= 16'b0;
         fifo_data_count <= 0;
         udp_en_buf <= 0;
      end
      else begin
         // This blinks LED 1 on the SP605 as a sanity check that the
         // clocks are working.
         tx_led <= tx_led+24'd1;
         // Buffer input flags and values which are crossing clock domains
         udp_en_buf <= udp_en;
         if (fifo_select) begin
            fifo_data_count <= fifo1_data_count;
         end else begin
            fifo_data_count <= fifo0_data_count;
         end
         case (tx_state)
           // IDLE STATE
           // The state machine idles in this state until switch 1 is
           // flipped, and returns to this state when the switch is
           // turned off. This is also the state that the FSM enters
           // after reset, so all signals must be set to their desired
           // initial state here.
           TX_IDLE: begin
              payload_length <= payload_length_in;
              tx_delay_count <= 0;
              // need the "+ 2" below because of the FWFT mode for FIFO
              if (udp_en_buf && ((fifo_data_count + 2) >= payload_length)) begin
                 tx_state <= TX_PACKET_CONSTRUCT_0;
                 crc_clear <= 0;
              end
              else begin // Reset signals to desired initial state
                 tx_outen <= 0;
                 tx_count <= 0;
                 crc_clear <= 1;
              end
           end // case: TX_IDLE
           // PACKET_CONSTRUCT STATES
           TX_PACKET_CONSTRUCT_0: begin
              mac_destination <= mac_destination_in;
              ip_source <= ip_source_in;
              ip_destination <= ip_destination_in;
              port_source <= port_source_in;
              port_destination <= port_destination_in;
              tx_state <= TX_PACKET_CONSTRUCT_1;
           end
           TX_PACKET_CONSTRUCT_1: begin
              packet_length <= payload_length+header_length;
              ip_header <= {16'h4500,payload_length+16'd28,48'hB3FE00008011};
              udp_header <= {port_source,port_destination,payload_length+16'd8};
              tx_state <= TX_PACKET_CONSTRUCT_2;
           end
           TX_PACKET_CONSTRUCT_2: begin
              ip_checksum_constant_0 <= ip_header[79:64]+ip_header[63:48]+ip_header[47:32]+ip_header[31:16]+ip_header[15:0];
              tx_state <= TX_PACKET_CONSTRUCT_3;
           end
           TX_PACKET_CONSTRUCT_3: begin
              ip_checksum_constant_1 <= ip_checksum_constant_0[31:16]+ip_checksum_constant_0[15:0];
              tx_state <= TX_PACKET_CONSTRUCT_4;
           end
           TX_PACKET_CONSTRUCT_4: begin
              ip_checksum_calc_0 <= ip_checksum_constant_1 + (ip_source[31:24]<<8)+ip_source[23:16]+(ip_source[15:8]<<8)+ip_source[7:0]+(ip_destination[31:24]<<8)+ip_destination[23:16]+(ip_destination[15:8]<<8)+(ip_destination[7:0]);
              tx_state <= TX_PACKET_CONSTRUCT_5;
           end
           TX_PACKET_CONSTRUCT_5: begin
              ip_checksum_calc_1 <=  ((ip_checksum_calc_0&32'h0000FFFF)+(ip_checksum_calc_0>>16));
              tx_state <= TX_PACKET_CONSTRUCT_6;
           end
           TX_PACKET_CONSTRUCT_6: begin
              ip_checksum_calc_2 <= ~((ip_checksum_calc_1&32'h0000FFFF)+(ip_checksum_calc_1>>16));
              tx_state <= TX_PACKET_CONSTRUCT_7;
           end
           TX_PACKET_CONSTRUCT_7: begin
              ip_checksum <= ip_checksum_calc_2[15:0];
              tx_state <= TX_PACKET_CONSTRUCT_8;
           end
           TX_PACKET_CONSTRUCT_8: begin
              packet_header <= {ethernet_preamble,mac_destination,mac_source,ethernet_type,ip_header,ip_checksum,ip_source,ip_destination,udp_header,udp_checksum};
              tx_state <= TX_SHIFT_OUT_0;
           end
           // SHIFT_OUT_0 STATE
           // This state shifts out the 50 Byte packet header,
           // starting with the ethernet preamble, one Byte at a
           // time. It is also necessary to "stage" the first byte of
           // payload data here so it is available to the CRC module.
           TX_SHIFT_OUT_0: begin
              // Output is enabled and bytes are being continually
              // shifted out of the packet_header reg
              tx_outen <= 1;
              tx_count <= tx_count+17'd1;
              tx_out[7:0] <= packet_header[((header_length*8)-1):((header_length*8)-8)];
              packet_header <= {packet_header[((header_length*8)-9):0],8'b0};
              // Once the packet header reg is nearing emptiness,
              // prepare to transition to shifting out payload
              if (tx_count == (header_length-2)) begin
                 // First payload byte (ie, data stored in address 0
                 // of a BRAM) should get passed in to
                 // payload_next_byte here
                 if (fifo_select) begin
                    fifo1_read_en <= 1;
                    payload_byte_next <= fifo1_data;
                 end else begin
                    fifo0_read_en <= 1;
                    payload_byte_next <= fifo0_data;
                 end
              end
              if (tx_count == (header_length-1)) begin
                 tx_state <= TX_SHIFT_OUT_1;
                 tx_count <= tx_count+17'd1;
                 payload_byte <= payload_byte_next;
                 // Second payload byte (ie, data stored in address 1
                 // of a BRAM) should get passed in to
                 // payload_next_byte here
                 if (fifo_select) begin
                    fifo1_read_en <= 1;
                    payload_byte_next <= fifo1_data;
                 end else begin
                    fifo0_read_en <= 1;
                    payload_byte_next <= fifo0_data;
                 end
              end

           end // case: TX_SHIFT_OUT_0
           // SHIFT_OUT_1 STATE
           // This state shifts out the 2500 Byte payload one Byte at
           // a time, and also shifts out the first Byte of the CRC
           // once the payload has all been sent.
           TX_SHIFT_OUT_1: begin
              // If the payload has all been sent, start sending the
              // CRC.
              if (tx_count == packet_length) begin
                 fifo0_read_en <= 0;
                 fifo1_read_en <= 0;
                 tx_state <= TX_CRC_OUT_0;
                 tx_out[7:0] <= crc_out;
                 tx_count <= tx_count+17'd1;
              end
              // Otherwise send out the payload one Byte at a time.
              else begin
                 if (tx_count + 2 >= packet_length) begin
                    fifo0_read_en <= 0;
                    fifo1_read_en <= 0;
                 end else begin
                    if (fifo_select) begin
                       fifo1_read_en <= 1;
                    end else begin
                       fifo0_read_en <= 1;
                    end
                 end
                 tx_outen <= 1;
                 tx_out[7:0] <= payload_byte;
                 payload_byte <= payload_byte_next;
                 // To utilize a BRAM or other memory element, hook up
                 // its output to payload_byte_next. Iterating through
                 // the elements of the BRAM (ie increased the read
                 // address) must occur elsewhere.
                 if (fifo_select) begin
                    payload_byte_next <= fifo1_data;
                 end else begin
                    payload_byte_next <= fifo0_data;
                 end
                 tx_count <= tx_count+17'd1;
              end
           end // case: TX_SHIFT_OUT_1
           // CRC_OUT_0 state
           // Finish shifting out the 32-bit CRC.
           TX_CRC_OUT_0: begin
              if (tx_count == (packet_length+4)) begin
                 tx_count <= 0;
                 tx_outen <= 0;
                 crc_clear <= 1;
                 tx_state <= TX_WAIT;
                 packet_count <= packet_count + 32'd1;
              end
              else begin
                 tx_out[7:0] <= crc_out;
                 tx_count <= tx_count+17'd1;
              end
           end // case: TX_CRC_OUT_0
           // WAIT state
           // The FSM ends up in this state after it has completed
           // shifting out a packet.
           TX_WAIT: begin
              // If switch 1 has since been turned off, stop
              // outputting packets and return to idle state.
              if (udp_en_buf == 0) begin
                 tx_state <= TX_IDLE;
              end
              // Otherwise wait .1us before returning to SHIFT_OUT_0
              // state to start shifting out another packet.
              else begin
                 if (tx_delay_count == 12) begin
                    tx_state <= TX_IDLE;
                    tx_delay_count <= 0;
                 end
                 else begin
                    tx_delay_count <= tx_delay_count+8'd1;
                    packet_header <= {ethernet_preamble,mac_destination,mac_source,ethernet_type,ip_header,ip_checksum,ip_source,ip_destination,udp_header,udp_checksum};
                    tx_outen <= 0;
                    tx_count <= 0;
                    crc_clear <= 0;
                 end
              end
           end // case: TX_WAIT
           default: tx_state <= TX_IDLE;
         endcase // case (tx_state)
      end // else: !if(RESET)
   end // always @


   // MIIM Module

   // This module controls the MIIM interface to the MV88E1111, a
   // two-wire management interface for reading and writing control
   // and status registers. This module is NOT a complete
   // implementation of the MIIM interface. It only reads out the
   // value of the single register specified by the parameter
   // DEFAULT_MIIM_ADDR. It does not support reading register pages other than
   // 0, or writing to registers.

   always @ (posedge clk125) begin
      miim_probe_en_buf <= miim_probe_en;

      case (miim_state)
        MIIM_IDLE: begin
           if (miim_probe_en_buf) miim_state <= MIIM_SHIFT_OUT_0;
           else miim_state <= MIIM_IDLE;
           //uart_do_transmit <= 0;
           mdc_initialize <= 0;
           mdio_data_o[4:0] <= miim_addr;
        end
        MIIM_SHIFT_OUT_0: begin
           mdc_count <= mdc_count+10'd1;
           if (mdc_count == MDC_WAIT) begin
              if (mdc_initialize == 31) begin
                 mdc_count <= 0;
                 mdc <= 0;
                 miim_state <= MIIM_SHIFT_OUT_2;
              end
              else begin
                 mdc_count <= 0;
                 mdc <= 0;
                 miim_state <= MIIM_SHIFT_OUT_1;
              end
           end // if (mdc_count == MDC_WAIT)
        end // case: SHIFT_OUT_0
        MIIM_SHIFT_OUT_1: begin
           mdc_count <= mdc_count+10'd1;
           if (mdc_count == MDC_WAIT) begin
              mdc_count <= 0;
              mdc <= 1;
              mdc_initialize <= mdc_initialize+5'd1;
              miim_state <= MIIM_SHIFT_OUT_0;
           end
        end
        MIIM_SHIFT_OUT_2: begin
           mdc_count <= mdc_count+10'd1;
           mdio <= mdio_data_o[13];
           if (mdc_count == MDC_WAIT) begin
              mdio_count <= mdio_count+5'd1;
              mdc_count <= 0;
              mdc <= 1;
              if (mdio_count == 14) begin
                 miim_state <= MIIM_SHIFT_IN_0;
                 mdio_count <= 0;
              end
              else begin
                 miim_state <= MIIM_SHIFT_OUT_0;
                 mdio_data_o <= {mdio_data_o[12:0],1'b0};
              end
           end
        end
        MIIM_SHIFT_IN_0: begin
           mdio <= 1'b1;
           mdc_count <= mdc_count + 10'd1;
           if (mdc_count == MDC_WAIT) begin
              mdc_count <= 0;
              mdc <= 1'b0;
              miim_state <= MIIM_SHIFT_IN_1;
              mdio_data_i <= {mdio_data_i[15:0],PHY_MDIO};
           end
        end
        MIIM_SHIFT_IN_1: begin
           mdc_count <= mdc_count + 10'd1;
           if (mdc_count == MDC_WAIT) begin
              mdio_count <= mdio_count+5'd1;
              mdc_count <= 0;
              mdc <= 1'b1;
              if (mdio_count == 16) begin
                 //uart_tx_byte <= mdio_data_i[15:8];
                 //uart_do_transmit <= 1;
                 miim_state <= MIIM_UART_OUT_0;
                 mdio_count <= 0;
              end
              else
                miim_state <= MIIM_SHIFT_IN_0;
           end
        end // case: SHIFT_IN_1
        MIIM_UART_OUT_0: begin
           //uart_do_transmit <= 0;
           //if (uart_tx_bsy == 1) begin
           //   miim_state <= MIIM_UART_OUT_1;
           //end
           miim_state <= MIIM_UART_OUT_1;
        end
        MIIM_UART_OUT_1: begin
           //if (uart_tx_bsy == 0) begin
           //   uart_tx_byte <= mdio_data_i[7:0];
           //   uart_do_transmit <= 1;
           //   miim_state <= MIIM_WAIT;
           //end
           miim_state <= MIIM_WAIT;
        end
        MIIM_WAIT: begin
           data_received <= ~(&mdio_data_i);
           //uart_do_transmit <= 0;
           if (miim_probe_en_buf == 0) begin
              miim_state <= MIIM_IDLE;
           end
           else miim_state <= MIIM_WAIT;
        end
      endcase
   end

   assign led_clock_throb = tx_led[23];
   assign phy_mdc = mdc ? 1'b1 : 1'b0;
   assign PHY_MDIO = mdio ? 1'bz : 1'b0;

endmodule
