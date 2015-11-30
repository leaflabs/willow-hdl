/*
 * rhd2000_driver.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: The MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: April 2013
 *
 * This module implements intermediate-level configuration, control, and data
 * readout from an Intan RHD2000-series digital front-end chip; it calls down
 * into a rhd2000_spi module to handle SPI-layer communications.
 *
 * See page 15 of the RHD2000-series datasheet for a command-level timing
 * diagram.
 *
 * See ../docs/sampling.txt for an overview of of the ADC and auxiliary sample
 * timing.
 *
 * Interface:
 *
 *      clk_i:
 *          44.1MHz clock to generate 22.05MHz SPI SCLK
 *      reset_i:
 *          global synchronous reset line
 *      reply_o:
 *          the SPI value from the chip
 *      slot_o:
 *          the slot number of the currently valid reply_o
 *      dac_waveform_value:
 *          the 8bit DAC value to be written in extra slot #0 of every chip
 *          sample
 *      extra_cmd_i, extra_cmd_en_i:
 *          mechanism for inserting "extra" commands every 15'th chip sample
 *          (CMD slot). if extra_cmd_en_i is high when that slot comes around,
 *          then extra_cmd_i will be inserted, else a DUMMY command is sent
 *          instead.
 *      chip_sample_index_o:
 *          the low 8bits of the internal 24bit chip sample index counter. the
 *          internal counter needs to be large to accomodate the long (20000+
 *          sample) initialization routine: during continous acquisition this
 *          value rolls over at 30 (the 1KHz DAC waveform period).
 *      sample_extra_en_i:
 *          Controls whether AUX0, AUX1, AUX2, TEMP, and SUPPLY should
 *          actually be sampled or just DUMMY slots.
 *
 * Build-time Parameters (defined in Makefile):
 *
 *      FORCE_ACQUISITION:
 *          ignore IDENT results and force continuous acquisition?
 *      SIMULATION:
 *          initialization is cut short in simulation (otherwise would take 2+
 *          seconds)
 *      SIGNED_ADC_VALUES:
 *          whether the Intan chip should be configured to return ADC samples
 *          as signed integers (in two's compliment mode) or unsigned
 *          integers.
 *      MOSI_ON_FALLING and MOSI_ON_NEXT_RISING:
 *          see rhd2000_spi.v
 */
module rhd2000_driver (
                       input wire         clk_i, // expect a 44.1MHz clock -> 22.05MHz SCLK
                       input wire         reset_i,
                       output wire        ready_o,
                       output wire [15:0] reply_o,
                       output wire [5:0]  slot_o,
                       output wire [7:0]  chip_sample_index_o,
                       output wire        sclk_o,
                       output wire        csbar_o,
                       output wire        mosi_o,
                       input wire         miso_i,
                       input wire [7:0]   dac_waveform_value,
                       input wire [3:0]   sample_index_mod15_i,
                       input wire         sample_extra_en_i,
                       input wire [15:0]  extra_cmd_i,
                       input wire         extra_cmd_en_i
                       );

   parameter SIMULATION = 0;
   parameter FORCE_ACQUISITION = 0;
   parameter MOSI_ON_FALLING = 1;
   parameter MOSI_ON_NEXT_RISING = 1;
   parameter SIGNED_ADC_VALUES = 1;

   reg [5:0]                              slot_request = 0;
   reg [23:0]                             chip_sample_index = 0;
   reg [3:0]                              sample_index_mod15 = 0;
   reg [15:0]                             cmd = 0;
   reg [5:0]                              timer = 0;
   reg [2:0]                              state = STATE_SETTLE;
   reg                                    chip_sync = 0;
   reg                                    ready = 0;
   reg                                    alive = 0;
   wire [15:0]                            reply;
   reg [15:0]                             reply_buf = 0;
   reg [5:0]                              slot_buf [0:3];
   initial begin
      slot_buf[0] = 0;
      slot_buf[1] = 0;
      slot_buf[2] = 0;
      slot_buf[3] = 0;
   end

   assign reply_o = reply_buf;
   assign ready_o = ready;
   assign slot_o = slot_buf[3];
   assign chip_sample_index_o = chip_sample_index[7:0];

   rhd2000_spi #(
                 .MOSI_ON_FALLING(MOSI_ON_FALLING),
                 .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING)
                 ) spi_module (
                               .clk_i(clk_i),
                               .reset_i(reset_i),
                               .cmd_i(cmd),
                               .reply_o(reply),
                               .sclk_o(sclk_o),
                               .miso_i(miso_i),
                               .mosi_o(mosi_o),
                               .csbar_o(csbar_o),
                               .sync_i(chip_sync)
                               );

   localparam STATE_WAIT = 0;
   localparam STATE_SETTLE = 1;
   localparam STATE_IDENT = 2;
   localparam STATE_INIT = 3;
   localparam STATE_LOOP = 4;

   // states: WAIT, INIT, LOOP
   // negedge chip_sync should happen right after the last acq, so we can
   // grab last results and setup for next SPI shift
   always @(negedge clk_i) begin
      // synchronous reset code
      if (reset_i) begin
         slot_request <= 6'd0;
         slot_buf[0] <= 6'd0;
         slot_buf[1] <= 6'd0;
         slot_buf[2] <= 6'd0;
         slot_buf[3] <= 6'd0;
         reply_buf <= 16'd0;
         chip_sample_index <= 24'd0;
         sample_index_mod15 <= 4'd0;  // buffers from input
         cmd <= 16'd0;
         timer <= 6'd30;    // give SPI module some time to recover
         chip_sync <= 1'b0;
         // can't just jump into INIT! need to wait for SPI to settle
         state <= STATE_SETTLE;
         ready <= 1'b0;
         alive <= 1'b0;
      end else if (timer >= 41) begin
         timer <= 6'd0;
         chip_sync <= 1'b0;
      end else if (timer > 0) begin
         timer <= timer + 6'd1;
         chip_sync <= 1'b0;
      end else if (timer == 6'd0) begin
         timer <= timer + 6'd1;
         chip_sync <= 1'b1;
         // main loop; rising edge chip_sync
         case (state)
           STATE_WAIT: begin
              cmd <= {2'b01, 14'd0};  // DUMMY
           end // zilch
           STATE_SETTLE: begin
              // wait for a "sample" to let SPI settle
              // TODO: this should be a CALIBRATE period or more?
              if (chip_sample_index == 24'd10) begin
                 chip_sample_index <= 24'd0;
                 if (FORCE_ACQUISITION) begin
                    state <= STATE_INIT;
                 end else begin
                    state <= STATE_IDENT;
                 end
              end else
                chip_sample_index <= chip_sample_index + 24'd1;
           end
           STATE_IDENT: begin
              chip_sample_index <= chip_sample_index + 24'd1;
              case (chip_sample_index)
                0: begin
                   cmd <= {2'b11, 6'd40, 8'h00}; // READ(40)
                end
                3: begin
                   if (reply[8:0] != 8'd73) begin
                      $display("FAIL: wrong ID");
                      cmd <= {2'b01, 14'd0};  // DUMMY
                      state <= STATE_WAIT;
                   end
                   cmd <= {2'b11, 6'd41, 8'h00}; // READ(41)
                end
                6: begin
                   if (reply[8:0] != 8'd78) begin
                      $display("FAIL: wrong ID");
                      state <= STATE_WAIT;
                   end
                   cmd <= {2'b11, 6'd42, 8'h00}; // READ(42)
                end
                9: begin
                   if (reply[8:0] != 8'd84) begin
                      $display("FAIL: wrong ID");
                      state <= STATE_WAIT;
                   end
                   cmd <= {2'b11, 6'd43, 8'h00}; // READ(43)
                end
                12: begin
                   if (reply[8:0] != 8'd65) begin
                      $display("FAIL: wrong ID");
                      state <= STATE_WAIT;
                   end
                   cmd <= {2'b11, 6'd44, 8'h00}; // READ(44)
                end
                15: begin
                   if (reply[8:0] != 8'd78) begin
                      $display("FAIL: wrong ID");
                      state <= STATE_WAIT;
                   end
                   chip_sample_index <= 24'd0;
                   alive <= 1'b1;  // TODO: move this to bottom of STATE_INIT?
                   state <= STATE_INIT;
                end
              endcase
           end
           STATE_INIT: begin
              if (!SIMULATION) begin
                 case (chip_sample_index)
                   // Init all the registers
                   00: cmd <= {2'b10, 6'd00, 8'hFE}; // WRITE(00, 0xFE)
                   01: cmd <= {2'b10, 6'd01, 8'h42};
                   02: cmd <= {2'b10, 6'd02, 8'h04};
                   03: cmd <= {2'b10, 6'd03, 8'h01}; // (LED on)
                   04: begin
                      if (SIGNED_ADC_VALUES) begin
                         cmd <= {2'b10, 6'd04, 8'h5C}; // signed ADC values, two's compliment; DSP enabled, cutoff set to 12
                      end else begin
                         cmd <= {2'b10, 6'd04, 8'h1C}; // unsigned ADC values; DSP enabled, cutoff set to 12
                      end
                   end
                   05: cmd <= {2'b10, 6'd05, 8'h00};
                   06: cmd <= {2'b10, 6'd06, 8'h00};
                   07: cmd <= {2'b10, 6'd07, 8'h00};
                   08: cmd <= {2'b10, 6'd08, 8'h11};
                   09: cmd <= {2'b10, 6'd09, 8'h80};
                   10: cmd <= {2'b10, 6'd10, 8'h10};
                   11: cmd <= {2'b10, 6'd11, 8'h80};
                   12: cmd <= {2'b10, 6'd12, 8'h23};
                   13: cmd <= {2'b10, 6'd13, 8'h91};
                   14: cmd <= {2'b10, 6'd14, 8'hFF};
                   15: cmd <= {2'b10, 6'd15, 8'hFF};
                   16: cmd <= {2'b10, 6'd16, 8'hFF};
                   17: cmd <= {2'b10, 6'd17, 8'hFF};
                   18: cmd <= {2'b01, 14'd0}; // wait for amps to settle (DUMMY)
                   500000: // turn off LED after 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h00}; // WRITE(03, 0x00)
                   1000000: // turn on LED after 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h01}; // WRITE(03, 0x01)
                   1500000: // turn off LED after 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h00}; // WRITE(03, 0x00)
                   2000000: // ... having waited another 0.5 seconds
                     cmd <= {2'b10, 6'd00, 8'hDE}; // WRITE(00, 0xDE) Turn off fast amp settle
                   2000001:
                     cmd <= {2'b01, 6'b010101, 8'h00}; // CALIBRATE
                   2000002: // a couple DUMMY cycles while calibrating
                     cmd <= {2'b01, 14'd0}; // DUMMY
                   2000011: begin
                      // all done initializing, get outa here!
                      cmd <= {2'b01, 14'd0}; // DUMMY
                      state <= STATE_LOOP;
                      chip_sample_index <= 24'd0;
                      slot_request <= 6'd0;
                      if (FORCE_ACQUISITION) begin
                         alive <= 1'b1;
                      end
                   end
                   default:
                     cmd <= {2'b01, 14'd0}; // DUMMY
                 endcase
                 if (chip_sample_index < 2000011) begin
                    chip_sample_index <= chip_sample_index + 24'd1;
                 end
              end else begin // else we are in SIMULATION
                 case (chip_sample_index)
                   // Init all the registers
                   00: cmd <= {2'b10, 6'd00, 8'hFE}; // WRITE(00, 0xFE)
                   01: cmd <= {2'b10, 6'd01, 8'h42};
                   02: cmd <= {2'b10, 6'd02, 8'h04};
                   03: cmd <= {2'b10, 6'd03, 8'h01}; // (LED on)
                   04: begin
                      if (SIGNED_ADC_VALUES) begin
                         cmd <= {2'b10, 6'd04, 8'h5C}; // signed ADC values, two's compliment; DSP enabled, cutoff set to 12
                      end else begin
                         cmd <= {2'b10, 6'd04, 8'h1C}; // unsigned ADC values; DSP enabled, cutoff set to 12
                      end
                   end
                   05: cmd <= {2'b10, 6'd05, 8'h00};
                   06: cmd <= {2'b10, 6'd06, 8'h00};
                   07: cmd <= {2'b10, 6'd07, 8'h00};
                   08: cmd <= {2'b10, 6'd08, 8'h11};
                   09: cmd <= {2'b10, 6'd09, 8'h80};
                   10: cmd <= {2'b10, 6'd10, 8'h10};
                   11: cmd <= {2'b10, 6'd11, 8'h80};
                   12: cmd <= {2'b10, 6'd12, 8'h23};
                   13: cmd <= {2'b10, 6'd13, 8'h91};
                   14: cmd <= {2'b10, 6'd14, 8'hFF};
                   15: cmd <= {2'b10, 6'd15, 8'hFF};
                   16: cmd <= {2'b10, 6'd16, 8'hFF};
                   17: cmd <= {2'b10, 6'd17, 8'hFF};
                   18: cmd <= {2'b01, 14'd0}; // wait for amps to settle (DUMMY)
                   050: // turn off LED for 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h00}; // WRITE(03, 0x00)
                   100: // turn on LED for 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h01}; // WRITE(03, 0x01)
                   150: // turn off LED for 0.5 seconds
                     cmd <= {2'b10, 6'd03, 8'h00}; // WRITE(03, 0x00)
                   200:
                     cmd <= {2'b10, 6'd00, 8'hDE}; // WRITE(00, 0xDE) Turn off fast amp settle
                   201:
                     cmd <= {2'b01, 6'b010101, 8'h00}; // CALIBRATE
                   202: // a couple DUMMY cycles while calibrating
                     cmd <= {2'b01, 14'd0}; // DUMMY
                   211: begin
                      // all done initializing, get outa here!
                      cmd <= {2'b01, 14'd0}; // DUMMY
                      state <= STATE_LOOP;
                      chip_sample_index <= 24'd1;
                      slot_request <= 6'd0;
                      if (FORCE_ACQUISITION) begin
                         alive <= 1'b1;
                      end
                   end
                   default:
                     cmd <= {2'b01, 14'd0}; // DUMMY
                 endcase
                 if (chip_sample_index < 211) begin
                    chip_sample_index <= chip_sample_index + 24'd1;
                 end
              end

           end
           STATE_LOOP: begin
              // increment indexes
              if (slot_request == 34) begin
                 slot_request <= 6'd0;
              end else begin
                 slot_request <= slot_request + 6'd1;
              end

              if (slot_buf[1] == 1) begin
                 // ready should only be low for first few samples, then
                 // reply is actually valid for slot_reply == 0.
                 ready <= alive;
              end else if (slot_buf[1] == 2) begin
                 if (chip_sample_index == 24'd29) begin
                    chip_sample_index <= 24'd0;
                 end else begin
                    chip_sample_index <= chip_sample_index + 24'd1;
                 end
              end
              if ((slot_buf[2] == 33) && (sample_index_mod15 == 4'd0)) begin
                 // check IDENT reply; note that sample_index_mod15 updated
                 // during slot 32 below.
                 if (reply != 16'h0001) begin
                    // chip failed to identify correctly
                    alive <= 1'b0;
                 end
              end
              slot_buf[0] <= slot_request;
              slot_buf[1] <= slot_buf[0];
              slot_buf[2] <= slot_buf[1];
              slot_buf[3] <= slot_buf[2];
              reply_buf <= reply;

              // special case the "extra" slots
              case (slot_request)
                32: begin
                   // DAC waveform slot
                   cmd <= {2'b10, 6'd6, dac_waveform_value};
                   // pull in sample_index_mod15
                   sample_index_mod15 <= sample_index_mod15_i;
                end
                33: begin
                   // "left" extra slot (see doc/sampling.txt)
                   case (sample_index_mod15)
                     4'd00: cmd <= {2'b11, 6'd63, 8'd0};    // READ(63) (IDENT; should be 0x0001)
                     4'd01: cmd <= {2'b11, 6'd00, 8'd0};    // READ(00)
                     4'd02: cmd <= {2'b11, 6'd02, 8'd0};    // READ(02)
                     4'd03: cmd <= {2'b11, 6'd04, 8'd0};    // READ(04)
                     4'd04: cmd <= {2'b11, 6'd06, 8'd0};    // READ(06)
                     4'd05: cmd <= {2'b11, 6'd08, 8'd0};    // READ(08)
                     4'd06: cmd <= {2'b11, 6'd10, 8'd0};    // READ(10)
                     4'd07: cmd <= {2'b11, 6'd12, 8'd0};    // READ(12)
                     4'd08: cmd <= {2'b11, 6'd14, 8'd0};    // READ(14)
                     4'd09: cmd <= {2'b11, 6'd16, 8'd0};    // READ(16)
                     4'd10: begin
                        if (sample_extra_en_i) begin
                           cmd <= {2'b00, 6'd49, 8'h00};   // CONVERT(49) (TEMP)
                        end else begin
                           cmd <= {2'b01, 14'd0};          // DUMMY
                        end
                     end
                     4'd11: cmd <= {2'b01, 14'd0};          // DUMMY
                     4'd12: cmd <= {2'b01, 14'd0};          // DUMMY
                     4'd13: cmd <= {2'b01, 14'd0};          // EXT, so dummy
                     4'd14: begin
                        if (sample_extra_en_i) begin
                           cmd <= {2'b00, 6'd33, 8'h00};   // CONVERT(33) (AUX1)
                        end else begin
                           cmd <= {2'b01, 14'd0};          // DUMMY
                        end
                     end
                   endcase
                end
                34: begin
                   // "right" extra slot (see doc/sampling.txt)
                   case (sample_index_mod15)
                     4'd00: begin
                        if (extra_cmd_en_i) begin
                           cmd <= extra_cmd_i;             // ether CMD...
                           //$display(extra_cmd_i);
                        end else begin
                           cmd <= {2'b01, 14'd0};          // ...or DUMMY
                           //$display(extra_cmd_i);
                        end
                     end
                     4'd01: cmd <= {2'b11, 6'd01, 8'd0};    // READ(01)
                     4'd02: cmd <= {2'b11, 6'd03, 8'd0};    // READ(03)
                     4'd03: cmd <= {2'b11, 6'd05, 8'd0};    // READ(05)
                     4'd04: cmd <= {2'b11, 6'd07, 8'd0};    // READ(07)
                     4'd05: cmd <= {2'b11, 6'd09, 8'd0};    // READ(09)
                     4'd06: cmd <= {2'b11, 6'd11, 8'd0};    // READ(11)
                     4'd07: cmd <= {2'b11, 6'd13, 8'd0};    // READ(13)
                     4'd08: cmd <= {2'b11, 6'd15, 8'd0};    // READ(15)
                     4'd09: cmd <= {2'b11, 6'd17, 8'd0};    // READ(17)
                     4'd10: begin
                        if (sample_extra_en_i) begin
                           cmd <= {2'b00, 6'd48, 8'h00};   // CONVERT(48) (SUPPLY)
                        end else begin
                           cmd <= {2'b01, 14'd0};          // DUMMY
                        end
                     end
                     4'd11: cmd <= {2'b01, 14'd0};          // DUMMY
                     4'd12: cmd <= {2'b01, 14'd0};          // DUMMY
                     4'd13: begin
                        if (sample_extra_en_i) begin
                           cmd <= {2'b00, 6'd32, 8'h00};   // CONVERT(32) (AUX0)
                        end else begin
                           cmd <= {2'b01, 14'd0};          // DUMMY
                        end
                     end
                     4'd14: begin
                        if (sample_extra_en_i) begin
                           cmd <= {2'b00, 6'd34, 8'h00};   // CONVERT(34) (AUX2)
                        end else begin
                           cmd <= {2'b01, 14'd0};          // DUMMY
                        end
                     end
                   endcase
                end
                default: begin
                   // CONVERT(slot_reqeust)
                   cmd <= {2'b00, slot_request, 8'h00};
                end
              endcase
           end
         endcase
      end
   end
endmodule

