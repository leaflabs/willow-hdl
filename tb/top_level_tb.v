/*
 * top_level_tb.v
 *
 * Copyright: (C) 2015 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Jess Barber (jessb@leaflabs.com)
 * Date: January 2015
 *
 * This module contains a top level test bench to drive all tests and
 * simulations for the high-level Willow system.
 *
 */


`timescale 1ps/1ps
module top_level_tb;

   reg clk50;
   always @(clk50) begin
      #9960 clk50 <= !clk50;
   end

   reg clk150;
   always @(clk150) begin
      #3320 clk150 <= !clk150;
   end

   initial begin
      // the first event that sets the clock in motion
      #0      clk50 <= 1'b0;
      #0      clk150 <= 1'b0;
   end

   // Resets
   reg rstb;
   reg rstb_drive; // delay the drive a bit longer to allow serdes to start

   // SPI lines
   reg clk20 = 1'b0;
   reg t_rb = 1'b0;
   reg mlb = 1'b1;
   reg start = 1'b0;
   reg [7:0] tdat = 8'b00000000;
   reg [1:0] cdiv = 0;
   reg [95:0] spi_packet_out = 0;
   wire       spi_miso;
   wire       spi_cs;
   wire       spi_sck;
   wire       spi_mosi;
   wire       done;
   wire [7:0] rdata;

   parameter PERIOD = 50000;
   parameter real DUTY_CYCLE = 0.5;
   parameter OFFSET = 100000;
   initial    // Clock process for clk20
     begin
        #OFFSET;
        forever
          begin
             clk20 = 1'b0;
             #(PERIOD-(PERIOD*DUTY_CYCLE)) clk20 = 1'b1;
             #(PERIOD*DUTY_CYCLE);
          end
     end

   // DDR3 lines
   wire        ddr3_ck, ddr3_ck_n;
   wire        ddr3_cke;
   wire        ddr3_ras_n;
   wire        ddr3_cas_n;
   wire        ddr3_we_n;
   wire        ddr3_dm;
   wire [2:0]  ddr3_ba; // 3bits
   wire [13:0] ddr3_a; //14bit
   wire [7:0]  ddr3_dq; //8bit
   wire        ddr3_dqs, ddr3_dqs_n;
   wire        ddr3_odt;
   wire        ddr3_dram_rst_n;

   wire        zio5, rzq5;
   PULLDOWN zio_pulldown5 (.O(zio5));   PULLDOWN rzq_pulldown5 (.O(rzq5));

   // SATA lines
   wire TXDT_P_post;
   wire TXDT_N_post;
   wire TXDT_P;
   wire TXDT_N;
   wire RXDT_P_post;
   wire RXDT_N_post;
   wire RXDT_P;
   wire RXDT_N;

   assign TXDT_P_post = TXDT_P === TXDT_N ? 1'bx : TXDT_P;
   assign RXDT_P_post = RXDT_P === RXDT_N ? 1'bx : RXDT_P;
   assign TXDT_N_post = TXDT_P === TXDT_N ? 1'bx : !TXDT_P;
   assign RXDT_N_post = RXDT_P === RXDT_N ? 1'bx : !RXDT_P;

   wire [5:0] n_led;

   main_wiredleaf uut (.clock_50mhz(clk50),
                       .n_reset_button(rstb),

                       .n_button(),
                       .n_led(n_led),
                       .ext(),

                       .sata1_tx_n(TXDT_N),
                       .sata1_tx_p(TXDT_P),
                       .sata1_rx_n(RXDT_N_post),
                       .sata1_rx_p(RXDT_P_post),
                       .sata_clk_n(!clk150),
                       .sata_clk_p(clk150),

                       .mcu_sclk(spi_sck),
                       .mcu_n_cs(spi_cs),
                       .mcu_mosi(spi_mosi),
                       .mcu_miso(spi_miso),

                       .mcb5_dram_dq(ddr3_dq),
                       .mcb5_dram_a(ddr3_a),
                       .mcb5_dram_ba(ddr3_ba),
                       .mcb5_dram_ras_n(ddr3_ras_n),
                       .mcb5_dram_cas_n(ddr3_cas_n),
                       .mcb5_dram_we_n(ddr3_we_n),
                       .mcb5_dram_odt(ddr3_odt),
                       .mcb5_dram_reset_n(ddr3_dram_rst_n),
                       .mcb5_dram_cke(ddr3_cke),
                       .mcb5_dram_dm(ddr3_dm),
                       .mcb5_rzq(rzq5),
                       .mcb5_zio(zio5),
                       .mcb5_dram_dqs(ddr3_dqs),
                       .mcb5_dram_dqs_n(ddr3_dqs_n),
                       .mcb5_dram_ck(ddr3_ck),
                       .mcb5_dram_ck_n(ddr3_ck_n)

                       );

   // SPI master data model
   spi_master spi_master_inst (
                               .rstb(rstb),
                               .clk(clk20),
                               .mlb(mlb),
                               .start(start),
                               .tdat(tdat),
                               .cdiv(cdiv),
                               .din(spi_miso),
                               .ss(spi_cs),
                               .sck(spi_sck),
                               .dout(spi_mosi),
                               .done(done),
                               .rdata(rdata));

   reg        dummy_pass = 1'b0;

   task check_dummy;
      begin
         $display("Dummy test passed.");
         dummy_pass = 1'b1;
      end
   endtask

   integer i;

   task spi_write;
      input [95:0] spi_packet;
      begin
         cdiv = 2'b00;
         start = 1'b0;
         t_rb = 1'b1;
         spi_packet_out = spi_packet;
         //         spi_packet_out = 96'h5A_00_01_00_0000_02_00_00000000;
         for (i = 0; i < 12; i = i +1)
           begin
              $display ("Current value of i is %d", i);
              tdat = spi_packet_out[95:88];
              spi_packet_out = {spi_packet_out[87:0],8'b0};
              #50_000 start = 1'b1;
              #100_000 start = 1'b0;
              while (done == 0) begin
                 #10_000 $display("SPI transmitting.");
              end
           end
         #2_000_000;
      end
   endtask

   task daq_acquire_recipe;
      begin
         spi_write(96'h5A_00_01_01_0000_03_09_00000000); // Ensure DAQ-UDP output is off
         spi_write(96'h5A_00_01_01_0001_03_0B_00000000); // Ensure SATA output is off
         spi_write(96'h5A_00_01_01_0002_03_01_00000000); // Ensure DAQ was stopped
         spi_write(96'h5A_00_01_01_0003_02_01_00000000); // Ensure SATA was disabled
         spi_write(96'h5A_00_01_01_0004_03_0D_00000001); // Reset DAQ-SATA FIFO
         spi_write(96'h5A_00_01_01_0005_03_0D_00000000);
         spi_write(96'h5A_00_01_01_0006_01_02_DEADBEEF); // Config experiment cookie
         spi_write(96'h5A_00_01_01_0007_01_03_1EAF1AB5);
         spi_write(96'h5A_00_01_01_0008_02_14_00000000); // Config SATA start index
         spi_write(96'h5A_00_01_01_0009_03_02_00000780); // Set sample index past start
         spi_write(96'h5A_00_01_01_000A_02_01_00000002); // Config SATA to write from DAQ
         spi_write(96'h5A_00_01_01_000B_03_01_00000001); // Enable DAQ acquisition
         spi_write(96'h5A_00_01_01_000C_03_0B_00000003); // Enable DAQ-SATA output
         spi_write(96'h5A_00_01_01_000D_03_02_00000000); // Reset sample index
         spi_write(96'h5A_00_01_00_000E_00_00_00000000); // Read error field
         spi_write(96'h5A_00_01_00_000F_01_00_00000000); // Read central module error field
         spi_write(96'h5A_00_01_00_0010_02_00_00000000); // Read sata error field
         spi_write(96'h5A_00_01_00_0011_03_00_00000000); // Read daq error field
      end
   endtask // spi_write

   reg all_tests_pass = 1'b0;

   initial begin
      #0         rstb        <= 1'b1; // only reset stays as a 0.
      #0         rstb_drive  <= 1'b0; // do not change
      #1000      rstb        <= 1'b0; // do not change
      #3000000   rstb        <= 1'b1; // do not change
      #10100000  rstb_drive  <= 1'b1; // make the drive model come up a bit later; do not change

      $display("=================== start %m");
      // insert desired tests/checks/recipes here
      // all_tests_pass = test0 && test1 && test2... etc.
      check_dummy();
      all_tests_pass = dummy_pass;
      if (all_tests_pass == 1'b1) begin
         $display("PASS: all tests pass.");
      end
      else begin
         $display("FAIL: one or more tests failed.");
      end
      $display("=================== end %m");

      $finish();
   end

endmodule
