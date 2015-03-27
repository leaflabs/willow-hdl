/*
 * sata_top.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Jess Barber <jessb@leaflabs.com>
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: July-August 2013
 *
 * This core is mostly a thin port-mapping wrapper between the (44.1MHz
 * domain) sata_core module and the lower-level (and variable clock)
 * sata_control core (which itself functionally wraps the Intelliprop SATA
 * implementation.
 *
 * One feature which has been added to this file is managing the output end of
 * the DAQ-SATA FIFO, which is used when writing acquired board samples to the
 * disk.
 *
 * Interface:
 *
 *      clkout:
 *          TODO
 *
 *      sata_ready:
 *          TODO
 *
 *      phy_ready:
 *          TODO
 *
 *      sata_clk:
 *      sata_txdt_p0:
 *      sata_txdt_n0:
 *      sata_rxdt_p0:
 *      sata_rxdt_n0:
 *          TODO
 *
 *      write_enable:
 *      read_enable:
 *          These two input lines initiate SATA operations, assuming a disk is
 *          attached and any appropriate setup has already taken place.
 *          These lines are mutually exclusive and should never be pulled high
 *          simultaneously.
 *
 *      read_sectorcount:
 *      write_sectorcount:
 *          These specifify the length (in 512 byte sectors) for read and
 *          write operations (respectively). They must be configured before
 *          the read_enable or write_enable lines are pulled high, and can not
 *          be modified in the middle of a continuous string of operations
 *          (aka, changes won't be caught without toggling the read_enable or
 *          write_enable line).
 *
 *      read_sectoraddress:
 *      write_sectoraddress:
 *          These specifify the initial location (in 512 byte sectors) for
 *          read and write operations (respectively). They must be configured
 *          before the read_enable or write_enable lines are pulled high, and
 *          can not be modified in the middle of a continuous string of
 *          operations (aka, changes won't be caught without toggling the
 *          read_enable or write_enable line).
 *
 *      write_delay_cycles:
 *          This port allows 
 *          Like the other configuration ports, this one should be setup
 *          before a write operation. It can be changed mid-operation, but
 *          shouldn't be to avoid cross-clock domain timing races.
 *
 *      zero_enable:
 *          Depricated.
 *
 *      daq_fifo_feedback_count_strobe (output):
 *          Strobed high at the begining of a SATA write so that the FIFO
 *          count (aka, number of words in the FIFO) is sampled at a known
 *          time. Otherwise the value would have a huge variance based on the
 *          time of sampling within the write transaction window. This signal
 *          crosses clock domains.
 *
 */


module sata_top #( parameter SERDES_TYPE = "SPARTAN6",
                   parameter SIMULATION = 0)
   (
    input wire         write_enable,
    input wire         read_enable,
    input wire         zero_enable, // Pull this high when writing to write zeroes to the HD

    input wire [16:0]  read_sectorcount,
    input wire [16:0]  write_sectorcount,
    input wire [47:0]  read_sectoraddress,
    input wire [47:0]  write_sectoraddress,
    output wire [47:0] current_sectoraddress,

    input wire [23:0]  write_delay_cycles,

    output wire        sata_txdt_p0,
    output wire        sata_txdt_n0,
    input wire         sata_rxdt_p0,
    input wire         sata_rxdt_n0,
    input wire         sata_clk,

    input wire         RESET,

    output wire        clkout,

    input wire         sata_read_fifo_rd_clk,
    input wire         sata_read_fifo_read_en,
    input wire         sata_read_fifo_reset,
    output wire        sata_read_fifo_write_en,
    output wire        sata_read_fifo_empty,
    output wire        sata_read_fifo_full,
    output wire [9:0]  sata_read_fifo_rd_data_count,
    output wire [31:0] sata_read_fifo_dout,
    output wire        sata_read_fifo_underflow,
    output wire        sata_read_fifo_overflow,

    output wire        daq_fifo_clock,
    output wire        daq_fifo_read_en,
    input wire         daq_fifo_empty,
    input wire [31:0]  daq_fifo_data,
    output reg         daq_fifo_feedback_count_strobe = 0,

    // status Lines
    output wire        sata_ready, 
    output wire        phy_ready,
    output wire        error_sata_lowlevel,
    output wire [7:0]  sata_error_code,
    output wire [3:0]  sata_state

    );

   wire [31:0]         sata_read_fifo_din;
   wire                sata_read_fifo_out_prog_full;

   // These "safe" lines are to ensure that low-level read transactions
   // complete even if the read_enable line has gone low (meaning the
   // sata_read_fifo is no longer being read from, and would fill up, causing
   // the SATA core to hang waiting for the FIFO to empty before completing
   // it's transaction).
   wire sata_read_fifo_full_safe;
   wire sata_read_fifo_out_prog_full_safe;
   wire sata_read_fifo_write_en_safe;
   assign sata_read_fifo_full_safe = sata_read_fifo_full && read_enable;
   assign sata_read_fifo_out_prog_full_safe = sata_read_fifo_out_prog_full && read_enable;
   assign sata_read_fifo_write_en_safe = sata_read_fifo_write_en && read_enable;

   wire                sata_write_fifo_read_en;
   wire                sata_write_fifo_empty;
   wire [31:0]         sata_write_fifo_data;

   sata_control
     #( .SERDES_TYPE("SPARTAN6"),
        .SIMULATION(SIMULATION))
   sata_control_inst (
                      .write_enable(write_enable),
                      .read_enable(read_enable),
                      .zero_enable(zero_enable),

                      .read_sectorcount(read_sectorcount),
                      .write_sectorcount(write_sectorcount),
                      .read_sectoraddress(read_sectoraddress),
                      .write_sectoraddress(write_sectoraddress),
                      .current_sectoraddress(current_sectoraddress),

                      .write_delay_cycles(write_delay_cycles),

                      .HOST_TXDT_P0(sata_txdt_p0),
                      .HOST_TXDT_N0(sata_txdt_n0),
                      .HOST_TXDT_P1(),
                      .HOST_TXDT_N1(),
                      .HOST_RXDT_P0(sata_rxdt_p0),
                      .HOST_RXDT_N0(sata_rxdt_n0),
                      .HOST_RXDT_P1(1'b0),
                      .HOST_RXDT_N1(1'b0),

                      .REFCLK(sata_clk),

                      .RESET(RESET),

                      .udp_fifo_din(sata_read_fifo_din),
                      .udp_fifo_write_en(sata_read_fifo_write_en),
                      .udp_fifo_full(sata_read_fifo_full_safe),
                      .udp_fifo_prog_full(sata_read_fifo_out_prog_full_safe),

                      .daq_fifo_clock(daq_fifo_clock),
                      .daq_fifo_read_en(sata_write_fifo_read_en),
                      .daq_fifo_empty(sata_write_fifo_empty),
                      .daq_fifo_data(sata_write_fifo_data),

                      .clkout(clkout),

                      .device_ready(),
                      .sata_ready(sata_ready),
                      .phy_ready(phy_ready),
                      .sata_error(error_sata_lowlevel),
                      .sata_error_code(sata_error_code),
                      .sata_state(sata_state)
                      );

   wire sata_read_fifo_reset_combined;
   assign sata_read_fifo_reset_combined = sata_read_fifo_reset || RESET;
   sata_read_fifo sata_read_fifo_inst (
                           .wr_clk(clkout),
                           .rd_clk(sata_read_fifo_rd_clk),
                           .rst(sata_read_fifo_reset),
                           .din(sata_read_fifo_din),
                           .wr_en(sata_read_fifo_write_en_safe),
                           .rd_en(sata_read_fifo_read_en),
                           .empty(sata_read_fifo_empty),
                           .full(sata_read_fifo_full),
                           .dout(sata_read_fifo_dout),
                           .overflow(sata_read_fifo_overflow),
                           .underflow(sata_read_fifo_underflow),
                           .rd_data_count(sata_read_fifo_rd_data_count),
                           .prog_full(sata_read_fifo_out_prog_full)
                           );


    // SATA write padding logic
    // this might get broken out to a seperate file?
    parameter SAMPLE_WORDS = 565;
    // NB: word_count will roll-over perfectly at 1024 words, which is 8x 128
    // word (512 byte) SATA sectors
    reg [9:0] word_count = 0;
    reg real_data_mode = 1'b1;
    assign daq_fifo_read_en = real_data_mode ? sata_write_fifo_read_en : 1'b0;
    assign sata_write_fifo_data = real_data_mode ? daq_fifo_data : 32'd0;
    assign sata_write_fifo_empty = real_data_mode ? daq_fifo_empty : 1'b0;
    always @(posedge daq_fifo_clock) begin
        if (RESET) begin
            word_count <= 10'd0;
            real_data_mode <= 1'b1;
            daq_fifo_feedback_count_strobe <= 0;
        end else if (!write_enable && daq_fifo_empty) begin
            // in this condition, might need to finish off the transaction with
            // zeros so that the SATA core doesn't get totally hung.
            // This is also the default case when write_enable has never been
            // asserted.
            // TODO: current code gives a spurious one-cycle pulse on
            // sata_write_fifo_empty at the end of writing; this may be
            // sub-optimal for disk write reliability.
            // TODO: this could perhaps be made more robust, but "seems to
            // work"
            word_count <= 0;
            real_data_mode <= 1'b0;
        end else begin
            if (sata_write_fifo_read_en) begin
                // rolls over automagically!
                word_count <= word_count + 10'd1;
            end
            // TODO: there are more readable ways to re-phrase the logic below.
            if ((word_count > SAMPLE_WORDS-2) && (word_count < 10'd1023)) begin
                if (word_count == (SAMPLE_WORDS-1) && !sata_write_fifo_read_en) begin
                    real_data_mode <= 1'b1;
                end else begin
                    real_data_mode <= 1'b0;
                end
            end else begin
                if (word_count == 10'd1023 && !sata_write_fifo_read_en) begin
                    real_data_mode <= 1'b0;
                end else begin
                    real_data_mode <= 1'b1;
                end
            end

            // For SATA write feedback loop; trigger just after start of
            // writing. See interface notes at top.
            if (sata_write_fifo_read_en && word_count == 10'd0) begin
                daq_fifo_feedback_count_strobe <= 1;
            end else begin
                daq_fifo_feedback_count_strobe <= 0;
            end
        end
    end


endmodule
