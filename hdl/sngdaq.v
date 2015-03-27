/*
 * sngdaq.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: The MIT License (See LICENSE file)
 * Author: Bryan Newbold <bnewbold@leaflabs.com>
 * Date: April 2013
 *
 * This module reads data out from up to 32 chip-level rhd2000_driver modules
 * and collects data and meta data for all 1024 channels into a "board sample"
 * into a dual-port BRAM which can be read out by other FPGA cores. The BRAM
 * is "double buffered", such that the external core can be reading from the
 * "last" BRAM while this core writes into the "current" BRAM.
 *
 * The NUM_CHIPS parameter can range from 1 to 32 and can be specified at
 * module instantiation time.
 *
 * Behavior:
 *
 *      Will initialize. As long as any chip driver has an active ready flag,
 *      acquistion will proceed indefinately.
 *
 *      If acquisition must start at a known sample index, it's best to write
 *      that number to initial_board_sample_index_i and pull
 *      reset_board_sample_index_i high for at least one clock cycle, then
 *      wait until the output board_sample_index ==
 *      initial_board_sample_index_i to start reading out samples.
 *
 * Timing:
 *
 *      There is a delay equivalent to several board samples between the
 *      sample actively being measured on the Intan chips at any given time
 *      and what is available in the BRAM output.
 *
 *      A board sample includes 35 slots of 16bit data for each of 32 chips.
 *      The chips sample in parallel, so, eg, they all sample the first
 *      spi_slot, then the second spit slot, etc. The BRAM can only be written
 *      to one 16bit value at a time, so this core buffers the most recent
 *      slot value for each of the chips seperately, and quickly iterates
 *      through that buffer, writing into the BRAM for all chips before the
 *      next set of SPI slots are available.
 *
 *      So, the chip_index loops fastest (at ~44.1MHz), then the spi_slot
 *      increments after every chip_index loop (at ~1.05MHz), then the board
 *      sample index increments after every spi_slot loop (at 30KHz).
 *
 *      On top of this loop is the 2KHz auxiliary channel sampling scheme
 *      (which loops every 15 board samples) and the 1KHz DAC output (which
 *      loops every 30 board samples).
 *
 * Interface:
 *
 *      clk_i:
 *          44.1MHz clock (double SPI SCLK)
 *      reset_i:
 *          synchronous global reset; after reset chip drivers must be
 *          re-initialized
 *      spi_csbar_o, spi_sclk_o, spi_mosi_o, spi_miso_i:
 *          SPI PHY lines for connection to Intan chips
 *      spi_ready_o:
 *          each pin of this wide bus indicates the connection/initialization
 *          status of a single Intan chip
 *      data_clk_i, data_en_i, data_addr_i, data_o:
 *          BRAM read-only external interface. there is 18kbits of data to
 *          read out; the data and address bitwidths can be reconfigured as
 *          usual with BRAMs
 *      data_ready_o:
 *          indicates "OK to read from BRAM". reads should start on the rising
 *          edge of this signal, after which the BRAM contents will be stable
 *          for 33 microseconds.
 *      board_sample_index_o:
 *          the monotonically-increasing board sample number of the currently
 *          valid BRAM contents
 *      extra_cmd_i, extra_cmd_en_i:
 *          the 16bit "extra" command is read and inserted at the start of
 *          every board sample acquistion. extra_cmd_i must be held; the value
 *          could be read at any time while extra_cmd_en_i is high.
 *          extra_cmd_en_i should be valid around the fall of the last
 *          data_ready_o for the command to be executed in the next board
 *          sample frame.
 *      extra_cmd_chip_i:
 *          specifies which chip will have the extra_cmd executed on it.
 *          if the top bit is high, the command will be sent to *all* chips
 *      gpio_pins:
 *          current GPIO pin status (both inputs and outputs)
 *      sample_extra_en_i:
 *          Controls whether AUX0, AUX1, AUX2, TEMP, and SUPPLY should
 *          actually be sampled or just DUMMY slots.
 *      dac_state:
 *          an 8-bit port intended to buffer the current global DAC output
 *          enable state, shared across all chips:
 *              bits 0 to 4: dac_select (which channel)
 *              bits 5 to 6: dac_scale (amplitude scale; see Intan datasheet)
 *              bit 7: DAC enabled (and powered)
 *          The motivation for this register is to avoid an extra auxiliary
 *          Intan register read transaction during live-sample streaming (when
 *          this information is embedded in every sample to aid impedance
 *          measurement implementation). During regular full-sample
 *          acquisition, this information can be derived from the round-robin
 *          register readout scheme every 15 samples.
 *          TODO: implemented in sngdaq_dummy, but not yet in this file.
 *      errors_o:
 *          a set of flags to indicated error conditions
 *      initial_board_sample_index_i, reset_board_sample_index_i:
 *          if reset_board_sample_index_i is high, the board_sample_index will
 *          be rewriten to initial_board_sample_index_i "as soon as
 *          convenient". if initial_board_sample_index_i ==
 *          board_sample_index_o, nothing will happen; otherwise the index
 *          will be reset the next time the two numbers match up modulo 30 (to
 *          preserve the DAC output waveform and AUX slot structure).
 *      FORCE_ACQUISITION:
 *          forces acquisition to loop continuously, regardless of whether any
 *          chips respond to IDENT
 *
 *  Notable Variables:
 *
 *      chip_sample_index: (8bit)
 *          as reported by the rhd2000_driver; what is currently being read
 *          in. NOT board_sample_index_o, which is the index of the sample
 *          currently residing the output BRAM. this 8bit value rolls over at
 *          30, not 255.
 *      chip_index: (0 to 31)
 *          the chip currently being read in.
 *      spi_slot: (0 to 34)
 *          the slot number currently being read in.
 *
 */
module sngdaq (
    input wire clk_i,           // 44.1 MHz clock
    input wire reset_i,
    input wire enable_acq,
    output wire spi_csbar_o,
    output wire spi_sclk_o,
    output wire spi_mosi_o,
    input wire[(NUM_CHIPS-1):0] spi_miso_i,
    input wire data_clk_i, // BRAM read clock
    input wire data_en_i, // BRAM read enable
    input wire[9:0] data_addr_i, // BRAM read address
    output wire[31:0] data_o, // BRAM data
    output wire data_ready_o, // "ok to read from BRAM"
    output reg[31:0] board_sample_index_o, // board sample number of BRAM contents
    output wire[(NUM_CHIPS-1):0] spi_ready_o, // status of all SPI drivers
    input wire[15:0] extra_cmd_i,
    input wire[5:0] extra_cmd_chip_i,
    input wire[15:0] gpio_pins,
    output wire [7:0] dac_state_o,
    input wire sample_extra_en_i,
    input wire extra_cmd_en_i,
    input wire[31:0] initial_board_sample_index_i,
    input wire reset_board_sample_index_i
    );

    parameter NUM_CHIPS = 32;
    parameter FORCE_ACQUISITION = 1;
    parameter SIMULATION = 0;
    parameter MOSI_ON_FALLING = 1;
    parameter MOSI_ON_NEXT_RISING = 1;
    parameter SIGNED_ADC_VALUES = 1;


    reg[2:0] state = STATE_WAIT;
    reg data_ready = 0;

    assign data_ready_o = data_ready;

    reg[4:0] chip_index = 0;
    reg[5:0] last_slot = 0;

    reg reset_board_sample_index = 0;

    parameter STATE_WAIT = 0;
    parameter STATE_LOOP_READ = 1;
    parameter STATE_LOOP_WAIT = 2;

    wire bram_flip;
    assign bram_flip = board_sample_index_o[0];
    reg bram_in_we = 1'b0;
    reg[10:0] bram_in_addr = 11'd0;
    reg[15:0] bram_in_data = 16'd0;

    wire bram0_in_we;
    assign bram0_in_we = bram_flip && bram_in_we;
    wire bram0_out_re;
    assign bram0_out_re = (!bram_flip) && data_en_i;
    wire[31:0] bram0_out_data;
    daq_bram inst_daq_bram0(
        .clka(clk_i),
        .wea(bram0_in_we),
        .addra(bram_in_addr),
        .dina(bram_in_data),
        .clkb(data_clk_i),
        .enb(bram0_out_re),
        .addrb(data_addr_i),
        .doutb(bram0_out_data)
        );
    wire bram1_in_we;
    assign bram1_in_we = (!bram_flip) && bram_in_we;
    wire bram1_out_re;
    assign bram1_out_re = bram_flip && data_en_i;
    wire[31:0] bram1_out_data;
    daq_bram inst_daq_bram1(
        .clka(clk_i),
        .wea(bram1_in_we),
        .addra(bram_in_addr),
        .dina(bram_in_data),
        .clkb(data_clk_i),
        .enb(bram1_out_re),
        .addrb(data_addr_i),
        .doutb(bram1_out_data)
        );
    assign data_o = bram_flip ? bram1_out_data : bram0_out_data;

    // DAC waveform from a sine wave lookup table
    // in general, dac_waveform_index = ((board_sample_index_o + 8'd2) % 30)
    wire [7:0] dac_waveform_value;
    reg [4:0] dac_waveform_index = 5'd1;
    sine_table inst_sine_table (
        .clk_i(clk_i),
        // take the "next" sample index, modulo 30, as the sine index
        .addr_i(dac_waveform_index),
        .value_o(dac_waveform_value)
        );
    reg [3:0] sample_index_mod15 = 4'd1;

    assign dac_state_o = 8'd0; // TODO: remember global DAC state

    wire[NUM_CHIPS-1:0] spi_mosi;
    wire[NUM_CHIPS-1:0] spi_sclk;
    wire[NUM_CHIPS-1:0] spi_csbar;
    wire[NUM_CHIPS-1:0] spi_ready;
    wire spi_all_ready, spi_any_ready;
    //assign spi_mosi_o = !(~spi_mosi);   // AND of all bits
    //assign spi_sclk_o = !(~spi_sclk);
    //assign spi_csbar_o = !(~spi_csbar);
    assign spi_mosi_o = spi_mosi[3];    // just use chip #3
    assign spi_sclk_o = spi_sclk[3];    // just use chip #3
    assign spi_csbar_o = spi_csbar[3];  // just use chip #3
    //assign spi_all_ready = !(~spi_ready); // AND of all bits
    assign spi_any_ready = !(!(spi_ready));   // OR of all bits
    assign spi_ready_o = spi_ready;

    wire[15:0] spi_reply [0:NUM_CHIPS-1];
    wire[5:0] spi_slot;
    wire[7:0] chip_sample_index;

    reg[(NUM_CHIPS-1):0] extra_cmd_en = 0;

    // this genvar block instantiates 32x rhd2000_driver modules
    genvar CHIP_INST;
    generate
        for (CHIP_INST = 0; CHIP_INST < NUM_CHIPS; CHIP_INST = CHIP_INST + 1) begin : drivers
            if (CHIP_INST == 3)
                // the first driver sets the chip_sample_index and spi_slot number
                rhd2000_driver #(
                    .SIMULATION(SIMULATION),
                    .FORCE_ACQUISITION(FORCE_ACQUISITION),
                    .MOSI_ON_FALLING(MOSI_ON_FALLING),
                    .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING),
                    .SIGNED_ADC_VALUES(SIGNED_ADC_VALUES)
                ) inst_rhd2000_driver (
                    .clk_i(clk_i),
                    .reset_i(reset_i),
                    .ready_o(spi_ready[CHIP_INST]),
                    .reply_o(spi_reply[CHIP_INST]),
                    .chip_sample_index_o(chip_sample_index),
                    .slot_o(spi_slot),
                    .mosi_o(spi_mosi[CHIP_INST]),
                    .sclk_o(spi_sclk[CHIP_INST]),
                    .csbar_o(spi_csbar[CHIP_INST]),
                    .miso_i(spi_miso_i[CHIP_INST]),
                    .dac_waveform_value(dac_waveform_value),
                    .sample_index_mod15_i(sample_index_mod15),
                    .sample_extra_en_i(sample_extra_en_i),
                    .extra_cmd_i(extra_cmd_i),
                    .extra_cmd_en_i(extra_cmd_en[CHIP_INST])
                    );
            else
                rhd2000_driver #(
                    .SIMULATION(SIMULATION),
                    .FORCE_ACQUISITION(FORCE_ACQUISITION),
                    .MOSI_ON_FALLING(MOSI_ON_FALLING),
                    .MOSI_ON_NEXT_RISING(MOSI_ON_NEXT_RISING),
                    .SIGNED_ADC_VALUES(SIGNED_ADC_VALUES)
                ) inst_rhd2000_driver (
                    .clk_i(clk_i),
                    .reset_i(reset_i),
                    .ready_o(spi_ready[CHIP_INST]),
                    .reply_o(spi_reply[CHIP_INST]),
                    .chip_sample_index_o(), // unconnected
                    .slot_o(),  // unconnected
                    .mosi_o(spi_mosi[CHIP_INST]),
                    .sclk_o(spi_sclk[CHIP_INST]),
                    .csbar_o(spi_csbar[CHIP_INST]),
                    .miso_i(spi_miso_i[CHIP_INST]),
                    .dac_waveform_value(dac_waveform_value),
                    .sample_extra_en_i(sample_extra_en_i),
                    .sample_index_mod15_i(sample_index_mod15),
                    .extra_cmd_i(extra_cmd_i),
                    .extra_cmd_en_i(extra_cmd_en[CHIP_INST])
                    );
        end
    endgenerate

    always @(posedge clk_i) begin
        if (reset_i) begin
            state <= STATE_WAIT;
            bram_in_we <= 1'b0;
            data_ready <= 1'b0;
            chip_index <= 5'd0;
            last_slot <= 6'd0;
            board_sample_index_o <= 32'd0;
            dac_waveform_index <= 5'd1;
            sample_index_mod15 <= 4'd1;
            extra_cmd_en <= 0;
            reset_board_sample_index <= 0;
        end else begin
            // ------- board sample reset stuff ---------
            if (reset_board_sample_index_i && !reset_board_sample_index) begin
                // is the index already set correctly?
                if (initial_board_sample_index_i != board_sample_index_o) begin
                    // need to reset index as soon as possible
                    reset_board_sample_index <= 1'b1;
                end
            end
            case (state)
            STATE_WAIT: begin
                if (enable_acq && (spi_any_ready || FORCE_ACQUISITION)) begin
                    state <= STATE_LOOP_READ;
                    chip_index <= 5'd0;
                end
            end
            STATE_LOOP_READ: begin
                if (enable_acq && (spi_any_ready || FORCE_ACQUISITION)) begin
                    if (chip_sample_index > 32'd1) begin
                        // did we write yet?
                        data_ready <= 1'b1;
                    end
                    // enable/disable the extra command during slot 0 so it will be
                    // available during slot 1
                    if (spi_slot == 0) begin
                        // begining of a board sample
                        if ((extra_cmd_chip_i[5] == 1'b1) || (extra_cmd_chip_i[4:0] == chip_index[4:0])) begin
                            extra_cmd_en[chip_index] <= extra_cmd_en_i;
                        end else begin
                            extra_cmd_en[chip_index] <= 1'b0;
                        end
                    end
                    if (chip_index == (NUM_CHIPS-1)) begin
                        // this is the last chip for this spi_slot
                        state <= STATE_LOOP_WAIT;
                        if (spi_slot == 34) begin
                            // also, end of board sample
                            if (dac_waveform_index >= 5'd29) begin
                                dac_waveform_index <= 5'd0;
                            end else begin
                                dac_waveform_index <= dac_waveform_index + 5'd1;
                            end
                            if (sample_index_mod15 >= 4'd14) begin
                                sample_index_mod15 <= 4'd0;
                            end else begin
                                sample_index_mod15 <= sample_index_mod15 + 4'd1;
                            end
                            // XXX: this code *should* check for sample
                            // indexes modulo 30, but does not because the
                            // division is too slow (~14MHz max).
                            // For now it is the responsibility of the user to
                            // only ever change the board sample number to
                            // something modulo 30.
                            // Else data will be corrupt.
                            // Bad.
                            if (reset_board_sample_index && (dac_waveform_index == 5'd0)) begin
                                board_sample_index_o <= initial_board_sample_index_i;
                                reset_board_sample_index <= 1'b0;
                            end else begin
                                board_sample_index_o <= board_sample_index_o + 32'd1;
                            end
                        end
                    end
                    last_slot <= spi_slot;
                    chip_index <= chip_index + 5'd1;
                    bram_in_we <= 1'b1;
                    if (spi_slot == 33 && sample_index_mod15 == 4'd11) begin
                        // Insert GPIO pins in "extra Slot 1, Index 11"
                        bram_in_data <= gpio_pins;
                    end else begin
                        bram_in_data <= spi_reply[chip_index];
                    end
                    case (spi_slot)
                    32: begin// Extra slot 0
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + spi_slot - 11'd32;
                    end
                    33: begin // Extra slot 1
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + spi_slot - 11'd32;
                    end
                    34: begin // Extra slot 2
                        bram_in_addr <= 11'd1024 + (11'd3 * chip_index) + spi_slot - 11'd32;
                    end
                    default: begin // regular channel slot (0 through 31)
                        bram_in_addr <= (11'd32 * chip_index) + spi_slot;
                    end
                    endcase
                end else begin
                    bram_in_we <= 1'b0;
                    state <= STATE_WAIT;
                end
            end
            STATE_LOOP_WAIT: begin
                if (enable_acq && (spi_any_ready || FORCE_ACQUISITION)) begin
                    bram_in_we <= 1'b0;
                    if (spi_slot != last_slot) begin
                        // new data must be ready
                        state <= STATE_LOOP_READ;
                        chip_index <= 5'd0;
                    end
                end else begin
                    bram_in_we <= 1'b0;
                    state <= STATE_WAIT;
                end
            end
            default:
                state <= STATE_WAIT;
        endcase
        end
    end
endmodule

