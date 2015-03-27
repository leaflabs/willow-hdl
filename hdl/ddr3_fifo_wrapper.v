`timescale 1ns/1ps


/*
* Log:
  * 08/13/2013: Initial Commit
*/

/*
* Description:
  * The top module demonstrates the DDR3 FIFO wrapper for the Spartan 6 MCB
  * This module also is used to verify functionality of the DDR3 FIFO
*
*/

module ddr3_fifo_wrapper #(parameter IS_SIMULATION = "FALSE")
(
    input         clk_50,
    input         reset,
    output        calib_done,

    input         wr_clk,
    input         wr_stb,
    input [31:0]  wr_data,
    output        wr_full,
    output        wr_empty,
    output [6:0]  wr_count,

    input         rd_clk,
    input         rd_stb,
    output [31:0] rd_data,
    output        rd_empty,
    output        rd_full,
    output [6:0]  rd_count,

    inout [7:0]   mcb5_dram_dq,
    output [13:0] mcb5_dram_a,
    output [2:0]  mcb5_dram_ba,
    output        mcb5_dram_ras_n,
    output        mcb5_dram_cas_n,
    output        mcb5_dram_we_n,
    output        mcb5_dram_odt,
    output        mcb5_dram_reset_n,
    output        mcb5_dram_cke,
    output        mcb5_dram_dm,
    inout         mcb5_rzq,
    inout         mcb5_zio,
    inout         mcb5_dram_dqs,
    inout         mcb5_dram_dqs_n,
    output        mcb5_dram_ck,
    output        mcb5_dram_ck_n
    );

wire           reset_n;
assign reset_n = ~reset; // ddr3 expects active low reset basically everywhere.

localparam C5_MEMCLK_PERIOD   = 3000;
localparam C5_INCLK_PERIOD    = ((C5_MEMCLK_PERIOD * C5_CLKFBOUT_MULT) / (C5_DIVCLK_DIVIDE * C5_CLKOUT0_DIVIDE * 2));
localparam C5_RST_ACT_LOW     = 1;
                                         // # = 1 for active low reset,
                                         // # = 0 for active high reset.
localparam C5_INPUT_CLK_TYPE  = "SINGLE_ENDED";
                                         // input clock type DIFFERENTIAL or SINGLE_ENDED
localparam C5_CLKOUT0_DIVIDE  = 1;
localparam C5_CLKOUT1_DIVIDE  = 1;
localparam C5_CLKOUT2_DIVIDE  = 16;
localparam C5_CLKOUT3_DIVIDE  = 8;
localparam C5_CLKFBOUT_MULT   = 12;
localparam C5_DIVCLK_DIVIDE   = 1;


wire                                memc_error;
wire                                ddr3_clk;
wire                                memc_rst0;
wire                                ddr3_rst;
wire                                memc_sysclk_2x;
wire                                memc_sysclk_2x_180;
wire                                memc_pll_ce_0;
wire                                memc_pll_ce_90;
wire                                memc_pll_lock;
wire                                memc_mcb_drp_clk;

//Disable the differential clock inputs
//assign  memc_sys_clk_p               = 1'b0;
//assign  memc_sys_clk_n               = 1'b0;



// Infrastructure-5 instantiation
infrastructure #(
    .C_INCLK_PERIOD                  (C5_INCLK_PERIOD),
    .C_RST_ACT_LOW                   (C5_RST_ACT_LOW),
    .C_INPUT_CLK_TYPE                (C5_INPUT_CLK_TYPE),
    .C_CLKOUT0_DIVIDE                (C5_CLKOUT0_DIVIDE),
    .C_CLKOUT1_DIVIDE                (C5_CLKOUT1_DIVIDE),
    .C_CLKOUT2_DIVIDE                (C5_CLKOUT2_DIVIDE),
    .C_CLKOUT3_DIVIDE                (C5_CLKOUT3_DIVIDE),
    .C_CLKFBOUT_MULT                 (C5_CLKFBOUT_MULT),
    .C_DIVCLK_DIVIDE                 (C5_DIVCLK_DIVIDE)
) inf (
    .sys_clk                         (clk_50),    // 50MHz, but be bufG'ed before hand!
    .sys_rst_i                       (reset_n),
    .clk0                            (ddr3_clk),       // [output] user clock which determines the operating frequency of user interface ports
    .rst0                            (memc_rst0),
    .async_rst                       (ddr3_rst),
    .sysclk_2x                       (memc_sysclk_2x),
    .sysclk_2x_180                   (memc_sysclk_2x_180),
    .pll_ce_0                        (memc_pll_ce_0),
    .pll_ce_90                       (memc_pll_ce_90),
    .pll_lock                        (memc_pll_lock),
    .mcb_drp_clk                     (memc_mcb_drp_clk)
);

ddr3_fifo #(
  .C5_MEMCLK_PERIOD                 (C5_MEMCLK_PERIOD),
  .C5_SIMULATION                    (IS_SIMULATION)
) ddr3_fifo_i
(
  .wr_clk                          (wr_clk),
  .wr_stb                          (wr_stb),
  .wr_data                         (wr_data),
  .wr_full                         (wr_full),
  .wr_empty                        (wr_empty),
  .wr_count                        (wr_count),

  .wr_underrun                     (),
  .wr_error                        (),

  .rd_clk                          (rd_clk),
  .rd_stb                          (rd_stb),
  .rd_data                         (rd_data),
  .rd_full                         (rd_full),
  .rd_empty                        (rd_empty),
  .rd_count                        (rd_count),

  .fifo_full                       (),
  .fifo_empty                      (),

  //DDR3 Memory Clock
  .ddr3_rst                        (ddr3_rst),
  .ddr3_clk                        (ddr3_clk),
  .memc_sysclk_2x                  (memc_sysclk_2x),
  .memc_sysclk_2x_180              (memc_sysclk_2x_180),
  .memc_pll_ce_0                   (memc_pll_ce_0),
  .memc_pll_ce_90                  (memc_pll_ce_90),
  .memc_pll_lock                   (memc_pll_lock),
  .memc_mcb_drp_clk                (memc_mcb_drp_clk),


  .mcb5_dram_dq                    (mcb5_dram_dq),
  .mcb5_dram_a                     (mcb5_dram_a),
  .mcb5_dram_ba                    (mcb5_dram_ba),
  .mcb5_dram_ras_n                 (mcb5_dram_ras_n),
  .mcb5_dram_cas_n                 (mcb5_dram_cas_n),
  .mcb5_dram_we_n                  (mcb5_dram_we_n),
  .mcb5_dram_odt                   (mcb5_dram_odt),
  .mcb5_dram_reset_n               (mcb5_dram_reset_n),
  .mcb5_dram_cke                   (mcb5_dram_cke),
  .mcb5_dram_dm                    (mcb5_dram_dm),
  .mcb5_rzq                        (mcb5_rzq),
  .mcb5_zio                        (mcb5_zio),
  .mcb5_dram_dqs                   (mcb5_dram_dqs),
  .mcb5_dram_dqs_n                 (mcb5_dram_dqs_n),
  .mcb5_dram_ck                    (mcb5_dram_ck),
  .mcb5_dram_ck_n                  (mcb5_dram_ck_n),
  .calibration_done                (calib_done),

  .chipscope_control               ()
);


endmodule
