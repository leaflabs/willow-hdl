`timescale 1ns/1ps

/*
 * Log:
 * 08/13/2013: Initial Commit
 */

/*
 * Author: ???
 *
 * Description:
 *
 * The DDR3 FIFO is a construct that wraps around the DDR3 Memory
 * controller and presents a FIFO interface to the user. The user writes
 * data into the FIFO in the same way they would write data into a normal
 * FIFO, the DDR3 controller will manage memory reads and writes in the
 * background.
 *
 */

//`define ENABLE_CHIPSCOPE 1

`define RDWR_FIFO_SIZE 7
`define BURST_LEN_MAX 32
`define DDR3_FIFO_SIZE 28


`define RDWR_FIFO_MAX ((2** `RDWR_FIFO_SIZE) - 1)
// we almost certainly dont need more than a few MB for this purpose. However, we are not using the memory for anything else. 64MB allows us a full second of drive pause.
`define DDR3_FIFO_MAX_ADDR ((2**24) - 1)
`define DDR3_FIFO_LEN (`DDR3_FIFO_MAX_ADDR + 1)

`define FULL_THRESHOLD (2 * `RDWR_FIFO_MAX)

module ddr3_fifo # (
                    parameter C5_P0_MASK_SIZE           = 4,
                    parameter C5_P0_DATA_PORT_SIZE      = 32,
                    parameter C5_P1_MASK_SIZE           = 4,
                    parameter C5_P1_DATA_PORT_SIZE      = 32,
                    parameter DEBUG_EN                  = 1,
                    // # = 1, Enable debug signals/controls,
                    //   = 0, Disable debug signals/controls.
                    parameter C5_MEMCLK_PERIOD          = 3000,
                    // Memory data transfer clock period
                    parameter C5_CALIB_SOFT_IP          = "TRUE",
                    // # = TRUE, Enables the soft calibration logic,
                    // # = FALSE, Disables the soft calibration logic.
                    parameter C5_SIMULATION             = "FALSE",
                    // # = TRUE, Simulating the design. Useful to reduce the simulation time,
                    // # = FALSE, Implementing the design.
                    parameter C5_HW_TESTING             = "FALSE",
                    // Determines the address space accessed by the traffic generator,
                    // # = FALSE, Smaller address space,
                    // # = TRUE, Large address space.
                    parameter C5_MEM_ADDR_ORDER         = "ROW_BANK_COLUMN",
                    // The order in which user address is provided to the memory controller,
                    // ROW_BANK_COLUMN or BANK_ROW_COLUMN
                    parameter C5_NUM_DQ_PINS            = 8,
                    // External memory data width
                    parameter C5_MEM_ADDR_WIDTH         = 14,
                    // External memory address width
                    parameter C5_MEM_BANKADDR_WIDTH     = 3
                    // External memory bank address width

                    )(
                      //Write Side
                      input                              wr_clk,
                      input                              wr_stb,
                      input [31:0]                       wr_data,
                      output                             wr_full,
                      output                             wr_empty,
                      output [`RDWR_FIFO_SIZE - 1:0]     wr_count,



                      output                             wr_underrun,
                      output                             wr_error,

                      //Read Side
                      input                              rd_clk,
                      input                              rd_stb,
                      output [31:0]                      rd_data,
                      output                             rd_full,
                      output                             rd_empty,
                      output [`RDWR_FIFO_SIZE - 1:0]     rd_count,

                      output                             rd_overflow,
                      output                             rd_error,

                      output                             fifo_full,
                      output                             fifo_empty,



                      input                              ddr3_clk,
                      input                              ddr3_rst,
                      input                              memc_sysclk_2x,
                      input                              memc_sysclk_2x_180,
                      input                              memc_pll_ce_0,
                      input                              memc_pll_ce_90,
                      input                              memc_pll_lock,
                      input                              memc_mcb_drp_clk,



                      inout [C5_NUM_DQ_PINS-1:0]         mcb5_dram_dq,
                      output [C5_MEM_ADDR_WIDTH-1:0]     mcb5_dram_a,
                      output [C5_MEM_BANKADDR_WIDTH-1:0] mcb5_dram_ba,
                      output                             mcb5_dram_ras_n,
                      output                             mcb5_dram_cas_n,
                      output                             mcb5_dram_we_n,
                      output                             mcb5_dram_odt,
                      output                             mcb5_dram_reset_n,
                      output                             mcb5_dram_cke,
                      output                             mcb5_dram_dm,
                      inout                              mcb5_rzq,
                      inout                              mcb5_zio,
                      inout                              mcb5_dram_dqs,
                      inout                              mcb5_dram_dqs_n,
                      output                             mcb5_dram_ck,
                      output                             mcb5_dram_ck_n,

                      output                             calibration_done,

                      inout [35:0]                       chipscope_control


                      );

   //Local Parameters

   // The parameter CX_PORT_ENABLE shows all the active user ports in the design.
   // For example, the value 6'b111100 tells that only port-2, port-3, port-4
   // and port-5 are enabled. The other two ports are inactive. An inactive port
   // can be a disabled port or an invisible logical port. Few examples to the
   // invisible logical port are port-4 and port-5 in the user port configuration,
   // Config-2: Four 32-bit bi-directional ports and the ports port-2 through
   // port-5 in Config-4: Two 64-bit bi-directional ports. Please look into the
   // Chapter-2 of ug388.pdf in the /docs directory for further details.
   localparam C5_PORT_ENABLE              = 6'b001100;
   localparam C5_PORT_CONFIG              = "B32_B32_W32_R32_R32_R32";
   localparam C5_P0_PORT_MODE             = "NONE";
   localparam C5_P1_PORT_MODE             = "NONE";
   localparam C5_P2_PORT_MODE             = "WR_MODE";
   localparam C5_P3_PORT_MODE             = "RD_MODE";
   localparam C5_P4_PORT_MODE             = "NONE";
   localparam C5_P5_PORT_MODE             = "NONE";
   localparam C5_ARB_ALGORITHM            = 0;
   localparam C5_ARB_NUM_TIME_SLOTS       = 12;
   localparam C5_ARB_TIME_SLOT_0          = 6'o23;
   localparam C5_ARB_TIME_SLOT_1          = 6'o32;
   localparam C5_ARB_TIME_SLOT_2          = 6'o23;
   localparam C5_ARB_TIME_SLOT_3          = 6'o32;
   localparam C5_ARB_TIME_SLOT_4          = 6'o23;
   localparam C5_ARB_TIME_SLOT_5          = 6'o32;
   localparam C5_ARB_TIME_SLOT_6          = 6'o23;
   localparam C5_ARB_TIME_SLOT_7          = 6'o32;
   localparam C5_ARB_TIME_SLOT_8          = 6'o23;
   localparam C5_ARB_TIME_SLOT_9          = 6'o32;
   localparam C5_ARB_TIME_SLOT_10         = 6'o23;
   localparam C5_ARB_TIME_SLOT_11         = 6'o32;
   localparam C5_MEM_TRAS                 = 36000;
   localparam C5_MEM_TRCD                 = 13500;
   localparam C5_MEM_TREFI                = 7800000;
   localparam C5_MEM_TRFC                 = 110000;
   localparam C5_MEM_TRP                  = 13500;
   localparam C5_MEM_TWR                  = 15000;
   localparam C5_MEM_TRTP                 = 7500;
   localparam C5_MEM_TWTR                 = 7500;
   localparam C5_MEM_TYPE                 = "DDR3";
   localparam C5_MEM_DENSITY              = "1Gb";
   localparam C5_MEM_BURST_LEN            = 8;
   localparam C5_MEM_CAS_LATENCY          = 6;
   localparam C5_MEM_NUM_COL_BITS         = 10;
   localparam C5_MEM_DDR1_2_ODS           = "FULL";
   localparam C5_MEM_DDR2_RTT             = "150OHMS";
   localparam C5_MEM_DDR2_DIFF_DQS_EN     = "YES";
   localparam C5_MEM_DDR2_3_PA_SR         = "FULL";
   localparam C5_MEM_DDR2_3_HIGH_TEMP_SR  = "NORMAL";
   localparam C5_MEM_DDR3_CAS_LATENCY     = 6;
   localparam C5_MEM_DDR3_ODS             = "DIV6";
   localparam C5_MEM_DDR3_RTT             = "DIV4";
   localparam C5_MEM_DDR3_CAS_WR_LATENCY  = 5;
   localparam C5_MEM_DDR3_AUTO_SR         = "ENABLED";
   localparam C5_MEM_MOBILE_PA_SR         = "FULL";
   localparam C5_MEM_MDDR_ODS             = "FULL";
   localparam C5_MC_CALIB_BYPASS          = "NO";
   localparam C5_MC_CALIBRATION_MODE      = "CALIBRATION";
   localparam C5_MC_CALIBRATION_DELAY     = "HALF";
   localparam C5_SKIP_IN_TERM_CAL         = 0;
   localparam C5_SKIP_DYNAMIC_CAL         = 0;
   localparam C5_LDQSP_TAP_DELAY_VAL      = 0;
   localparam C5_LDQSN_TAP_DELAY_VAL      = 0;
   localparam C5_UDQSP_TAP_DELAY_VAL      = 0;
   localparam C5_UDQSN_TAP_DELAY_VAL      = 0;
   localparam C5_DQ0_TAP_DELAY_VAL        = 0;
   localparam C5_DQ1_TAP_DELAY_VAL        = 0;
   localparam C5_DQ2_TAP_DELAY_VAL        = 0;
   localparam C5_DQ3_TAP_DELAY_VAL        = 0;
   localparam C5_DQ4_TAP_DELAY_VAL        = 0;
   localparam C5_DQ5_TAP_DELAY_VAL        = 0;
   localparam C5_DQ6_TAP_DELAY_VAL        = 0;
   localparam C5_DQ7_TAP_DELAY_VAL        = 0;
   localparam C5_DQ8_TAP_DELAY_VAL        = 0;
   localparam C5_DQ9_TAP_DELAY_VAL        = 0;
   localparam C5_DQ10_TAP_DELAY_VAL       = 0;
   localparam C5_DQ11_TAP_DELAY_VAL       = 0;
   localparam C5_DQ12_TAP_DELAY_VAL       = 0;
   localparam C5_DQ13_TAP_DELAY_VAL       = 0;
   localparam C5_DQ14_TAP_DELAY_VAL       = 0;
   localparam C5_DQ15_TAP_DELAY_VAL       = 0;
   localparam C5_MCB_USE_EXTERNAL_BUFPLL  = 1;
   localparam C5_SMALL_DEVICE             = "FALSE";       // The parameter is set to TRUE for all packages of xc6slx9 device
   // as most of them cannot fit the complete example design when the
   // Chip scope modules are enabled
   localparam C5_p0_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p0_DATA_MODE                       = 4'b0010;
   localparam C5_p0_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h02ffffff:32'h000002ff;
   localparam C5_p0_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hfc000000:32'hfffffc00;
   localparam C5_p0_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p1_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h03000000:32'h00000300;
   localparam C5_p1_DATA_MODE                       = 4'b0010;
   localparam C5_p1_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h04ffffff:32'h000004ff;
   localparam C5_p1_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hf8000000:32'hfffff800;
   localparam C5_p1_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h03000000:32'h00000300;
   localparam C5_p2_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h05000000:32'h00000500;
   localparam C5_p2_DATA_MODE                       = 4'b0010;
   localparam C5_p2_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h06ffffff:32'h000006ff;
   localparam C5_p2_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hf8000000:32'hfffff800;
   localparam C5_p2_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h05000000:32'h00000500;
   localparam C5_p3_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p3_DATA_MODE                       = 4'b0010;
   localparam C5_p3_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h02ffffff:32'h000002ff;
   localparam C5_p3_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hfc000000:32'hfffffc00;
   localparam C5_p3_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p4_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p4_DATA_MODE                       = 4'b0010;
   localparam C5_p4_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h02ffffff:32'h000002ff;
   localparam C5_p4_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hfc000000:32'hfffffc00;
   localparam C5_p4_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p5_BEGIN_ADDRESS                   = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam C5_p5_DATA_MODE                       = 4'b0010;
   localparam C5_p5_END_ADDRESS                     = (C5_HW_TESTING == "TRUE") ? 32'h02ffffff:32'h000002ff;
   localparam C5_p5_PRBS_EADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'hfc000000:32'hfffffc00;
   localparam C5_p5_PRBS_SADDR_MASK_POS             = (C5_HW_TESTING == "TRUE") ? 32'h01000000:32'h00000100;
   localparam DBG_WR_STS_WIDTH                      = 32;
   localparam DBG_RD_STS_WIDTH                      = 32;
   localparam C5_ARB_TIME0_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_0[5:3], C5_ARB_TIME_SLOT_0[2:0]};
   localparam C5_ARB_TIME1_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_1[5:3], C5_ARB_TIME_SLOT_1[2:0]};
   localparam C5_ARB_TIME2_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_2[5:3], C5_ARB_TIME_SLOT_2[2:0]};
   localparam C5_ARB_TIME3_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_3[5:3], C5_ARB_TIME_SLOT_3[2:0]};
   localparam C5_ARB_TIME4_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_4[5:3], C5_ARB_TIME_SLOT_4[2:0]};
   localparam C5_ARB_TIME5_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_5[5:3], C5_ARB_TIME_SLOT_5[2:0]};
   localparam C5_ARB_TIME6_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_6[5:3], C5_ARB_TIME_SLOT_6[2:0]};
   localparam C5_ARB_TIME7_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_7[5:3], C5_ARB_TIME_SLOT_7[2:0]};
   localparam C5_ARB_TIME8_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_8[5:3], C5_ARB_TIME_SLOT_8[2:0]};
   localparam C5_ARB_TIME9_SLOT                     = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_9[5:3], C5_ARB_TIME_SLOT_9[2:0]};
   localparam C5_ARB_TIME10_SLOT                    = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_10[5:3], C5_ARB_TIME_SLOT_10[2:0]};
   localparam C5_ARB_TIME11_SLOT                    = {3'b000, 3'b000, 3'b000, 3'b000, C5_ARB_TIME_SLOT_11[5:3], C5_ARB_TIME_SLOT_11[2:0]};

   localparam                          IDLE  = 4'h0;
   localparam                          WRITE = 4'h1;
   localparam                          READ  = 4'h1;

   //Registers/Wires

   //Write Side Registers/Wires
   wire                                                  cc_fifo_full;
   reg [`DDR3_FIFO_SIZE    :0]                           ddr3_fifo_max = 0;
   reg [`DDR3_FIFO_SIZE - 1:0]                           wr_mem_addr = 0;
   reg [`RDWR_FIFO_SIZE - 1:0]                           wr_usr_count = 0;
   wire [29:0]                                           wr_addr;
   wire                                                  wr_full_raw;
   assign wr_full = wr_empty ? 1'b0 : wr_full_raw;



   //DDR3 Write Path Controls
   reg [`RDWR_FIFO_SIZE - 2:0]                           wr_cmd_bl = 0;
   reg                                                   wr_cmd_stb = 0;
   wire                                                  wr_cmd_full;
   wire                                                  wr_cmd_empty;

   //Read Side Registers/Wires
   wire [`DDR3_FIFO_SIZE - 1:0]                          cc_wr_mem_addr;
   reg [`DDR3_FIFO_SIZE - 1:0]                           rd_mem_addr = 0;

   //reg         [`RDWR_FIFO_SIZE - 1:0] rd_usr_count;
   wire [`RDWR_FIFO_SIZE - 1:0]                          rd_fifo_avail;
   wire [`RDWR_FIFO_SIZE - 1:0]                          rd_fifo_request;

   wire [`DDR3_FIFO_SIZE - 1:0]                          ddr3_fifo_avail;
   reg [`DDR3_FIFO_SIZE - 1:0]                           r_ddr3_fifo_avail = 0;
   wire                                                  ddr3_fifo_full;
   reg                                                   r_ddr3_fifo_full = 0;

   //DDR3 Read Path Controls
   wire                                                  rd_cmd_empty;
   wire                                                  rd_cmd_full;
   reg [`RDWR_FIFO_SIZE - 2:0]                           rd_cmd_bl = 0;
   reg                                                   rd_cmd_stb = 0;
   wire [29:0]                                           rd_addr;
   reg [3:0]                                             rd_state = IDLE;
   reg [3:0]                                             rd_wait = 4'b0;


   reg [7:0]                                             write_count = 8'b0;
   reg [3:0]                                             wr_state = IDLE;
   reg [3:0]                                             wr_wait = 4'b0;
   reg [7:0]                                             read_count = 8'b0;

   //Chipscope controller
   wire [255:0]                                          ila_data;

   reg [`DDR3_FIFO_SIZE - 1: 0]                          next_count = 0;



   //Sub Modules
   cross_clock_data #(
                      .DATA_WIDTH         (28               )
                      )ccaddr(
                              .rst                (ddr3_rst         ),

                              .out_clk            (rd_clk           ),
                              .in_data            (wr_mem_addr      ),
                              .out_data           (cc_wr_mem_addr   )
                              );

   cross_clock_enable ccfff(
                            .rst                (ddr3_rst         ),

                            .in_en              (ddr3_fifo_full   ),

                            .out_clk            (wr_clk           ),
                            .out_en             (cc_fifo_full     )
                            );

   // Controller-5 instantiation
   memc_wrapper #(
                  .C_MEMCLK_PERIOD                (C5_MEMCLK_PERIOD),
                  .C_CALIB_SOFT_IP                (C5_CALIB_SOFT_IP),
                  //synthesis translate_off
                  .C_SIMULATION                   (C5_SIMULATION),
                  //synthesis translate_on
                  .C_ARB_NUM_TIME_SLOTS           (C5_ARB_NUM_TIME_SLOTS),
                  .C_ARB_TIME_SLOT_0              (C5_ARB_TIME0_SLOT),
                  .C_ARB_TIME_SLOT_1              (C5_ARB_TIME1_SLOT),
                  .C_ARB_TIME_SLOT_2              (C5_ARB_TIME2_SLOT),
                  .C_ARB_TIME_SLOT_3              (C5_ARB_TIME3_SLOT),
                  .C_ARB_TIME_SLOT_4              (C5_ARB_TIME4_SLOT),
                  .C_ARB_TIME_SLOT_5              (C5_ARB_TIME5_SLOT),
                  .C_ARB_TIME_SLOT_6              (C5_ARB_TIME6_SLOT),
                  .C_ARB_TIME_SLOT_7              (C5_ARB_TIME7_SLOT),
                  .C_ARB_TIME_SLOT_8              (C5_ARB_TIME8_SLOT),
                  .C_ARB_TIME_SLOT_9              (C5_ARB_TIME9_SLOT),
                  .C_ARB_TIME_SLOT_10             (C5_ARB_TIME10_SLOT),
                  .C_ARB_TIME_SLOT_11             (C5_ARB_TIME11_SLOT),
                  .C_ARB_ALGORITHM                (C5_ARB_ALGORITHM),
                  .C_PORT_ENABLE                  (C5_PORT_ENABLE),
                  .C_PORT_CONFIG                  (C5_PORT_CONFIG),
                  .C_MEM_TRAS                     (C5_MEM_TRAS),
                  .C_MEM_TRCD                     (C5_MEM_TRCD),
                  .C_MEM_TREFI                    (C5_MEM_TREFI),
                  .C_MEM_TRFC                     (C5_MEM_TRFC),
                  .C_MEM_TRP                      (C5_MEM_TRP),
                  .C_MEM_TWR                      (C5_MEM_TWR),
                  .C_MEM_TRTP                     (C5_MEM_TRTP),
                  .C_MEM_TWTR                     (C5_MEM_TWTR),
                  .C_MEM_ADDR_ORDER               (C5_MEM_ADDR_ORDER),
                  .C_NUM_DQ_PINS                  (C5_NUM_DQ_PINS),
                  .C_MEM_TYPE                     (C5_MEM_TYPE),
                  .C_MEM_DENSITY                  (C5_MEM_DENSITY),
                  .C_MEM_BURST_LEN                (C5_MEM_BURST_LEN),
                  .C_MEM_CAS_LATENCY              (C5_MEM_CAS_LATENCY),
                  .C_MEM_ADDR_WIDTH               (C5_MEM_ADDR_WIDTH),
                  .C_MEM_BANKADDR_WIDTH           (C5_MEM_BANKADDR_WIDTH),
                  .C_MEM_NUM_COL_BITS             (C5_MEM_NUM_COL_BITS),
                  .C_MEM_DDR1_2_ODS               (C5_MEM_DDR1_2_ODS),
                  .C_MEM_DDR2_RTT                 (C5_MEM_DDR2_RTT),
                  .C_MEM_DDR2_DIFF_DQS_EN         (C5_MEM_DDR2_DIFF_DQS_EN),
                  .C_MEM_DDR2_3_PA_SR             (C5_MEM_DDR2_3_PA_SR),
                  .C_MEM_DDR2_3_HIGH_TEMP_SR      (C5_MEM_DDR2_3_HIGH_TEMP_SR),
                  .C_MEM_DDR3_CAS_LATENCY         (C5_MEM_DDR3_CAS_LATENCY),
                  .C_MEM_DDR3_ODS                 (C5_MEM_DDR3_ODS),
                  .C_MEM_DDR3_RTT                 (C5_MEM_DDR3_RTT),
                  .C_MEM_DDR3_CAS_WR_LATENCY      (C5_MEM_DDR3_CAS_WR_LATENCY),
                  .C_MEM_DDR3_AUTO_SR             (C5_MEM_DDR3_AUTO_SR),
                  .C_MEM_MOBILE_PA_SR             (C5_MEM_MOBILE_PA_SR),
                  .C_MEM_MDDR_ODS                 (C5_MEM_MDDR_ODS),
                  .C_MC_CALIB_BYPASS              (C5_MC_CALIB_BYPASS),
                  .C_MC_CALIBRATION_MODE          (C5_MC_CALIBRATION_MODE),
                  .C_MC_CALIBRATION_DELAY         (C5_MC_CALIBRATION_DELAY),
                  .C_SKIP_IN_TERM_CAL             (C5_SKIP_IN_TERM_CAL),
                  .C_SKIP_DYNAMIC_CAL             (C5_SKIP_DYNAMIC_CAL),
                  .LDQSP_TAP_DELAY_VAL            (C5_LDQSP_TAP_DELAY_VAL),
                  .UDQSP_TAP_DELAY_VAL            (C5_UDQSP_TAP_DELAY_VAL),
                  .LDQSN_TAP_DELAY_VAL            (C5_LDQSN_TAP_DELAY_VAL),
                  .UDQSN_TAP_DELAY_VAL            (C5_UDQSN_TAP_DELAY_VAL),
                  .DQ0_TAP_DELAY_VAL              (C5_DQ0_TAP_DELAY_VAL),
                  .DQ1_TAP_DELAY_VAL              (C5_DQ1_TAP_DELAY_VAL),
                  .DQ2_TAP_DELAY_VAL              (C5_DQ2_TAP_DELAY_VAL),
                  .DQ3_TAP_DELAY_VAL              (C5_DQ3_TAP_DELAY_VAL),
                  .DQ4_TAP_DELAY_VAL              (C5_DQ4_TAP_DELAY_VAL),
                  .DQ5_TAP_DELAY_VAL              (C5_DQ5_TAP_DELAY_VAL),
                  .DQ6_TAP_DELAY_VAL              (C5_DQ6_TAP_DELAY_VAL),
                  .DQ7_TAP_DELAY_VAL              (C5_DQ7_TAP_DELAY_VAL),
                  .DQ8_TAP_DELAY_VAL              (C5_DQ8_TAP_DELAY_VAL),
                  .DQ9_TAP_DELAY_VAL              (C5_DQ9_TAP_DELAY_VAL),
                  .DQ10_TAP_DELAY_VAL             (C5_DQ10_TAP_DELAY_VAL),
                  .DQ11_TAP_DELAY_VAL             (C5_DQ11_TAP_DELAY_VAL),
                  .DQ12_TAP_DELAY_VAL             (C5_DQ12_TAP_DELAY_VAL),
                  .DQ13_TAP_DELAY_VAL             (C5_DQ13_TAP_DELAY_VAL),
                  .DQ14_TAP_DELAY_VAL             (C5_DQ14_TAP_DELAY_VAL),
                  .DQ15_TAP_DELAY_VAL             (C5_DQ15_TAP_DELAY_VAL),
                  .C_P0_MASK_SIZE                 (C5_P0_MASK_SIZE),
                  .C_P0_DATA_PORT_SIZE            (C5_P0_DATA_PORT_SIZE),
                  .C_P1_MASK_SIZE                 (C5_P1_MASK_SIZE),
                  .C_P1_DATA_PORT_SIZE            (C5_P1_DATA_PORT_SIZE)
                  )

   mem_wrapper(
               .mcbx_dram_addr                 (mcb5_dram_a),
               .mcbx_dram_ba                   (mcb5_dram_ba),
               .mcbx_dram_ras_n                (mcb5_dram_ras_n),
               .mcbx_dram_cas_n                (mcb5_dram_cas_n),
               .mcbx_dram_we_n                 (mcb5_dram_we_n),
               .mcbx_dram_cke                  (mcb5_dram_cke),
               .mcbx_dram_clk                  (mcb5_dram_ck),
               .mcbx_dram_clk_n                (mcb5_dram_ck_n),
               .mcbx_dram_dq                   (mcb5_dram_dq),
               .mcbx_dram_dqs                  (mcb5_dram_dqs),
               .mcbx_dram_dqs_n                (mcb5_dram_dqs_n),
               .mcbx_dram_udqs                 (),
               .mcbx_dram_udqs_n               (),
               .mcbx_dram_udm                  (),
               .mcbx_dram_ldm                  (mcb5_dram_dm),
               .mcbx_dram_odt                  (mcb5_dram_odt),
               .mcbx_dram_ddr3_rst             (mcb5_dram_reset_n),
               .mcbx_rzq                       (mcb5_rzq),
               .mcbx_zio                       (mcb5_zio),
               .calib_done                     (calibration_done),

               .async_rst                      (ddr3_rst),
               .sysclk_2x                      (memc_sysclk_2x),
               .sysclk_2x_180                  (memc_sysclk_2x_180),
               .pll_ce_0                       (memc_pll_ce_0),
               .pll_ce_90                      (memc_pll_ce_90),
               .pll_lock                       (memc_pll_lock),
               .mcb_drp_clk                    (memc_mcb_drp_clk),

               // The following port map shows all the six logical user ports. However, all
               // of them may not be active in this design. A port should be enabled to
               // validate its port map. If it is not,the complete port is going to float
               // by getting disconnected from the lower level MCB modules. The port enable
               // information of a controller can be obtained from the corresponding local
               // parameter CX_PORT_ENABLE. In such a case, we can simply ignore its port map.
               // The following comments will explain when a port is going to be active.
               // Config-1: Two 32-bit bi-directional and four 32-bit unidirectional ports
               // Config-2: Four 32-bit bi-directional ports
               // Config-3: One 64-bit bi-directional and two 32-bit bi-directional ports
               // Config-4: Two 64-bit bi-directional ports
               // Config-5: One 128-bit bi-directional port

               // User Port-0 command interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3, Config-4 and Config-5
               .p0_cmd_clk                     (ddr3_clk),
               .p0_cmd_en                      (1'b0),
               .p0_cmd_instr                   (3'b000),
               .p0_cmd_bl                      (6'h0),
               .p0_cmd_byte_addr               (30'h00000000),
               .p0_cmd_full                    (),
               .p0_cmd_empty                   (),
               // User Port-0 data write interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3, Config-4 and Config-5
               .p0_wr_clk                      (ddr3_clk),
               .p0_wr_en                       (1'b0),
               .p0_wr_mask                     (4'b0),
               .p0_wr_data                     (32'h00000000),
               .p0_wr_full                     (),
               .p0_wr_count                    (),
               .p0_wr_empty                    (),
               .p0_wr_underrun                 (),
               .p0_wr_error                    (),
               // User Port-0 data read interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3, Config-4 and Config-5
               .p0_rd_clk                      (ddr3_clk),
               .p0_rd_en                       (1'b0),
               .p0_rd_data                     (),
               .p0_rd_empty                    (),
               .p0_rd_count                    (),
               .p0_rd_full                     (),
               .p0_rd_overflow                 (),
               .p0_rd_error                    (),

               // User Port-1 command interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3 and Config-4
               .p1_cmd_clk                     (ddr3_clk),
               .p1_cmd_en                      (1'b0),
               .p1_cmd_instr                   (3'b000),
               .p1_cmd_bl                      (6'h00),
               .p1_cmd_byte_addr               (30'h00000000),
               .p1_cmd_full                    (),
               .p1_cmd_empty                   (),
               // User Port-1 data write interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3 and Config-4
               .p1_wr_clk                      (ddr3_clk),
               .p1_wr_en                       (1'b0),
               .p1_wr_mask                     (4'b0000),
               .p1_wr_data                     (),
               .p1_wr_full                     (),
               .p1_wr_count                    (),
               .p1_wr_empty                    (),
               .p1_wr_underrun                 (),
               .p1_wr_error                    (),
               // User Port-1 data read interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2, Config-3 and Config-4
               .p1_rd_clk                      (ddr3_clk),
               .p1_rd_en                       (1'b0),
               .p1_rd_data                     (),
               .p1_rd_empty                    (),
               .p1_rd_count                    (),
               .p1_rd_full                     (),
               .p1_rd_overflow                 (),
               .p1_rd_error                    (),

               // User Port-2 command interface will be active only when the port is enabled in
               // the port configurations Config-1, Config-2 and Config-3
               .p2_cmd_clk                     (wr_clk),
               .p2_cmd_en                      (wr_cmd_stb),
               .p2_cmd_instr                   (3'b010),
               .p2_cmd_bl                      (wr_cmd_bl),
               .p2_cmd_byte_addr               (wr_addr),
               .p2_cmd_full                    (wr_cmd_full),
               .p2_cmd_empty                   (wr_cmd_empty),
               // User Port-2 data write interface will be active only when the port is enabled in
               // the port configurations Config-1 write direction, Config-2 and Config-3
               .p2_wr_clk                      (wr_clk),
               .p2_wr_en                       (wr_stb),
               .p2_wr_mask                     (4'b0000),
               .p2_wr_data                     (wr_data),
               .p2_wr_full                     (wr_full_raw),
               .p2_wr_count                    (wr_count),
               .p2_wr_empty                    (wr_empty),
               .p2_wr_underrun                 (wr_underrun),
               .p2_wr_error                    (wr_error),
               // User Port-2 data read interface will be active only when the port is enabled in
               // the port configurations Config-1 read direction, Config-2 and Config-3
               .p2_rd_clk                      (wr_clk),
               .p2_rd_en                       (1'b0),
               .p2_rd_data                     (),
               .p2_rd_empty                    (),
               .p2_rd_count                    (),
               .p2_rd_full                     (),
               .p2_rd_overflow                 (),
               .p2_rd_error                    (),

               // User Port-3 command interface will be active only when the port is enabled in
               // the port configurations Config-1 and Config-2
               .p3_cmd_clk                     (rd_clk),
               .p3_cmd_en                      (rd_cmd_stb),
               .p3_cmd_instr                   (3'b011),
               .p3_cmd_byte_addr               (rd_addr),
               .p3_cmd_full                    (rd_cmd_full),
               .p3_cmd_empty                   (rd_cmd_empty),
               .p3_cmd_bl                      (rd_cmd_bl),
               // User Port-3 data write interface will be active only when the port is enabled in
               // the port configurations Config-1 write direction and Config-2
               .p3_wr_clk                      (rd_clk),
               .p3_wr_en                       (1'b0),
               .p3_wr_mask                     (4'b0000),
               .p3_wr_data                     (32'h00000000),
               .p3_wr_full                     (),
               .p3_wr_count                    (),
               .p3_wr_empty                    (),
               .p3_wr_underrun                 (),
               .p3_wr_error                    (),
               // User Port-3 data read interface will be active only when the port is enabled in
               // the port configurations Config-1 read direction and Config-2
               .p3_rd_clk                      (rd_clk),
               .p3_rd_en                       (rd_stb),
               .p3_rd_data                     (rd_data),
               .p3_rd_empty                    (rd_empty),
               .p3_rd_count                    (rd_count),
               .p3_rd_full                     (rd_full),
               .p3_rd_overflow                 (rd_overflow),
               .p3_rd_error                    (rd_error),

               // User Port-4 command interface will be active only when the port is enabled in
               // the port configuration Config-1
               .p4_cmd_clk                     (ddr3_clk),
               .p4_cmd_en                      (1'b0),
               .p4_cmd_instr                   (3'b000),
               .p4_cmd_bl                      (6'h00),
               .p4_cmd_byte_addr               (30'h000000),
               .p4_cmd_full                    (),
               .p4_cmd_empty                   (),
               // User Port-4 data write interface will be active only when the port is enabled in
               // the port configuration Config-1 write direction
               .p4_wr_clk                      (ddr3_clk),
               .p4_wr_en                       (1'b0),
               .p4_wr_mask                     (4'b0000),
               .p4_wr_data                     (32'h0000000),
               .p4_wr_full                     (),
               .p4_wr_count                    (),
               .p4_wr_empty                    (),
               .p4_wr_underrun                 (),
               .p4_wr_error                    (),
               // User Port-4 data read interface will be active only when the port is enabled in
               // the port configuration Config-1 read direction
               .p4_rd_clk                      (ddr3_clk),
               .p4_rd_en                       (1'b0),
               .p4_rd_data                     (),
               .p4_rd_empty                    (),
               .p4_rd_count                    (),
               .p4_rd_full                     (),
               .p4_rd_overflow                 (),
               .p4_rd_error                    (),

               // User Port-5 command interface will be active only when the port is enabled in
               // the port configuration Config-1
               .p5_cmd_clk                     (ddr3_clk),
               .p5_cmd_en                      (1'b0),
               .p5_cmd_instr                   (3'b000),
               .p5_cmd_bl                      (6'h00),
               .p5_cmd_byte_addr               (30'h00000000),
               .p5_cmd_full                    (),
               .p5_cmd_empty                   (),
               // User Port-5 data write interface will be active only when the port is enabled in
               // the port configuration Config-1 write direction
               .p5_wr_clk                      (ddr3_clk),
               .p5_wr_en                       (1'b0),
               .p5_wr_mask                     (4'b0000),
               .p5_wr_data                     (32'h00000000),
               .p5_wr_full                     (),
               .p5_wr_count                    (),
               .p5_wr_empty                    (),
               .p5_wr_underrun                 (),
               .p5_wr_error                    (),
               // User Port-5 data read interface will be active only when the port is enabled in
               // the port configuration Config-1 read direction
               .p5_rd_clk                      (ddr3_clk),
               .p5_rd_en                       (1'b0),
               .p5_rd_data                     (),
               .p5_rd_empty                    (),
               .p5_rd_count                    (),
               .p5_rd_full                     (),
               .p5_rd_overflow                 (),
               .p5_rd_error                    (),

               .selfrefresh_enter              (1'b0),
               .selfrefresh_mode               ()
               );


   //Asynchronous Logc
   assign  wr_addr         = {wr_mem_addr, 2'b00};
   assign  rd_addr         = {rd_mem_addr, 2'b00};
  //assign  `DDR3_FIFO_MAX_ADDR  = 28'h`DDR3_FIFO_MAX_ADDR;

   //Normal:Wrap Around:Empty
   assign  ddr3_fifo_avail = (cc_wr_mem_addr > rd_mem_addr) ? cc_wr_mem_addr - rd_mem_addr  :
                             (cc_wr_mem_addr < rd_mem_addr) ? (`DDR3_FIFO_MAX_ADDR - rd_mem_addr + cc_wr_mem_addr + 28'd1) :
                             0;

   //A little bit of a fib here but this give the enough of an overhead so that
   //when communicating accros the read/write clock domain the write side has
   //time to react
   assign  ddr3_fifo_full  = (ddr3_fifo_avail >= (`DDR3_FIFO_MAX_ADDR - `FULL_THRESHOLD)) ? 1: 0;

   assign  fifo_full       = ddr3_fifo_full;
   assign  fifo_empty      = (ddr3_fifo_avail == 0);


   assign  ila_data     ={
                          wr_state,
                          wr_mem_addr,
                          r_ddr3_fifo_avail,
                          r_ddr3_fifo_full,
                          fifo_empty,

                          rd_addr,
                          rd_cmd_bl,
                          rd_count,
                          rd_cmd_full,
                          rd_cmd_empty,
                          rd_cmd_stb,

                          rd_stb,
                          rd_data,
                          rd_full,
                          rd_empty,
                          rd_overflow,
                          rd_error,

                          wr_addr,
                          wr_cmd_bl,
                          write_count,
                          wr_count,
                          wr_cmd_full,
                          wr_cmd_empty,
                          wr_cmd_stb,

                          wr_stb,
                          wr_data,
                          wr_full,
                          wr_empty,
                          wr_underrun
                          };

   always @ (posedge wr_clk) begin
      if (ddr3_rst) begin
         wr_mem_addr       <=  0;
         wr_usr_count      <=  0;
         wr_cmd_bl         <=  0;
         write_count       <=  0;
         wr_state          <=  IDLE;
         wr_wait           <=  0;
      ddr3_fifo_max   <=  `DDR3_FIFO_MAX_ADDR;
      end
      else begin
         if (wr_stb) begin
            write_count     <=  write_count + 1;
         end
         //Strobes
         wr_cmd_stb         <=  0;


         case (wr_state)
           IDLE: begin
              if ((write_count > 0) && !wr_empty && wr_cmd_empty && !cc_fifo_full) begin
                 //Tell the command FIFO how many data words to write
                 if (wr_stb) begin
                    if ((write_count + wr_mem_addr) > `DDR3_FIFO_MAX_ADDR) begin
                       wr_cmd_bl   <=  (`DDR3_FIFO_LEN - wr_mem_addr) - 1;
                       wr_usr_count<=  (`DDR3_FIFO_LEN - wr_mem_addr);
                       write_count <= write_count - (`DDR3_FIFO_LEN - wr_mem_addr) + 1;
                    end
                    else begin
                       //If the strobe is enabled then I can't overwrite the + 1 that
                       //occured
                       if (write_count > 63) begin
                          wr_cmd_bl   <=  63;
                          wr_usr_count<=  64;
                          write_count <= write_count - 63;
                       end
                       else begin
                          wr_cmd_bl   <=  write_count;
                          wr_usr_count<=  write_count + 1;
                          write_count     <=  0;
                       end
                    end
                 end
                 else begin
                    if ((write_count + wr_mem_addr) > `DDR3_FIFO_MAX_ADDR) begin
                       wr_cmd_bl   <=  (`DDR3_FIFO_LEN - wr_mem_addr) - 1;
                       wr_usr_count<=  (`DDR3_FIFO_LEN - wr_mem_addr);
                       write_count <= write_count - (`DDR3_FIFO_LEN - wr_mem_addr);
                    end
                    else begin
                       if (write_count > 64) begin
                          //Special case where there is more data in the data FIFO then
                          //there is room in the 'burst length command"
                          wr_cmd_bl     <=  63;
                          wr_usr_count  <=  64;
                          write_count   <=  write_count - 64;
                       end
                       else begin
                          wr_cmd_bl   <=  write_count - 1;
                          wr_usr_count<=  write_count;
                          write_count     <=  0;
                       end
                    end
                 end
                 //Initiate a transaction
                 wr_cmd_stb    <=  1;
                 wr_state      <=  WRITE;
                 wr_wait       <=  0;
              end
           end
           WRITE: begin
              if (wr_cmd_empty) begin
                 //if (wr_wait < 4) begin
                 //It takes a clock cycle for the wr_cmd_empty to deassert so this
                 //will eat up a couple of clocks
                 //  wr_wait <=  wr_wait + 1;
                 //end
                 //else begin
                 //When the command has finished executed update the wr_mem_addr
                 //with the new memory count
                 //  the !wr_cmd_stb is here to prevent a possible race condition

                 if (wr_mem_addr + wr_usr_count > `DDR3_FIFO_MAX_ADDR) begin
                    wr_mem_addr <=  0;
                 end
                 else begin
                    wr_mem_addr     <=  wr_mem_addr + wr_usr_count;
                 end

                 wr_usr_count    <=  0;
                 wr_state        <=  IDLE;
              end
           end
         endcase
      end
   end


   //Read Side Synchronous Logic
   always @ (posedge rd_clk) begin
      if (ddr3_rst) begin
         rd_mem_addr       <=  0;
         rd_cmd_bl         <=  0;
         rd_cmd_stb        <=  0;
         r_ddr3_fifo_avail <=  0;
         r_ddr3_fifo_full  <=  0;
         rd_state          <=  IDLE;
         rd_wait           <=  0;
         read_count        <=  0;
      end
      else begin
         r_ddr3_fifo_avail <=  ddr3_fifo_avail;
         r_ddr3_fifo_full  <=  ddr3_fifo_full;
         //Strobes
         rd_cmd_stb        <=  0;
         if (rd_stb) begin
            read_count      <=  read_count - 1;
         end

         case (rd_state)
           IDLE: begin
              rd_wait       <=  0;
              if (rd_cmd_empty && rd_empty && (read_count == 0)) begin
                 if (ddr3_fifo_avail > 0) begin
                    if (ddr3_fifo_avail < 64) begin
                       if ((ddr3_fifo_avail + rd_mem_addr) > `DDR3_FIFO_LEN) begin
                          rd_cmd_bl   <=  `DDR3_FIFO_LEN - rd_mem_addr - 1;
                          read_count  <=  `DDR3_FIFO_LEN - rd_mem_addr;
                       end
                       else begin
                          rd_cmd_bl   <=  ddr3_fifo_avail - 1;
                          read_count  <=  ddr3_fifo_avail;
                       end
                    end
                    else begin
                       if ((64 + rd_mem_addr) > `DDR3_FIFO_LEN) begin
                          rd_cmd_bl   <=  `DDR3_FIFO_LEN - rd_mem_addr - 1;
                          read_count  <=  `DDR3_FIFO_LEN - rd_mem_addr;
                       end
                       else begin
                          rd_cmd_bl   <=  64 - 1;
                          read_count  <=  64;
                       end
                    end
                    rd_cmd_stb  <=  1;
                    rd_state    <=  READ;
                 end
              end
           end
           READ: begin
              if (!rd_empty) begin
                 //Update the rd_mem_addr, We need to do it here because the rd_mem_addr
                 //address was just locked into a command request and now we can
                 //increment it
                 if ((rd_mem_addr + rd_cmd_bl + 1) > `DDR3_FIFO_MAX_ADDR) begin
                    rd_mem_addr <=  0;
                 end
                 else begin
                    rd_mem_addr     <=  rd_mem_addr + rd_cmd_bl + 28'd1;
                 end
                 rd_state    <=  IDLE;
              end
           end
         endcase
      end
   end

`ifdef ENABLE_CHIPSCOPE
   chipscope_ila_ddr3_fifo ila(
                               .CONTROL    (chipscope_control),
                               .CLK        (wr_clk),
                               .TRIG0      (ila_data)
                               );
`else
   //Set to high Z when not in use
   assign  chipscope_control = 32'hZZZZZZZZ;
`endif
endmodule
