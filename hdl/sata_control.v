/*
 * sata_control.v
 *
 * Copyright: (C) 2013 LeafLabs, LLC
 * License: MIT License (See LICENSE file)
 * Author: Jessica Barber <jessb@leaflabs.com>
 * Author: Andew Meyer <ajm@leaflabs.com>
 * Date: May-July 2013
 *
 */

module sata_control #(
                       parameter SERDES_TYPE = "SPARTAN6",
                       parameter SIMULATION = 0
                       )
   (
    input wire         write_enable,
    input wire         read_enable,
    input wire         zero_enable, //Pull this high when writing to write
    //zeroes to the HD

    input wire [16:0]  read_sectorcount,
    input wire [16:0]  write_sectorcount,
    input wire [47:0]  read_sectoraddress,
    input wire [47:0]  write_sectoraddress,
    output wire [47:0] current_sectoraddress,

    input wire [23:0]  write_delay_cycles,

    output wire        HOST_TXDT_P0,
    output wire        HOST_TXDT_N0,
    output wire        HOST_TXDT_P1,
    output wire        HOST_TXDT_N1,
    input wire         HOST_RXDT_P0,
    input wire         HOST_RXDT_N0,
    input wire         HOST_RXDT_P1,
    input wire         HOST_RXDT_N1,

    input wire         REFCLK,
    input wire         RESET,

    output wire        clkout,

    input wire         udp_fifo_full,
    output wire        udp_fifo_write_en,
    output wire [31:0] udp_fifo_din,
    input wire         udp_fifo_prog_full,

    output wire        daq_fifo_clock,
    output wire        daq_fifo_read_en,
    input wire         daq_fifo_empty,
    input wire [31:0]  daq_fifo_data,

    //Status Lines
    output wire        device_ready, // TODO get rid of
                                     // this. deprecated by
                                     // sata_ready. HDD has linkup and
                                     // is not currently executing a
                                     // read or write
    output wire        sata_ready, // USE THIS to determine if it's ok
                                   // to send sata commands. same as
                                   // device_ready AND sata_io logic
                                   // is ready to accept a command.
    output wire        phy_ready, // indicates that a hard drive is
                                  // connected and has linkup. stays
                                  // high unless the drive is
                                  // disconnected or the connection is
                                  // lost somehow
    output wire        sata_error, // an error has occured, check
                            // sata_error_code for more information
    output wire [7:0] sata_error_code,
    output reg [3:0]   sata_state = SATA_IDLE

    );

   localparam START_SPEED = 3'b001;
   localparam NUM_REF_CLOCKS = 1;

   wire [1:0]          TXDT_P;
   wire [1:0]          TXDT_N;
   wire [1:0]          RXDT_P;
   wire [1:0]          RXDT_N;
   wire [47:0]         MAX_LBA;
   (* keep = "true" *) wire [6:0]       XferReq;
   wire                XferExpectStatus;
   wire [47:0]         CmdLBA;
   wire [7:0]          CmdDevice;
   wire [15:0]         CmdFeatures;
   wire [7:0]          CmdCommand;
   wire [16:0]         CmdSectorCnt;
   wire [7:0]          CmdControl;
   wire [3:0]          SoftReset;
   wire                CmdIsDma;

   (* keep = "true" *) wire         Ready_wire;
   (* keep = "true" *) wire [47:0]  MAXLBA;

   (* keep = "true" *) wire     DataGenPopData;
   (* keep = "true" *) wire     DataCheckPushData;
   (* keep = "true" *) wire     FIFO_TX_RD;
   (* keep = "true" *) wire     FIFO_RX_WR;
   (* keep = "true" *) wire     WrFifoPopData;
   (* keep = "true" *) wire     RdFifoPushData;
   (* keep = "true" *) wire [31:0]  GeneratedData;
   (* keep = "true" *) wire [33:0]  DATA_TO_RX_FIFO;

   (* keep = "true" *) wire     REFCLK_wire;
   (* keep = "true" *) wire     TestCompleteErr;
   (* keep = "true" *) wire [63:0]      TimerCounter;
   //(* keep = "true" *) wire [NUM_REF_CLOCKS-1:0] REFCLK;

   wire                ff1_full;
   wire                ff1_empty;
   wire                ff1_almost_full;
   wire                ff1_almost_empty;
   wire                ff1_rd_valid;
   wire [13:0]         ff1_rd_count;
   wire [13:0]         ff1_wr_count;

   wire                overflow;
   wire                underflow;

   // SATA Signals

   wire [5:0]          XferBlockSize1;     // input 6bits,  used for pio transfers, number of sectors between pio setups
   // - can be 2, 4, 8, 16, 32, etc...
   wire [15:0]         StatusSectorCount1; // output 16bits,  SectorCount returned on D2H status frame
   wire [47:0]         StatusLBA1;         // output 48bits,  LBA returned on D2H status Frame  Not needed
   wire [7:0]          StatusStatus;      // output 8bits,  status register returned on D2H frame, this is always 0x50 or x40,
   // if it is ever a 0x51 or 0x41 an error occurred StatusDevice // Not needed
   wire [7:0]          StatusDevice1;      // output 8bits,  device register returnd on D2H frame
   wire [7:0]          StatusError;       // output 8bits,  error register returned on D2H frame
   wire                DevReady;          // output 1 bit,  asserted when ready to send new commands // The drive is online,
   // and you can send a command - like Raid Ready (please use this)
   wire                Gen2Speed1;         // output 1bit
   wire                AcceptXrdy1;        // input  1bit,  Blocks RRdy from asserting
   wire                ff1_FullWarning;    // input REQUIRED, this is to tell the SATA core that on reads the FIFO is near full.
   // Must be at least 32 Dwords (128Bytes) from being full.
   wire [31:0]         notuse0;
   wire [31:0]         notuse1;
   wire [31:0]         notuse2;
   wire [31:0]         notuse3;
   wire [31:0]         notuse4;
   wire [31:0]         notuse5;
   wire [31:0]         notuse6;
   wire [31:0]         notuse7;
   wire                HostRstn1;      //output 1bit,  if you have logic that you want to be in reset when the core is in reset, use this signal
   wire                ClockDword;     // 75MHZ,1bit,  if you have logic that needs to be synchronous to the core, use this clock.
   //In your test state machine, please use this as the CLOCK.

   wire                CrcError1;      //output 1bit - the frame received has a CRC error
   wire                Rerr1;          //output 1bit - on a frame we sent, there was a CRC error
   wire                PhyReady;      //output 1bit - Host core is connected -
   wire                PhyOffline1;    //output 1bit - Host core is not connected (in sleep)
   wire                FrameReceived1; //output 1bit - Frame received
   wire                FrameSent1;     //output 1bit - Frame was sent
   wire                ComwakeRcvd1;   //output 1bit - Received a startup signal OOB from the drive
   wire                CominitRcvd1;   //output 1bit - Received a reset from the OOB from the drvie
   wire [3:0]          SERDES_STATUS1; //output 4bits- checks if there are errors in the SERDES

   wire [2:0]          speeds_allowed;
   wire [2:0]          speedcurrent_wire;

   reg [39:0]          ByteCnt = 0; // 40 bits lets us count up to 1 TB written to the drive.
   reg [23:0]          BenchCnt = 0;
   reg [3:0]           BenchState = 0;
   reg [1:0]           ClockCnt = 0;

   assign  XferBlockSize1=6'h8;
   //input 6bits used for pio transfers, number of sectors between pio
   //setups - set for 8 now, can be 2, 4, 8, 16, 32, etc...
   assign  MAXLBA = 48'h1_0000_0000;//for a single SATA
   assign AcceptXrdy1=1;
   //This is a way to control the SATA core frames being received,
   //should be =1.  =0 will prevent the core from getting to DevReady.
   //needs be set input REQUIRED, this is to tell the SATA core that
   //on reads the FIFO is near full.  Must be at least 32 Dwords
   //(128Bytes) from being full.

   assign TXDT_P[1]= 1'b0;
   assign TXDT_N[1]=1'b0;

   assign HOST_TXDT_P0 = TXDT_P[0];
   assign HOST_TXDT_N0 = TXDT_N[0];
   assign HOST_TXDT_P1 = TXDT_P[1];
   assign HOST_TXDT_N1 = TXDT_N[1];
   assign RXDT_P[0] = HOST_RXDT_P0;
   assign RXDT_N[0] = HOST_RXDT_N0;
   assign RXDT_P[1] = HOST_RXDT_P1;
   assign RXDT_N[1] = HOST_RXDT_N1;

   localparam sectors = 8;  // 8 for fifo32x1024 64 for fifo32x8192

   assign DataCheckPushData =  udp_fifo_write_en ;
   assign DataGenPopData    =  1'b1; // FIFO_TX_RD ;

   //---------------------------------------
   // FIFO IN LOGIC
   // Data comes INTO the HD OUT of this FIFO
   //---------------------------------------

   assign daq_fifo_clock = clkout; // Dword

   always @ (posedge ClockDword) begin
      ClockCnt <= ClockCnt+2'b01;

      if (RESET)    ByteCnt <= 40'b0;
      else if (daq_fifo_read_en == 1) begin ByteCnt<=ByteCnt+1;
      end

   end


   // Sata Control Logic
   reg start_read, start_write = 0;
   reg [47:0] SectorAddress = 0;
   reg [16:0] SectorCount = 0;

   localparam [3:0]
     SATA_IDLE = 4'h0,
     SATA_WRITE_0 = 4'h1,
     SATA_WRITE_1 = 4'h2,
     SATA_WRITE_2 = 4'h3,
     SATA_READ_0 = 4'h4,
     SATA_READ_1 = 4'h5,
     SATA_READ_2 = 4'h6,
     SATA_WRITE_DELAY = 4'h7;

   /* Overall logic: When switch_0 is high, we continuously write to
    consecutive sectors, with the FIFO logic figuring out the actualy
    data to put in. LBA is zerod out when switch_0 goes high, and a
    pulse is emitted on do_write. When the ready line goes back high,
    if switch_0 is still high, then increment SectorAddress by the DMA
    Sector Cnt, and pulse do_write again.

    Switch_0 takes precendence over switch_1, but they have roughly
    the same logic.  */

   reg [23:0] delay_counter = 0;
   always @ (posedge ClockDword) begin
      if (RESET)
        begin
           sata_state <= SATA_IDLE;
        end
      else
        case (sata_state)
          SATA_IDLE: begin
             delay_counter <= 0;
             start_read <= 1'b0;
             start_write <= 1'b0;
             if (write_enable == 1) begin
                sata_state <= SATA_WRITE_0;
                SectorAddress <= write_sectoraddress;
                SectorCount <= write_sectorcount;
             end
             else if (read_enable == 1) begin
                sata_state <= SATA_READ_0;
                SectorAddress <= read_sectoraddress;
                SectorCount <= read_sectorcount;
             end
             else sata_state <= SATA_IDLE;
          end
          SATA_WRITE_0: begin
             start_write <= 1'b1;
             sata_state <= SATA_WRITE_1;
          end
          SATA_WRITE_1: begin
             if (~sata_ready) begin
                start_write <= 1'b0;
                sata_state <= SATA_WRITE_2;
             end
             else sata_state <= SATA_WRITE_1;
          end
          SATA_WRITE_2: begin
             if (sata_ready == 1'b1)
               begin
                 if (write_enable == 1'b1)
                   begin
                    delay_counter <= 0;
                    sata_state <= SATA_WRITE_DELAY;
                    SectorAddress <= SectorAddress + SectorCount;
                   end
                 else
                   begin
                     sata_state <= SATA_IDLE;
                   end
               end
          end
          SATA_WRITE_DELAY: begin
             if (delay_counter >= write_delay_cycles) begin
                sata_state <= SATA_WRITE_0;
                delay_counter <= 0;
             end else begin
                delay_counter <= delay_counter + 24'd1;
             end
          end
          SATA_READ_0: begin
             start_read <= 1'b1;
             sata_state <= SATA_READ_1;
          end
          SATA_READ_1: begin
             if (~sata_ready)
               begin
                  start_read <= 1'b0;
                  sata_state <= SATA_READ_2;
               end
             else sata_state <= SATA_READ_1;
          end
          SATA_READ_2: begin
             if (sata_ready == 1'b1)
               if (read_enable == 1'b1)
                 begin
                    sata_state <= SATA_READ_0;
                    SectorAddress <= SectorAddress + SectorCount;
                 end
               else
                 sata_state <= SATA_IDLE;
          end
          default : begin
             sata_state <= SATA_IDLE;
          end

        endcase // case (sata_state)
   end // always @ (posedge ClockDword)

   //---------------------------------------
   // Test Logic
   //---------------------------------------
   sata_io sata_io_i
     (
      .SystemClk(ClockDword),
      .nRESET(HostRstn1),

      .sectorcount(SectorCount),

      .StartWrite(start_write),
      .StartRead(start_read),
      .SectorAddress(SectorAddress),
      .DevReady(DevReady),
      .sata_error(sata_error),
      .MAXLBA(MAXLBA),

      .XferReq(XferReq),
      .CmdLBA(CmdLBA),
      .CmdDevice(CmdDevice),
      .CmdFeatures(CmdFeatures),
      .CmdCommand(CmdCommand),
      .CmdSectorCnt(CmdSectorCnt),  //number of sectors
      .CmdControl(CmdControl),
      .SoftReset(SoftReset),
      .CmdIsDma(CmdIsDma),

      .sata_io_ready(sata_io_ready)
      );


   //IBUFDS iBUFClk(.I(REFCLK_P), .IB(REFCLK_N), .O(REFCLK));
   assign speeds_allowed = 3'b001;

   //---------------------------------------
   // simulation registers - not for synth
   //---------------------------------------

   reg                       allow_powerdown_reg = 1'b0;// Allows host to respond to a power sequence correctly or to reject (if 0)
   reg                       partial_go_reg = 1'b0; // host will start the partial power down sequence
   reg                       slumber_go_reg = 1'b0; // host will start the slumber power down sequence
   reg                       wake_up_reg = 1'b0;    // host will start the wake up sequence
   wire                      core_in_sleep_wire;    // This signal (output) will tell you that the core is in SLEEP
   wire                      pm_fail_wire;

   //---------------------------------------
   // Sata host wrapper
   //---------------------------------------
   (* keep = "true" *) sata_core_wrap_hst_app #(
                                                .SERDES_TYPE(SERDES_TYPE),
                                                .NUM_REF_CLOCKS(NUM_REF_CLOCKS),
                                                .START_SPEED(START_SPEED),
                                                .SIMULATION(SIMULATION)
                                                )
   sata_core_wrap_hst_app_i0 (

                           .CoreConfig(64'b0),
                           .refClk(REFCLK),                // master clock  150Mhz
                           .ReconfigClk(),
                           .nReset(!RESET),    // async master reset
                           .PortReset(1'b0),               // holds the sata core in reset, If you want to prevent this core from working, set this to a 1.
                           .PutCrcInFifo(1'b0),            // this is for our bridging application, not to use here, tells the sata core to put the received crc in the fifo on data xfers
                           .BypassCrc(1'b0),               // Removes CRC - do not use, tells the sata core not to send the calculated crc
                           .SoftReset(SoftReset[0]),               // resets the host state machine
                           .XferReq(XferReq),              // assert for one clock to start a transfer
                           // 5'b00001 : SEND_ND_CMD;
                           // 5'b00010 : SEND_PIO_RD;
                           // 5'b00100 : SEND_PIO_WR;
                           // 5'b01000 : SEND_DMA_RD;
                           // 5'b10000 : SEND_DMA_WR;

                           .XferBlockSize(XferBlockSize1[4:0]), // used for pio transfers, number of sectors between pio setups: - set for 8 now, can be 2, 4, 8, 16, 32, etc...
                           .XferExpectStatus(1'b1),        // 48 bits input, Indicates to expect a status after a non data command, mainly intended to be deasserted when setting srst bit
                           .CmdAuxiliary(16'b0),
                           .CmdSectorCount(CmdSectorCnt[15:0]),  // input [15:0] Sector Count to send with command
                           .CmdLBA(CmdLBA),                    // LBA to send with command
                           .CmdCommand(CmdCommand),            // 8bits input,  Command to send
                           .CmdDevice(CmdDevice),                // 16bits input, Device register to send with command
                           .CmdFeatures(CmdFeatures),        // 8bits input,  Features register to send with the command
                           .CmdControl(CmdControl),
                           .SectorCount(CmdSectorCnt),       // input [16:0] actual number of sectors to transfer on data commands
                           // This should be connected to CmdSectorCount.
                           .SataPmField(8'h80),            // 8bis PM field to send in H2D sata,  Do not use - it is for talking to a PM
                           .StatusSectorCount(StatusSectorCount1),  // output 16bite output,  SectorCount returned on D2H status frame
                           .StatusLBA(StatusLBA1),                        // output 48biits output, LBA returned on D2H status Frame  Not needed
                           .StatusStatus(StatusStatus),            // output 8bits output,  status register returned on D2H frame  Needed - just check this is always 0x50 or x40, if it is ever a 0x51 or 0x41 an error occurred StatusDevice // Not needed
                           .StatusDevice(StatusDevice1),            // output 8bits output, device register returnd on D2H frame
                           .StatusError(StatusError),              // output 8bits output, error register returned on D2H frame
                           .DevReady(DevReady),                          // output 1bits output, asserted when ready to send new commands, The drive is online, and you can send a command - like Raid Ready (please use this)
                           //.Gen2Speed(Gen2Speed),                   // output 1bits output,
                           .AcceptXrdy(AcceptXrdy1),                // input  1bit input,  Blocks RRdy from asserting

                           // serial interface
                           .RXDT_P(RXDT_P[0]),                         //Serial Bus
                           .RXDT_N(RXDT_N[0]),                         //Serial Bus
                           .TXDT_P(TXDT_P[0]),                         //Serial Bus
                           .TXDT_N(TXDT_N[0]),                         //Serial Bus

                           // FIFO interface

                           .FifoReadEn(daq_fifo_read_en),                 //output
                           .FifoWriteEn(udp_fifo_write_en),                //output
                           .FifoFull(udp_fifo_full),             //input
                           .FifoFullWarning(udp_fifo_prog_full),     //input REQUIRED - this is to tell the SATA core that on reads the FIFO is near full.  Must be at least 32 Dwords (128Bytes) from being full.
                           .FifoEmpty(daq_fifo_empty),            //input
                           .FifoReadData({2'b0,daq_fifo_data}),          //input[33:0] send data out to HD,
                           //                          .FifoReadData({2'b0,fifo_in_din}),          //input[33:0] send data out to HD,
                           .FifoWriteData(udp_fifo_din),         //outpu[33:0]
                           /* -----\/----- EXCLUDED -----\/-----

                            .FifoReadEn(FIFO_TX_RD),                 //output
                            .FifoWriteEn(FIFO_RX_WR),                //output
                            .FifoFull(1'b0),             //input
                            .FifoFullWarning(1'b0),     //input REQUIRED - this is to tell the SATA core that on reads the FIFO is near full.  Must be at least 32 Dwords (128Bytes) from being full.
                            .FifoEmpty(1'b0),            //input
                            .FifoReadData(34'b0),//DATAFRM_TX_FIFO),          //input[33:0] send data out to HD,
                            .FifoWriteData(),         //outpu[33:0]
                            -----/\----- EXCLUDED -----/\----- */

                           //RxRegisters
                           .RxHeaderReg0(notuse0),                  //do not use
                           .RxHeaderReg1(notuse1),                  //do not use
                           .RxHeaderReg2(notuse2),                  //do not use
                           .RxHeaderReg3(notuse3),                  //do not use
                           .RxHeaderReg4(notuse4),                  //do not use
                           .RxHeaderReg5(notuse5),                  //do not use
                           .RxHeaderReg6(notuse6),                  //do not use
                           .RxHeaderReg7(notuse7),                  //do not use

                           // core generated clock and synchrnoized reset
                           .HostRstn(HostRstn1),                    //1bit output,  if you have logic that you want to be in reset when the core is in reset, use this signal
                           .ClockDword(ClockDword),                //1bit output,  if you have logic that needs to be synchronous to the core, use this clock.  In your test state machine, please use this as the CLOCK.

                           // misc status signals
                           // You don't need any of these signals right now, but if you want to know what they do:
                           .CrcError(CrcError1),                    //output - the frame received has a CRC error
                           .Rerr(Rerr1),                            //output - on a frame we sent, there was a CRC error
                           .PhyReady(PhyReady),                    //output - Host core is connected -
                           .PhyOffline(PhyOffline1),                //output - Host core is not connected (in sleep)
                           .FrameReceived(FrameReceived1),          //output - Frame received
                           .FrameSent(FrameSent1),                  //output - Frame was sent
                           .ComwakeRcvd(ComwakeRcvd1),              //output - Received a startup signal OOB from the drive
                           .CominitRcvd(CominitRcvd1),              //output - Received a reset from the OOB from the drvie
                           .SERDES_STATUS(SERDES_STATUS1),          //output - checks if there are errors in the SERDES

                           // test mux interface
                           //.TestSelect(4'b0),                       //input [3:0]
                           //.TestMuxBus(TestMuxBus1),                 //output [31:0]

                           .StatusEnding(),
                           .TerminateXfer(1'b0),
                           //.XrdyCollision(),
                           .allow_powerdown(1'b0),  // Allows host to respond to a power sequence correctly or to reject (if 0)
                           .partial_go(1'b0),       // host will start the partial power down sequence
                           .slumber_go(1'b0),       // host will start the slumber power down sequence
                           .wake_up(1'b0),          // host will start the wake up sequence
                           .core_insleep(core_in_sleep_wire),         // This signal (output) will tell you that the core is in SLEEP
                           .pm_fail(pm_fail_wire),              // This signal (output) will tell you that the Powermode that was started by the core, FAILED due to the drive
                           //.RcvdStartOfFrame(),
                           //.TagVal(),
                           //.TransferCount(),
                           //.SActive(),
                           //.FrameReceivedEarly(),
                           .DevFrameTransmitted(),
                           .FrameSentOk(),
                           .RcvdTxSyncTerminate(),
                           .RcvdRxSyncTerminate(),
                           .RcvdSync(),
                           .RxLos(),
                           .Rok(),
                           .TransmitError(),

                           //Intentionally Left Unconnected
                           .SpeedCurrent(speedcurrent_wire),
                           .XrdyCollision (),
                           .RcvdStartOfFrame (),
                           .TagVal (),
                           .TransferCount (),
                           .SActive (),
                           .FrameReceivedEarly (),

                           .speeds_allowed(speeds_allowed)
                           );

   assign device_ready = DevReady;
   assign sata_ready = sata_io_ready;
   assign phy_ready = PhyReady;
   assign sata_error = StatusStatus[0]; // The low bit of the
                                        // StatusStatus bus indicates
                                        // than an error has occured
                                        // if high. Check the value of
                                        // StatusError to determine
                                        // which error has occured.
   assign sata_error_code = StatusError;
   assign clkout = ClockDword;
   assign current_sectoraddress = SectorAddress;

endmodule
