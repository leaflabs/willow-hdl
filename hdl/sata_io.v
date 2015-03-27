module sata_io (
                input wire         SystemClk,
                input wire         nRESET,

                input wire [16:0]  sectorcount,

                input wire         StartWrite,
                input wire         StartRead,
                input wire [47:0]  SectorAddress,
                input wire         sata_error,
                input wire         DevReady, // Allows us to know when the command is completed...
                input wire [47:0]  MAXLBA,

                output reg [6:0]   XferReq = 0,
                output reg [47:0]  CmdLBA = 0, // change to reg when ready to experiment with LBA addressing
                output reg [7:0]   CmdCommand = 0,
                output wire [16:0] CmdSectorCnt, // change to reg when ready to experiment with variable DMA requests
                output wire [7:0]  CmdDevice,
                output wire [15:0] CmdFeatures,
                output wire [7:0]  CmdControl,
                output wire [3:0]  SoftReset,
                output wire        CmdIsDma,

                output wire  sata_io_ready

                );

   parameter [4:0]  TEST_IDLE   = 5'h0,
     TEST_ST_CNT = 5'h1,  // Start test - counter, not used.
     TEST_WR_STP = 5'h2,  // setup a write command
     TEST_WR_CMD = 5'h3,  // send the command -
     TEST_WR_WT0 = 5'h4,  // Wait for completion
     TEST_WR_WT1 = 5'h5,  // Wait for completion
     TEST_WR_STS = 5'h6,  // Check Status
     TEST_WR_ERR = 5'h7,  // Error state
     TEST_RD_STP = 5'h8, // setup a write command
     TEST_RD_CMD = 5'h9, // send the command -
     TEST_RD_WT0 = 5'ha, // Wait for completion
     TEST_RD_WT1 = 5'hb, // Wait for completion
     TEST_RD_STS = 5'hc, // Check Status
     TEST_RD_ERR = 5'hd; // Error state

   reg [4:0]                       SysState = TEST_IDLE;

   reg                             StartWrite_l = 0;
   reg                             StartRead_l = 0;
   reg                             StartOp_d = 0;

   wire                            StartOp_edge;

   // a one cycle pulse that starts the state machine running
   assign StartOp_edge = (StartWrite_l || StartRead_l) && !StartOp_d;

   assign sata_io_ready = (DevReady && (SysState == TEST_IDLE));

   // For DMA READ/Write these stay constant so tie them off
   assign CmdDevice   = 8'h40;
   assign CmdControl  = 8'h80;
   assign CmdFeatures = 16'h0000;
   assign SoftReset   = 4'h0;
   assign CmdIsDma    = 1'b1;

   // For now, tie this off, but eventually we need this to select the start address of any DMA op.
   assign CmdSectorCnt = sectorcount;

   // delayed version of startOp - allows us to create a "pulse" on StartOpEdge
   always @ (posedge SystemClk or negedge nRESET)
     if (!nRESET) StartOp_d <= 1'b0;
     else         StartOp_d <= (StartRead_l || StartWrite_l);

   always @ (posedge SystemClk or negedge nRESET)
     begin
        if (!nRESET)
          begin
             StartWrite_l <= 1'b0;
             StartRead_l <= 1'b0;
          end
        else
          begin
             StartWrite_l <= StartWrite && DevReady; // ignore commands when drive is not ready
             StartRead_l <= StartRead && DevReady;
          end
     end

   // Xfer Req
   //  5'b00001 : SEND_ND_CMD;
   //  5'b00010 : SEND_PIO_RD;
   //  5'b00100 : SEND_PIO_WR;
   //  5'b01000 : SEND_DMA_RD;
   //  5'b10000 : SEND_DMA_WR;
   // Asert one of these command bits for one clock
   always @ (posedge SystemClk or negedge nRESET)
     begin
        if (!nRESET)                          XferReq <= 5'b00000;
        else if (SysState == TEST_WR_CMD) XferReq <= 5'b10000; // SEND_DMA_WR
        else if (SysState == TEST_RD_CMD) XferReq <= 5'b01000;
        else                                  XferReq <= 5'b00000;
     end

   // Command
   // value to place in the command register of the host to device register frame. See SATA command set.
   // Many useful commands are described in the SATA CORE USER GUIDE from intelliprop.

   always @ (posedge SystemClk or negedge nRESET)
     begin
        if (!nRESET) CmdCommand <= 8'h0;
        else if (SysState == TEST_WR_STP) CmdCommand <= 8'h35;
        else if (SysState == TEST_RD_STP) CmdCommand <= 8'h25;
     end

   // sensitivty list for the master state machine includes ALL dependent signals
   always @ (posedge SystemClk)
     begin
        if (!nRESET) SysState <= TEST_IDLE;
        else begin
           case(SysState)
             TEST_IDLE : begin
                SysState <= StartOp_edge ? TEST_ST_CNT : TEST_IDLE;
                CmdLBA <= SectorAddress;
             end
             TEST_ST_CNT: begin  // Start test - counter
                if (StartWrite_l)     SysState <= TEST_WR_STP;
                else if (StartRead_l) SysState <= TEST_RD_STP;
                else                  SysState <= TEST_IDLE;
             end
             TEST_WR_STP: begin  // setup a write command
                SysState <= TEST_WR_CMD;
             end
             TEST_WR_CMD: begin  // send the command -
                SysState <= TEST_WR_WT0;
             end
             TEST_WR_WT0: begin  // Wait for completion
                if (!DevReady) SysState <= TEST_WR_WT1;
                else             SysState <= TEST_WR_WT0;
             end
             TEST_WR_WT1: begin  // Wait for completion
                if      (sata_error) SysState <= TEST_WR_ERR;
                else if ((sata_error == 1'b0) && DevReady)        SysState <= TEST_WR_STS;
                else                                                  SysState <= TEST_WR_WT1;
             end
             TEST_WR_STS: begin  // Check Status
                SysState <= TEST_IDLE;
             end
             TEST_WR_ERR: begin  // Error state, we dont recover...yet.
                SysState <= TEST_WR_ERR;
             end
             TEST_RD_STP: begin // setup a write command
                SysState <= TEST_RD_CMD;
             end
             TEST_RD_CMD: begin // send the command -
                SysState <= TEST_RD_WT0;
             end
             TEST_RD_WT0: begin // Wait for completion
                if (!DevReady) SysState <= TEST_RD_WT0;
                else             SysState <= TEST_RD_WT1;
             end
             TEST_RD_WT1: begin // Wait for completion
                if      (sata_error) SysState <= TEST_RD_ERR;
                else if ((sata_error == 1'b0) && DevReady)        SysState <= TEST_RD_STS;
                else                                                  SysState <= TEST_RD_WT1;
             end
             TEST_RD_STS: begin // Check Status
                SysState <= TEST_IDLE;
             end
             TEST_RD_ERR: begin // Error state
                SysState <= TEST_RD_ERR;
             end
             default:
               SysState <= TEST_IDLE;
           endcase // case (SysState)
        end
     end // always @
endmodule // sata_io
