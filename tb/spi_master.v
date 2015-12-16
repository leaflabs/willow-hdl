/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 SPI MODE 3
 CHANGE DATA @ NEGEDGE
 read data @posedge

 RSTB-active low asyn reset, CLK-clock, T_RB=0-rx  1-TX, mlb=0-LSB 1st 1-msb 1st
 START=1- starts data transmission cdiv 0=clk/4 1=/8   2=/16  3=/32
 CPOL = 0, CPHA = 0
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
module spi_master(rstb,clk,mlb,start,tdat,cdiv,din, ss,sck,dout,done,rdata);
   input rstb,clk,mlb,start;
   input [7:0] tdat;  //transmit data
   input [1:0] cdiv;  //clock divider
   input       din;
   output reg  ss = 1'b0;
   output reg  sck = 1'b0;
   output reg  dout = 1'b0;
   output reg  done = 1'b0;
   output reg [7:0] rdata = 8'b0; //received data

   parameter idle = 2'b00;
   parameter send = 2'b10;
   parameter finish = 2'b11;
   reg [1:0]        state = 2'b00;

   reg [7:0]        treg = 8'b0;
   reg [7:0]        rreg = 8'b0;
   reg [3:0]        nbit = 4'b0;
   reg [4:0]        mid = 5'b0;
   reg [4:0]        cnt = 5'b0;
   reg              shift = 1'b0;
   reg              clr = 1'b0;

   //FSM i/o
   always @(posedge clk) begin
      case(state)
        idle:begin
           if(start==1)
             begin
                case (cdiv)
                  2'b00: mid<=2;
                  2'b01: mid<=4;
                  2'b10: mid<=8;
                  2'b11: mid<=16;
                endcase
                shift<=1;
                done<=1'b0;
                state<=send;
             end // if (start==1)
           else begin
              ss <= 1;
              clr <= 1;
           end
        end //idle
        send:begin
           ss<=0;
           clr <= 0;
           if(nbit!=8)
             begin shift<=1; end
           else begin
              rdata<=rreg;
              done<=1'b1;
              state<=finish;
           end
        end//send
        finish:begin
           done <= 1'b0;
           shift<=0;
           ss<=0;
           clr<=1;
           state<=idle;
        end
        default: state<=idle;
      endcase
   end//always

   //setup falling edge (shift dout) sample rising edge (read din)
   always@(negedge clk or posedge clr) begin
      if(clr==1)
        begin cnt=0; sck=0; end
      else begin
         if(shift==1) begin
            cnt=cnt+1;
            if(cnt==mid) begin
               sck=~sck;
               cnt=0;
            end //mid
         end //shift
      end //rst
   end //always

   //sample @ rising edge (read din)
   always@(posedge sck or posedge clr ) begin // or negedge rstb
      if(clr==1)  begin
         nbit=0;  rreg=8'hFF;  end
      else begin
         if(mlb==0) //LSB first, din@msb -> right shift
           begin  rreg={din,rreg[7:1]};  end
         else  //MSB first, din@lsb -> left shift
           begin  rreg={rreg[6:0],din};  end
         nbit=nbit+1;
      end //rst
   end //always

   always@(negedge sck or clr) begin
      if(clr==1) begin
         treg=8'hFF;  dout=1;
      end
      else begin
         if(nbit==0) begin //load data into TREG
            treg=tdat; dout=mlb?treg[7]:treg[0];
         end //nbit_if
         else begin
            if(mlb==0) //LSB first, shift right
              begin treg={1'b1,treg[7:1]}; dout=treg[0]; end
            else//MSB first shift LEFT
              begin treg={treg[6:0],1'b1}; dout=treg[7]; end
         end
      end //rst
   end //always


endmodule
