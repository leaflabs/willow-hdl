`timescale 1ns/1ps

module cross_clock_data #(
  parameter   DATA_WIDTH    = 8
)(
  input                           rst,
  input       [DATA_WIDTH - 1: 0] in_data,

  input                           out_clk,
  output  reg [DATA_WIDTH - 1: 0] out_data = 0
);


reg     [DATA_WIDTH - 1: 0] data_sync [2:0] = {0,0,0};

always @ (posedge out_clk) begin
  if (rst) begin
    out_data                      <=  0;
    data_sync[0]                  <=  0;
    data_sync[1]                  <=  0;
    data_sync[2]                  <=  0;
  end
  else begin
    if ((data_sync[2] == data_sync[1]) && (data_sync[2] == data_sync[0])) begin
      out_data                    <=  data_sync[2];
    end
    data_sync[0]                  <=  in_data;
    data_sync[1]                  <=  data_sync[0];
    data_sync[2]                  <=  data_sync[1];
  end
end

endmodule
