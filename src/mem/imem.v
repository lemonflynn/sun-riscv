`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:35:57 12/16/2020 
// Design Name: 
// Module Name:    imem 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module imem (
  input clk,
  input ena,
  input [3:0] wea,
  input [13:0] addra,
  input [31:0] dina,
  input [13:0] addrb,
  output [31:0] doutb
);
  //reg [31:0] mem [4096-1:0];
  reg [31:0] mem [16384-1:0];
  wire[11:0] addra_align;
  wire[11:0] addrb_align;

  assign addra_align = addra[13:2];
  assign addrb_align = addrb[13:2];

  assign doutb = mem[addrb_align];

  genvar i;
  generate for (i = 0; i < 4; i = i+1) begin:imem_byte
    always @(posedge clk) begin
      if (wea[i] && ena)
          mem[addra_align][i*8 +: 8] <= dina[i*8 +: 8]; //indexed part selected.
    end
  end endgenerate
endmodule
