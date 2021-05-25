`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:40:11 12/18/2020 
// Design Name: 
// Module Name:    dmem 
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
`define UART_CONTROL        16'h00
`define UART_RECEIVE_DATA   16'h04
`define UART_TRANSMIT_DATA  16'h08
`define CYCLE_COUNTER       16'h10
`define INSTRUCTION_COUNTER 16'h14
`define RESET_COUNTER       16'h18

module mmio(
  input clk,
  input en,
  input [3:0] we,
  input instruction_complete,
  input [13:0] addr,
  input [31:0] din,
  output reg [31:0] dout
);
reg [31:0] clock_counter;
reg [31:0] instruction_counter;
reg [31:0] uart_control;
reg [31:0] uart_receive_data;
reg [31:0] uart_transmit_data;

always@(posedge clk)
begin
    if(addr[7:0] != `RESET_COUNTER || we == 4'b0) begin
        clock_counter <= clock_counter + 1;
        if(instruction_complete)
            instruction_counter <= instruction_counter + 1;
        else
            instruction_counter <= instruction_counter;
    end else begin
        clock_counter <= 32'b0;
        instruction_counter <= 32'b0;
    end
end

always@(*)
begin
    if(en) begin
        case(addr[7:0])
            `UART_CONTROL: dout = uart_control;
            `UART_RECEIVE_DATA: dout = uart_receive_data;
            `CYCLE_COUNTER: dout = clock_counter;
            `INSTRUCTION_COUNTER: dout = instruction_counter;
            default: dout = 32'b0;
        endcase
    end else begin
        dout = 32'b0;
    end
end

genvar i;
generate for (i = 0; i < 4; i = i+1) begin:dmem_byte
always @(posedge clk)
begin
    if(en) begin
        case(addr[7:0])
            `UART_TRANSMIT_DATA: begin
                if (we[i])
                    uart_transmit_data[i*8 +: 8] <= din[i*8 +: 8];
            end
            `RESET_COUNTER: begin
                if (we[i]) begin
                    instruction_counter <= 32'b0;
                end
            end
        endcase
    end
end
end endgenerate
endmodule
