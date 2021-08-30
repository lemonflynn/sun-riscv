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

module mmio # (
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input clk,
    input reset,
    input en,
    input [3:0] we,
    input instruction_complete,
    input [13:0] addr,
    input [31:0] din,
    output reg [31:0] dout,
    input serial_in,
    output serial_out
);
reg [31:0] clock_counter;
reg [31:0] instruction_counter;
reg [31:0] uart_control;
reg [31:0] uart_transmit_data;
reg trans_valid;
reg fifo_rd_en;
wire fifo_full, fifo_empty, fifo_wr_en;
reg reset_instruction_cnt;
wire trans_ready;
wire [7:0] fifo_in;
wire[7:0] fifo_out;

always@(posedge clk)
begin
    if(!en || addr[7:0] != `RESET_COUNTER || we == 4'b0) begin
        clock_counter <= clock_counter + 1;
        if(instruction_complete)
            instruction_counter <= instruction_counter + 1;
        else
            instruction_counter <= instruction_counter;
    end else if(reset_instruction_cnt == 1'b1) begin
        instruction_counter <= 32'b0;
    end else begin
        clock_counter <= 32'b0;
        instruction_counter <= 32'b0;
    end
end

always@(*)
begin
    if(en && addr[7:0] == `UART_RECEIVE_DATA)
        fifo_rd_en = 1'b1;
    else
        fifo_rd_en = 1'b0;
end

always@(*)
begin
    if(en) begin
        case(addr[7:0])
            `UART_CONTROL: dout = uart_control;
            `UART_RECEIVE_DATA:begin
                if(!fifo_empty)
                    dout = {24'b0, fifo_out};
                else
                    dout = 32'b0;
            end
            `CYCLE_COUNTER: dout = clock_counter;
            `INSTRUCTION_COUNTER: dout = instruction_counter;
            default: dout = 32'b0;
        endcase
    end else begin
        dout = 32'b0;
    end
end

always @(posedge clk)
begin
    trans_valid <= 1'b0;
    reset_instruction_cnt <= 1'b0;
    if(en) begin
        case(addr[7:0])
            `UART_TRANSMIT_DATA: begin
                if (we[0]) begin
                    trans_valid <= 1'b1;
                    uart_transmit_data <= {24'b0, din[7:0]};
                end
            end
            `RESET_COUNTER: begin
                if (we)
                    reset_instruction_cnt <= 1'b1;
            end
        endcase
    end
end

always @(posedge clk)
begin
    uart_control <= {30'b0, !fifo_empty, trans_ready};
end

uart #(
    .CLOCK_FREQ(CPU_CLOCK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) uart_on_chip (
    .clk(clk),
    .reset(reset),
    .data_in(uart_transmit_data[7:0]), //transmit data
    .data_in_valid(trans_valid),
    .data_in_ready(trans_ready),
    .data_out(fifo_in), //receive data
    .data_out_valid(fifo_wr_en),
    .data_out_ready(!fifo_full),
    .serial_in(serial_in),
    .serial_out(serial_out)
);

fifo # (
    .data_width(8),
    .fifo_depth(32)
) fifo_1 (
    .clk(clk),
    .reset(reset),
    .wr_en(fifo_wr_en),
    .din(fifo_in),
    .full(fifo_full),

    .rd_en(fifo_rd_en),
    .dout(fifo_out),
    .empty(fifo_empty)
);

endmodule
