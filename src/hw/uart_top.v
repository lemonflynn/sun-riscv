`timescale 1ns / 1ps

module uart_top #(
    parameter CLOCK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,
    input uart_rx_i,
    output uart_tx_o
);

wire[7:0] data_in, data_out;
reg [7:0] data_convert, data_latch;
wire data_in_valid, data_in_ready;
wire data_out_valid, data_out_ready;
wire valid, ready;

uart # (
    .CLOCK_FREQ(CLOCK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) on_chip_uart (
    .clk(clk),
    .reset(reset),
    .data_in(data_in),
    .data_in_valid(!data_in_valid),
    .data_in_ready(data_in_ready),
    .data_out(data_out),
    .data_out_valid(data_out_valid),
    .data_out_ready(!data_out_ready),
    .serial_in(uart_rx_i),
    .serial_out(uart_tx_o)
);

fifo # (
    .data_width(8),
    .fifo_depth(32)
) fifo_1 (
    .clk(clk),
    .reset(reset),
    .wr_en(data_out_valid),
    .din(data_out),
    .full(data_out_ready),
    .rd_en(data_in_ready),
    .dout(data_in),
    .empty(data_in_valid) 
);

//always@(*)
//begin
//    if(data_in >= 97 && data_in <= 122)
//        data_convert = data_in + 32;
//    else
//        data_convert = data_in;
//end

endmodule
