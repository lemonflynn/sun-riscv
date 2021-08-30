`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2021 10:10:13 AM
// Design Name: 
// Module Name: uart_recv
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module uart_recv #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    output [7:0] data_out,
    output data_out_valid,
    input data_out_ready,

    input serial_in
);

parameter SYMBOL_EDGE_TIME     = CLOCK_FREQ / BAUD_RATE;
parameter SAMPLE_TIME          = SYMBOL_EDGE_TIME / 2;
parameter CLOCK_COUNTER_WIDTH  = $clog2(SYMBOL_EDGE_TIME);
parameter state_idle       = 2'b00; 
parameter state_receiving  = 2'b01; 
parameter state_received   = 2'b10; 

reg [1:0] cur_state, next_state;
reg [9:0] rx_shift = 10'b0;
reg [3:0] bit_counter;
reg [CLOCK_COUNTER_WIDTH-1:0] clock_counter;

assign data_out = rx_shift[9:1];
assign data_out_valid = cur_state == state_received;

wire neg_edge, sample;
assign neg_edge = clock_counter == SYMBOL_EDGE_TIME;
assign sample = clock_counter == SAMPLE_TIME;

always@(posedge clk or negedge reset)
begin
    if(reset == 1'b0 || (cur_state != state_receiving))
        clock_counter <= 0;
    else if(clock_counter > SYMBOL_EDGE_TIME)
        clock_counter <= 0;
    else
        clock_counter <= clock_counter + 1;
end

always@(posedge clk or negedge reset)
begin
    if(reset == 1'b0 || (cur_state != state_receiving))
        bit_counter <= 4'd0;
    else if(neg_edge)
        bit_counter <= bit_counter + 4'd1;
end

always@(posedge clk)
begin
    if((cur_state == state_receiving) && sample)
        rx_shift <= {serial_in, rx_shift[9:1]};
end

always@(posedge clk or negedge reset)
begin
    if(reset == 1'b0)
        cur_state <= state_idle;
    else
        cur_state <= next_state;
end

//Moore style FSM
always@(*)
begin
    case(cur_state)
        state_idle:begin
            if(!serial_in)
                next_state = state_receiving;
            else
                next_state = state_idle;
        end
        state_receiving:begin
            if(bit_counter != 4'd10)
                next_state = state_receiving;
            else
                next_state = state_received;
        end
        state_received:begin
            if(data_out_ready)
                next_state = state_idle;
            else
                next_state = state_received;
        end
        default:begin
            next_state = state_idle;
        end
    endcase
end
endmodule
