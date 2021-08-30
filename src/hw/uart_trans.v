`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2021 07:48:03 PM
// Design Name: 
// Module Name: uart_trans
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


module uart_trans #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    input [7:0] data_in,
    input data_in_valid,
    output data_in_ready,

    output serial_out
);

localparam  SYMBOL_EDGE_TIME    =   CLOCK_FREQ / BAUD_RATE;
localparam  CLOCK_COUNTER_WIDTH =   $clog2(SYMBOL_EDGE_TIME);
localparam  state_idle = 0;
localparam  state_sending = 1;

reg [CLOCK_COUNTER_WIDTH-1:0] clock_counter;
reg [3:0] bit_counter;
reg [9:0] tx_shift = 10'b1;
reg [7:0] data_in_latch;
reg cur_state, next_state;
reg pose_edge_delay_one;
wire pose_edge;

assign serial_out = tx_shift[0];
assign pose_edge = clock_counter == SYMBOL_EDGE_TIME;
assign data_in_ready = cur_state != state_sending;

always@(posedge clk)
begin
    if(pose_edge)
        pose_edge_delay_one <= 1;
    else
        pose_edge_delay_one <= 0;
end

always@(posedge clk)
begin
    if(cur_state != state_sending)
        bit_counter <= 0;
    else if(pose_edge)
        bit_counter <= bit_counter + 1;
end

/* clock_counter would NOT be rounded when reach to SYMBOL_EDGE_TIME */
always@(posedge clk)
begin
    if(cur_state != state_sending)
        clock_counter <= 0;
    else if(clock_counter > SYMBOL_EDGE_TIME)
        clock_counter <= 0;
    else
        clock_counter <= clock_counter + 1;
end

always@(posedge clk)
begin
    if(cur_state == state_idle)
        tx_shift <= 10'b11_1111_1111;
    else if(cur_state == state_sending && !bit_counter)
        tx_shift <= {1'b1, data_in_latch, 1'b0};
    /* 
    * we use one clock delayed pose edge to start shifting,
    * otherwise, we would miss the first shift
    */
    else if(cur_state == state_sending && pose_edge_delay_one)
        tx_shift <= {1'b1, tx_shift[9:1]};
end

always@(posedge clk or negedge reset)
begin
    if(reset == 1'b0)
        cur_state <= state_idle;
    else
        cur_state <= next_state;
end

always@(posedge clk)
begin
    if(cur_state == state_idle && data_in_valid)
        data_in_latch <= data_in;
end

always@(*)
begin
    case(cur_state)
    state_idle:begin
        if(data_in_valid) begin
            next_state = state_sending;
        end else begin
            next_state = state_idle;
        end
    end
    state_sending:begin
        if(bit_counter != 4'd10)
            next_state = state_sending;
        else
            next_state = state_idle;
    end
    default:begin
       next_state = state_idle;
    end
    endcase
end

endmodule
