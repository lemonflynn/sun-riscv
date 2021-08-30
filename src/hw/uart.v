module uart #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    input [7:0] data_in,
    input data_in_valid,
    output data_in_ready,

    output [7:0] data_out,
    output data_out_valid,
    input data_out_ready,

    input serial_in,
    output serial_out
);
    wire serial_in_reg;
    wire serial_out_tx;
    assign serial_out = !reset ? 1'b1 : serial_out_tx;
    assign serial_in_reg = !reset ? 1'b1 : serial_in;

    uart_trans #(
        .CLOCK_FREQ(CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uatransmit (
        .clk(clk),
        .reset(reset),
        .data_in(data_in),
        .data_in_valid(data_in_valid),
        .data_in_ready(data_in_ready),
        .serial_out(serial_out_tx)
    );

    uart_recv #(
        .CLOCK_FREQ(CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uareceive (
        .clk(clk),
        .reset(reset),
        .data_out(data_out),
        .data_out_valid(data_out_valid),
        .data_out_ready(data_out_ready),
        .serial_in(serial_in_reg)
    );
endmodule
