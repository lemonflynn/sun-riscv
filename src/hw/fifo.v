module fifo #(
    parameter data_width = 8,
    parameter fifo_depth = 32, 
    parameter addr_width = $clog2(fifo_depth)
) (
    input clk, reset,

    // Write side
    input wr_en,
    input [data_width-1:0] din,
    output full,

    // Read side
    input rd_en,
    output [data_width-1:0] dout,
    output empty
);
reg[data_width-1:0] memory[fifo_depth-1:0];
reg[addr_width-1:0] write_index, read_index;
wire [addr_width-1:0] write_next;

assign write_next = write_index + 1;
assign full = write_next == read_index;
assign empty = read_index == write_index;
assign dout = rd_en ? memory[read_index] : 0;

always@(posedge clk)
begin
    if(reset == 1'b0) begin
        write_index <= 0; 
    end else if(wr_en && !full) begin
        memory[write_index] <= din;
        write_index <= write_index + 1;
    end
end
always@(posedge clk)
begin
    if(reset == 1'b0)
        read_index <= 0; 
    else if(rd_en && !empty)
        read_index <= read_index + 1;
end
endmodule
