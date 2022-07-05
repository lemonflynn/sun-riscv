`timescale 1ns / 1ps
module vga(
    input                 clk,           //pixel clock
    input                 rst,           //reset signal high active
    output                hs,            //horizontal synchronization
    output                vs,            //vertical synchronization
    output                de,            //video valid
    output reg [7:0]      rgb_r,         //video red data
    output reg [7:0]      rgb_g,         //video green data
    output reg [7:0]      rgb_b          //video blue data
);

`define VIDEO_1280_720

//video timing parameter definition
//1280x720 74.25Mhz
`ifdef  VIDEO_1280_720
parameter H_ACTIVE = 16'd1280;           //horizontal active time (pixels)
parameter H_FP = 16'd110;                //horizontal front porch (pixels)
parameter H_SYNC = 16'd40;               //horizontal sync time(pixels)
parameter H_BP = 16'd220;                //horizontal back porch (pixels)
parameter V_ACTIVE = 16'd720;            //vertical active Time (lines)
parameter V_FP  = 16'd5;                 //vertical front porch (lines)
parameter V_SYNC  = 16'd5;               //vertical sync time (lines)
parameter V_BP  = 16'd20;                //vertical back porch (lines)
parameter HS_POL = 1'b1;                 //horizontal sync polarity, 1 : POSITIVE,0 : NEGATIVE;
parameter VS_POL = 1'b1;                 //vertical sync polarity, 1 : POSITIVE,0 : NEGATIVE;
`endif

//1920x1080 148.5Mhz
`ifdef  VIDEO_1920_1080
parameter H_ACTIVE = 16'd1920;
parameter H_FP = 16'd88;
parameter H_SYNC = 16'd44;
parameter H_BP = 16'd148;
parameter V_ACTIVE = 16'd1080;
parameter V_FP  = 16'd4;
parameter V_SYNC  = 16'd5;
parameter V_BP  = 16'd36;
parameter HS_POL = 1'b1;
parameter VS_POL = 1'b1;
`endif

parameter H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;
parameter V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;

parameter WHITE_R       = 8'hff;
parameter WHITE_G       = 8'hff;
parameter WHITE_B       = 8'hff;
parameter YELLOW_R      = 8'hff;
parameter YELLOW_G      = 8'hff;
parameter YELLOW_B      = 8'h00;
parameter CYAN_R        = 8'h00;
parameter CYAN_G        = 8'hff;
parameter CYAN_B        = 8'hff;
parameter GREEN_R       = 8'h00;
parameter GREEN_G       = 8'hff;
parameter GREEN_B       = 8'h00;

reg[11:0] h_cnt;
reg[11:0] v_cnt;
reg[11:0] h_pos;
reg[11:0] v_pos;
reg hs_reg;
reg vs_reg;
wire h_video_active;
wire v_video_active;

assign h_video_active = (h_cnt > H_FP + H_SYNC + H_BP - 1) ? 1 : 0;
assign v_video_active = (v_cnt > V_FP + V_SYNC + V_BP - 1) ? 1 : 0;
assign de = h_video_active & v_video_active;
assign hs = hs_reg;
assign vs = vs_reg;

always@(posedge clk)
begin
    if(rst)
        h_cnt <= 12'd0;
    else
        h_cnt <= (h_cnt > H_TOTAL - 1) ? 12'b0 : h_cnt + 1;
end

always@(posedge clk)
begin
    if(rst) begin
        v_cnt <= 12'd0;
    end else begin
        if(h_cnt == H_TOTAL - 1)
            v_cnt <= (v_cnt > V_TOTAL - 1) ? 12'b0 : v_cnt + 1;
        else
            v_cnt <= v_cnt;
    end
end

always@(posedge clk)
begin
    if(rst)
        h_pos <= 12'd0;
    else if(h_cnt >= H_FP + H_SYNC + H_BP - 1)
        h_pos <= h_cnt - H_FP - H_SYNC - H_BP + 1;
    else
        h_pos <= 12'd0;
end

always@(posedge clk)
begin
    if(rst)
        v_pos <= 12'd0;
    else if(v_cnt >= V_FP + V_SYNC + V_BP - 1)
        v_pos <= v_cnt - V_FP - V_SYNC - V_BP + 1;
    else
        v_pos <= 12'd0;
end

always@(posedge clk)
begin
    if(rst)
        hs_reg <= ~HS_POL;
    else if(h_cnt < H_FP - 1)
        hs_reg <= ~HS_POL;
    else if(h_cnt < H_FP + H_SYNC - 1)
        hs_reg <= HS_POL;
    else
        hs_reg <= ~HS_POL;
end

always@(posedge clk)
begin
    if(rst)
        vs_reg <= ~VS_POL;
    else if(v_cnt < V_FP - 1)
        vs_reg <= ~VS_POL;
    else if(v_cnt <= V_FP + V_SYNC - 1)
        vs_reg <= VS_POL;
    else
        vs_reg <= ~VS_POL;
end

//draw the color rectangle
always@(posedge clk)
begin
    if(de) begin
        if(v_pos <= (V_ACTIVE/2)) begin
            if(h_pos <= (H_ACTIVE/2)) begin
                rgb_r <= WHITE_R;
                rgb_g <= WHITE_G;
                rgb_b <= WHITE_B;
            end else begin
                rgb_r <= YELLOW_R;
                rgb_g <= YELLOW_G;
                rgb_b <= YELLOW_B;
            end
        end else begin
            if(h_pos <= (H_ACTIVE/2)) begin
                rgb_r <= CYAN_R;
                rgb_g <= CYAN_G;
                rgb_b <= CYAN_B;
            end else begin
                rgb_r <= GREEN_R;
                rgb_g <= GREEN_G;
                rgb_b <= GREEN_B;
            end
        end
    end else begin
        rgb_r <= 8'd0;
        rgb_g <= 8'd0;
        rgb_b <= 8'd0;
    end
end

endmodule
