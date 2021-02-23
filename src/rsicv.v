`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: francisco flynn
// 
// Create Date:    17:49:34 12/19/2020 
// Design Name: 
// Module Name:    rsicv 
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
`include "common_define.h"

//used for forwarding logic
`define FORWORD_ID_EX	2'b00
`define FORWORD_EX_MEM	2'b10
`define FORWORD_MEM_WB	2'b01

module rsicv#(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
)(
    input clk,
    input rst,
    input FPGA_SERIAL_RX,
    output FPGA_SERIAL_TX,
    output reg[31:0] wd
);
// Memories
wire [11:0] bios_addra, bios_addrb;
wire [31:0] bios_douta, bios_doutb;
wire bios_ena, bios_enb;
wire [13:0] dmem_addr;
wire [31:0] dmem_din, dmem_dout;
wire [3:0] dmem_we;
wire dmem_en;
wire [31:0] imem_dina, imem_doutb;
wire [13:0] imem_addra, imem_addrb;
wire [3:0] imem_wea;
wire imem_ena;
wire RegWen;
wire [1:0] WBSel;
wire [4:0] ra1, ra2, wa;
//reg  [31:0] wd;
wire [31:0] rd1, rd2;
wire BrUn, BrLt, BrEq; 
wire [31:0] alu_a, alu_b, alu_forward_a, alu_forward_b, alu_out;
wire ASel, BSel;
wire[3:0] ALUSel;
reg [31:0] PC, inst, inst_little;
wire [31:0] imm_dout;
wire [2:0] ImmSel;
wire PCSel;
wire [3:0] MemRW;

//pipeline register
reg [31:0] F_D_PC, D_E_PC, E_M_PC;
reg [31:0] D_E_rd1, D_E_rd2, E_M_rd2;
reg [31:0] D_E_inst, E_M_inst, M_W_inst;
reg [31:0] E_M_alu;
reg [31:0] M_W_wd;
//pipeline register to implement forwarding logic
reg [4:0]	D_E_ra1, D_E_ra2, D_E_wa;
reg [4:0]	E_M_wa;
reg [4:0]	M_W_wa;
reg [1:0]	Forward_A, Forward_B;
//pipeline control register
// Signal E_ASel, E_BSel, E_ImmSel, E_ALUSel will
// be create in Decode stage, and consume in Execute stage, 
// others will flow to next stage.
reg E_ASel, E_BSel;
reg [2:0] E_ImmSel;
reg [3:0] E_ALUSel;
reg [3:0] E_MemRW;
reg [1:0] E_WBSel;
reg E_RegWen;
//M_MemRW and M_WBSel will be consumed in Memory stage,
//M_RegWen flow to next stage.
reg [3:0] M_MemRW;
reg [1:0] M_WBSel;
reg M_RegWen;
//The last signal W_RegWen will be consumed in Write
//back stage.
reg W_RegWen;

assign bios_ena = 1;
assign bios_enb = 1;
assign bios_addra = F_D_PC[11:0];
assign bios_addrb = alu_out[11:0];
assign dmem_addr = alu_out[13:0];
assign dmem_din = E_M_rd2;
assign dmem_we = M_MemRW;
assign dmem_en = 1;
assign imem_ena = 1;
assign imem_addra = alu_out[13:0];
assign imem_addrb = F_D_PC[13:0];
assign imem_dina = E_M_rd2;
assign imem_wea = M_MemRW;
assign ra1 = inst[19:15];
assign ra2 = inst[24:20];
assign wa = inst[11:7];
//assign inst = (F_D_PC[30]==1'b1) ? bios_douta:imem_doutb;
assign alu_a = (E_ASel==`ASel_reg) ? D_E_rd1:D_E_PC;
assign alu_b = (E_BSel==`BSel_reg) ? D_E_rd2:imm_dout;

always@(posedge clk)
begin
	D_E_ra1 <= ra1;
end

always@(posedge clk)
begin
	D_E_ra2 <= ra2;
end

always@(posedge clk)
begin
	D_E_wa <= wa;
end

/* forwarding unit */
always@(*)
begin
	if(M_RegWen && E_M_wa != 5'b0 && E_M_wa == D_E_ra1)
		Forward_A = `FORWORD_EX_MEM;
	else if(W_RegWen && M_W_wa != 5'b0 && M_W_wa == D_E_ra1)
		Forward_A = `FORWORD_MEM_WB;
	else
		Forward_A = `FORWORD_ID_EX;
end

always@(*)
begin
	if(M_RegWen && E_M_wa != 5'b0 && E_M_wa == D_E_ra2)
		Forward_B = `FORWORD_EX_MEM;
	else if(W_RegWen && M_W_wa != 5'b0 && M_W_wa == D_E_ra2)
		Forward_B = `FORWORD_MEM_WB;
	else
		Forward_B = `FORWORD_ID_EX;
end

assign alu_forward_a = (Forward_A==`FORWORD_EX_MEM)?E_M_alu:((Forward_A==`FORWORD_MEM_WB)?M_W_wd:alu_a);
assign alu_forward_b = (Forward_B==`FORWORD_EX_MEM)?E_M_alu:((Forward_B==`FORWORD_MEM_WB)?M_W_wd:alu_b);

always@(*)
begin
	inst_little = (F_D_PC[30]==1'b1) ? bios_douta:imem_doutb;
end

always@(*)
begin
	inst = {inst_little[7:0], inst_little[15:8], inst_little[23:16], inst_little[31:24]};
end

bios_mem bios_mem (
    .clk(clk),
    .ena(bios_ena),
    .addra(bios_addra),
    .douta(bios_douta),
    .enb(bios_enb),
    .addrb(bios_addrb),
    .doutb(bios_doutb)
);

dmem dmem (
    .clk(clk),
    .en(dmem_en),
    .we(dmem_we),
    .addr(dmem_addr),
    .din(dmem_din),
    .dout(dmem_dout)
);

imem imem (
    .clk(clk),
    .ena(imem_ena),
    .wea(imem_wea),
    .addra(imem_addra),
    .dina(imem_dina),
    .addrb(imem_addrb),
    .doutb(imem_doutb)
);

always@(*)
begin
    case (M_WBSel)
        `WBSel_mem: begin
            //we should also decode I/O memory, which MSB is 1
            if(E_M_PC[30] == 1'b1)
                wd = bios_doutb; 
            else if(E_M_PC[28] == 1'b1)
                wd = dmem_dout; 
            else
                wd = dmem_dout;
            end
        `WBSel_alu: wd = E_M_alu;
        `WBSel_pc_next: wd = E_M_PC + 31'd4;
        default: wd = E_M_alu;
    endcase
end

reg_file rf (
    .clk(clk),
    .we(W_RegWen),
    .ra1(ra1), .ra2(ra2), .wa(M_W_wa),
    .wd(M_W_wd),
    .rd1(rd1), .rd2(rd2)
);

branch_comparator br(
    .input1(rd1),
    .input2(rd2),
    .BrUn(BrUn),
    .BrLt(BrLt),
    .BrEq(BrEq)
);

alu alu_0(
    .inputA(alu_forward_a), 
    .inputB(alu_forward_b), 
    .ALUSel(E_ALUSel), 
    .out(alu_out)
);

imm_gen imm_gen0(
    .inst(D_E_inst),
    .immSel(E_ImmSel),
    .out(imm_dout)
);

control control_0(
    .instruction(inst), 
    .BrLt(BrLt), 
    .BrEq(BrEq), 
    .PCSel(PCSel), 
    .RegWen(RegWen), 
    .ImmSel(ImmSel), 
    .BrUn(BrUn), 
    .ASel(ASel), 
    .BSel(BSel), 
    .ALUSel(ALUSel), 
    .MemRW(MemRW), 
    .WBSel(WBSel)
);

always@(posedge clk or negedge rst)
begin
    if(rst == 1'b0)begin
        PC <= RESET_PC; 
    end else begin
        if(PCSel == `PCSel_next)
            PC <= PC + 31'd4;
        else
            PC <= E_M_alu;
    end
end

//update pipeline register
always@(posedge clk or negedge rst)
begin
    if(rst == 1'b0)begin
        F_D_PC  <= RESET_PC;
        D_E_PC  <= RESET_PC;
        D_E_rd1 <= 32'd0;
        D_E_rd2 <= 32'd0;
		D_E_ra1 <= 5'd0;
		D_E_ra2 <= 5'd0;
		D_E_wa  <= 5'd0;
		E_M_wa  <= 5'd0;
		M_W_wa  <= 5'd0;
        E_M_rd2 <= 32'd0;
        E_M_PC  <= RESET_PC;
        E_M_alu <= 32'd0;
        M_W_wd  <= 32'd0;
        D_E_inst<= 32'd0;
        E_M_inst<= 32'd0;
        M_W_inst<= 32'd0;
        E_ASel      <= 1'b0;
        E_BSel      <= 1'b0;
        E_ImmSel    <= 3'b0;
        E_ALUSel    <= 4'b0;
        E_MemRW     <= 4'b0;
        E_WBSel     <= 2'b0;
        E_RegWen    <= 1'b0;
        M_MemRW     <= 4'b0;
        M_WBSel     <= 2'b0;
        M_RegWen    <= 1'b0;
        W_RegWen    <= 1'b0;
    end else begin
        //fetch->Decode
        F_D_PC  <= PC;
        //Decode->Execute
        D_E_PC      <= F_D_PC;
        D_E_rd1     <= rd1;
        D_E_rd2     <= rd2;
        D_E_inst    <= inst;
        //--control signal
        E_ASel      <= ASel;
        E_BSel      <= BSel;
        E_ImmSel    <= ImmSel;
        E_ALUSel    <= ALUSel;
        E_MemRW     <= MemRW;
        E_WBSel     <= WBSel;
        E_RegWen    <= RegWen;
        //Execute->Memory
        E_M_PC      <= D_E_PC;
        E_M_rd2     <= D_E_rd2;
        E_M_alu     <= alu_out; 
        E_M_inst    <= D_E_inst;
		E_M_wa		<= D_E_wa;
        M_MemRW     <= E_MemRW;
        M_WBSel     <= E_WBSel;
        M_RegWen    <= E_RegWen;
        //Memory->Write back
        M_W_wd      <= wd;
		M_W_wa		<= E_M_wa;
        M_W_inst    <= E_M_inst;
        W_RegWen    <= M_RegWen;
    end
end
// On-chip UART

endmodule
