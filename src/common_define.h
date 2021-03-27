`ifndef OPCODE
`define OPCODE

//second most significant bit in func7, concatenate with 3 bits in func3
`define op_add     4'b0_000
`define op_sub     4'b1_000
`define op_sll     4'b0_001//shift left logic
`define op_slt     4'b0_010//set if less than
`define op_sltu    4'b0_011//set if less than unsigned
`define op_xor     4'b0_100
`define op_srl     4'b0_101//shift right logic
`define op_sra     4'b1_101//shift right arithmetic
`define op_or      4'b0_110
`define op_and     4'b0_111
//srl: padding zero on the MSB.
//sra: padding sign bit on the MSB.

//register type
`define R_type  5'b01100
// Loads are encoded in the I-type format and stores are S-type.
`define I_type     5'b00100
`define Load_type  5'b00000
`define S_type     5'b01000
`define B_type     5'b11000
`define JAL_type   5'b11011
`define JALR_type  5'b11001
`define AUIPC_type 5'b00101
`define LUI_type   5'b01101
`define CSR_type   5'b11100

// Load and store function codes
`define FNC_LB     3'b000
`define FNC_LH     3'b001
`define FNC_LW     3'b010
`define FNC_LBU    3'b100
`define FNC_LHU    3'b101
`define FNC_SB     3'b000
`define FNC_SH     3'b001
`define FNC_SW     3'b010

//Control signal from control module
`define ImmSel_I       3'd0
`define ImmSel_S       3'd1
`define ImmSel_B       3'd2
`define ImmSel_J       3'd3
`define ImmSel_U       3'd4
`define ImmSel_CSR     3'd5
`define PCSel_next     1'b0
`define PCSel_alu      1'b1
`define CSRSel_reg     1'b0
`define CSRSel_imm     1'b1
`define ASel_reg       0
`define ASel_pc        1
`define BSel_reg       0
`define BSel_imm       1
`define WBSel_mem      2'd0
`define WBSel_alu      2'd1
`define WBSel_pc_next  2'd2
`define WBSel_csr      2'd3
`define MemRead        4'b0000
`define MemWrite_B     4'b0001 //store byte
`define MemWrite_H     4'b0011 //store half word
`define MemWrite_W     4'b1111 //store word
`endif
