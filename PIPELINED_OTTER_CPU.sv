`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Jack Marshall
//////////////////////////////////////////////////////////////////////////////////


typedef enum logic [6:0] {
       LUI      = 7'b0110111,
       AUIPC    = 7'b0010111,
       JAL      = 7'b1101111,
       JALR     = 7'b1100111,
       BRANCH   = 7'b1100011,
       LOAD     = 7'b0000011,
       STORE    = 7'b0100011,
       OP_IMM   = 7'b0010011,
       OP       = 7'b0110011,
       SYSTEM   = 7'b1110011
} opcode_t;
    
typedef struct packed {
    // data
    logic [31:0]    pc_jalr,
                    pc_branch,
                    pc_jal;
    logic [31:0]    instr, //current instruction
                    pc, // current pc
                    pc_4; // pc + 4
    logic [31:0]    rs1, //read out from regfile
                    rs2, //read out from regfile 2
                    rs1_addr,
                    rs2_addr,
                    utype, // generated immediates
                    itype,
                    stype,
                    jtype,
                    btype;
    logic [4:0]     rf_wa; //write address for regfile
    logic [31:0]    alu_res, //result of ALU
                    mem_data; //data read from memory
    logic           br_eq, br_lt, br_ltu;
    
    // ctrl
    logic [2:0]     pc_ctrl; //mux control for PC
    logic [2:0]     imm_ctrl; //mux control for Immediate Gen
    logic           rf_we; //write enable for regfile
    logic [1:0]     rf_wr_ctrl; //mux control for regfile
    logic [31:0]    alu_srcA, // ALU mux select
                    alu_srcB, // ALU mux select
                    alu_a, //a input to ALU
                    alu_b; //b input to ALU
    logic [3:0]     alu_ctrl; //mux control for ALU operation
    logic           mem_we, mem_re; //write enable to memory
} preg_t; //pipeline register

module PIPELINED_OTTER_CPU (
    input CLK,
    input INTR,
    input RESET,
    input [31:0] IOBUS_IN,
    output [31:0] IOBUS_OUT,
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR 
);     
  
// pipeline registers
preg_t if_de, de_ex, ex_mem, mem_wb;

// FETCH signals
wire [31:0] pc, pc_next, IR;
wire pcWrite;
wire [2:0] pc_ctrl;

// DECODE signals
wire [4:0] rs1_addr, rs2_addr, rd_addr;
wire [31:0] rs1_data, 
            rs2_data, 
            UType, 
            IType, 
            SType, 
            JType, 
            BType, 
            alu_srcA, 
            alu_srcB, 
            alu_A, 
            alu_B;
wire [2:0] imm_type;
wire rf_we, mem_we, mem_re;

// EXECUTE signals
wire [31:0] alu_result;
wire [31:0] branch_target, jalr_target, jal_target;
wire br_eq, br_lt, br_ltu;
wire [3:0] alu_func;

// MEMORY signals
wire mem_write, mem_read;
wire [31:0] mem_data_out;

// WRITEBACK signals
wire [31:0] wb_data;
wire [1:0] rf_wr_sel;

// control signals
wire [6:0] opcode;
assign opcode = if_de.instr[6:0];
opcode_t OPCODE;
assign OPCODE = opcode_t'(opcode);

// HAZARD signals
wire [1:0] ForwardAE, ForwardBE;
wire [31:0] SrcAE, SrcBE;
wire [1:0] ForwardAD, ForwardBD;
wire [31:0] SrcAD, SrcBD;
wire Stall;
wire FlushE, FlushD;
    
//==== Instruction Fetch ===========================================
 
assign pcWrite = ~Stall;
 
 // program counter
 PC OTTER_PC(
    .CLK(CLK), 
    .RST(RESET), 
    .PC_WRITE(pcWrite),
    .PC_SOURCE(pc_ctrl),
    .JALR(jalr_target),
    .JAL(jal_target), 
    .BRANCH(branch_target), 
    .MTVEC(32'b0), 
    .MEPC(32'b0),
    
    .PC_OUT(pc), 
    .PC_OUT_INC(pc_next)
    );
 
// instruction memory
imem imem (
    .imem_a(pc),
    .imem_out(IR)
    );

// IF/DE pipeline register
always_ff @(posedge CLK) begin
    if (RESET || FlushD) begin
        if_de <= '0;
        //if_de.pc <= pc; 
    end else if (!Stall) begin
        if_de.pc <= pc;
        if_de.instr <= IR;
        if_de.pc_4 <= pc + 4;
    end
end

 
//==== Instruction Decode ===========================================

// register file inputs
assign rs1_addr = if_de.instr[19:15];
assign rs2_addr = if_de.instr[24:20];
assign rd_addr = mem_wb.rf_wa;  // from MEM/WB pipeline register

// register file
REG_FILE register_file(
    .CLK(CLK),
    .EN(mem_wb.rf_we),          // write enable from WB stage
    .ADR1(rs1_addr),            // read during DE
    .ADR2(rs2_addr),            // read during DE
    .WA(rd_addr),               // write address from WB
    .WD(wb_data),               // data to write from WB mux
    
    .RS1(rs1_data),             // output to DE
    .RS2(rs2_data)              // output to DE
);

 // decoder
 CU_DCDR CU_DCDR (
    .IR_30(if_de.instr[30]),
    .IR_OPCODE(if_de.instr[6:0]),
    .IR_FUNCT(if_de.instr[14:12]),
    
    .ALU_FUN(alu_func),
    .ALU_SRCA(alu_srcA),
    .ALU_SRCB(alu_srcB),
    .RF_WR_SEL(rf_wr_sel),
    // added these new ones to account for FSM
    .REG_WRITE(rf_we),
    .MEM_WE(mem_we),
    .MEM_RDEN(mem_re)
 );
 
 // immed gen
 ImmediateGenerator ImmediateGenerator (
    .IR(if_de.instr),
    .U_TYPE(UType),
    .I_TYPE(IType),
    .S_TYPE(SType),
    .B_TYPE(BType),
    .J_TYPE(JType)
 );
 
 
 // selects RS1 or forwarded register value
 ThreeMux RD1D_mux (
    .SEL(ForwardAD),
    .ZERO(rs1_data),
    .ONE(wb_data),
    //.TWO(alu_res),
    .TWO(ex_mem.alu_res),
    .OUT(SrcAD)
 );
 
  // selects RS2 or forwarded register value
 ThreeMux RD2D_mux (
    .SEL(ForwardBD),
    .ZERO(rs2_data),
    .ONE(wb_data),
    //.TWO(alu_res),
    .TWO(ex_mem.alu_res),
    .OUT(SrcBD)
 );
 
 
// target gen. BAG
BAG BAG (
    .RS1(rs1_data),
    .I_TYPE(IType),
    .J_TYPE(JType),
    .B_TYPE(BType),
    .FROM_PC(if_de.pc),
    .JAL(jal_target),
    .JALR(jalr_target),
    .BRANCH(branch_target)
);

// branch cond gen. BCG
BCG BCG (
    .RS1(SrcAD),
    .RS2(SrcBD),
    .IR_OPCODE(if_de.instr[6:0]),
    .IR_FUNCT(if_de.instr[14:12]),
    .PC_SOURCE(pc_ctrl)
);


 // DE/EX pipeline register
 always_ff @(posedge CLK) begin
     if (RESET || FlushE) begin
        de_ex <= '0;
        //de_ex.pc <= if_de.pc; 
    end else if (!Stall) begin
        de_ex.pc <= if_de.pc;
        de_ex.instr <= if_de.instr;
        de_ex.pc_4 <= if_de.pc_4;
        
        de_ex.rs1 <= rs1_data;
        de_ex.rs2 <= rs2_data;
        de_ex.alu_ctrl <= alu_func;
        de_ex.alu_srcA <= alu_srcA;
        de_ex.alu_srcB <= alu_srcB;
        de_ex.rf_wa <= if_de.instr[11:7];
        de_ex.utype <= UType;
        de_ex.itype <= IType;
        de_ex.stype <= SType;
        de_ex.jtype <= JType;
        de_ex.btype <= BType;
        de_ex.rf_wr_ctrl <= rf_wr_sel;
        de_ex.rf_we <= rf_we;
        de_ex.mem_we <= mem_we;
        de_ex.mem_re <= mem_re;
        de_ex.rs1_addr <= rs1_addr;
        de_ex.rs2_addr <= rs2_addr;

    end
 end



//==== Execute ======================================================
 
 // selects RS1 or forwarded register value
 ThreeMux RD1E_mux (
    .SEL(ForwardAE),
    .ZERO(de_ex.rs1),
    .ONE(wb_data),
    .TWO(ex_mem.alu_res),
    .OUT(SrcAE)
 );
 
  // selects RS2 or forwarded register value
 ThreeMux RD2E_mux (
    .SEL(ForwardBE),
    .ZERO(de_ex.rs2),
    .ONE(wb_data),
    .TWO(ex_mem.alu_res),
    .OUT(SrcBE)
 );
 
 // alu src A mux
 TwoMux alu_srcA_mux (
    .SEL(de_ex.alu_srcA),
    .ZERO(SrcAE),
    .ONE(de_ex.utype),
    .OUT(alu_A)
 );
 
 // alu src B mux
 FourMux alu_srcB_mux (
    .SEL(de_ex.alu_srcB),
    .ZERO(SrcBE),
    .ONE(de_ex.itype),
    .TWO(de_ex.stype),
    .THREE(de_ex.pc),
    .OUT(alu_B)
 );
  
 // ALU
ALU ALU (
    .ALU_FUN(de_ex.alu_ctrl), 
    .SRC_A(alu_A), 
    .SRC_B(alu_B), 
    .RESULT(alu_result)
);

//// target gen. BAG
//BAG BAG (
//    .RS1(de_ex.rs1),
//    .I_TYPE(de_ex.itype),
//    .J_TYPE(de_ex.jtype),
//    .B_TYPE(de_ex.btype),
//    .FROM_PC(de_ex.pc),
//    .JAL(jal_target),
//    .JALR(jalr_target),
//    .BRANCH(branch_target)
//);

//// branch cond gen. BCG
//BCG BCG (
//    .RS1(SrcAE),
//    .RS2(SrcBE),
//    .IR_OPCODE(de_ex.instr[6:0]),
//    .IR_FUNCT(de_ex.instr[14:12]),
//    .PC_SOURCE(pc_ctrl)
//);


always_ff @(posedge CLK) begin
    if (RESET) begin
        ex_mem <= '0;
    end 
    else begin
        ex_mem.pc <= de_ex.pc;
        ex_mem.instr <= de_ex.instr;
        ex_mem.pc_4 <= de_ex.pc_4;
        
        ex_mem.rs1 <= de_ex.rs1;
        ex_mem.rs2 <= SrcBE;
        ex_mem.alu_a <= alu_A;
        ex_mem.alu_b <= alu_B;
        ex_mem.rf_wa <= de_ex.rf_wa;
        ex_mem.utype <= de_ex.utype;
        ex_mem.itype <= de_ex.itype;
        ex_mem.stype <= de_ex.stype;
        ex_mem.jtype <= de_ex.jtype;
        ex_mem.btype <= de_ex.btype;
        ex_mem.pc_ctrl <= de_ex.pc_ctrl;
        ex_mem.rf_wr_ctrl <= de_ex.rf_wr_ctrl;
        ex_mem.rf_we <= de_ex.rf_we;
        ex_mem.mem_we <= de_ex.mem_we;
        ex_mem.mem_re <= de_ex.mem_re;
        ex_mem.alu_ctrl <= de_ex.alu_ctrl;
        
        ex_mem.alu_res <= alu_result;
        ex_mem.pc_jal <= jal_target;
        ex_mem.pc_jalr <= jalr_target;
        ex_mem.pc_branch <= branch_target;
        ex_mem.br_eq <= br_eq;
        ex_mem.br_lt <= br_lt;
        ex_mem.br_ltu <= br_ltu;
    end
end


//==== Memory ======================================================
 
assign IOBUS_ADDR = ex_mem.alu_res;
assign IOBUS_OUT = ex_mem.rs2;
assign IOBUS_WR = ex_mem.mem_we;


// data memory
dmem dmem (
    .clk(CLK),
    .we(ex_mem.mem_we),
    .dmem_addr(ex_mem.alu_res),
    .dmem_data(ex_mem.rs2),
    .dmem_out(mem_data_out)
);


// MEM/WB pipeline register
always_ff @(posedge CLK) begin
    if (RESET) begin
        mem_wb <= '0;
    end else begin
        mem_wb.pc <= ex_mem.pc;
        mem_wb.instr <= ex_mem.instr;
        mem_wb.pc_4 <= ex_mem.pc_4;
        mem_wb.rf_wa <= ex_mem.rf_wa;
        mem_wb.alu_res <= ex_mem.alu_res;
        mem_wb.mem_data <= mem_data_out;
        mem_wb.rf_we <= ex_mem.rf_we;
        mem_wb.rf_wr_ctrl <= ex_mem.rf_wr_ctrl;
    end
end

 
//==== Write Back ==================================================
  
// register file and register mux 
FourMux reg_file_mux (
    .SEL(mem_wb.rf_wr_ctrl),
    .ZERO(mem_wb.pc_4),
    
    // need CSR register for this
    .ONE(32'b0),
    
    .TWO(mem_wb.mem_data),
    .THREE(mem_wb.alu_res),
    .OUT(wb_data)
 );
 
 
 //==== Hazard Detection ==================================================
 
wire [31:0] store_addr_E, load_addr_D;

// Calculate store address in execute stage (for stores)
assign store_addr_E = (de_ex.mem_we) ? (SrcAE + de_ex.stype) : 32'b0;

// Calculate load address in decode stage (for loads)  
assign load_addr_D = (mem_re) ? (rs1_data + IType) : 32'b0;

 
 Hazard_Unit Hazard_Unit (
    .RESET(RESET),
    .CLK(CLK),
    .Rs1D(rs1_addr),
    .Rs2D(rs2_addr),
    .RdD(),
    .Rs1E(de_ex.rs1_addr),
    .Rs2E(de_ex.rs2_addr),
    .RdE(de_ex.rf_wa),
    .PCSrcE(pc_ctrl),
    .MemReadE(de_ex.mem_re),
    .RdM(ex_mem.rf_wa),
    .RdW(mem_wb.rf_wa),
    .RegWriteM(ex_mem.rf_we),
    .RegWriteW(mem_wb.rf_we),
    .Stall(Stall),
    .FlushD(FlushD),
    .FlushE(FlushE),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE),
    .ForwardAD(ForwardAD),
    .ForwardBD(ForwardBD),
    
    .mem_we_E(de_ex.mem_we),
    .mem_re_D(mem_re),
    .store_addr_E(store_addr_E),
    .load_addr_D(load_addr_D)
 );
 
 
 
 
 
 
 
      
endmodule
