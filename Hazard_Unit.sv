`timescale 1ns / 1ps


module Hazard_Unit(
    input logic CLK,
    input logic RESET,
    input logic [4:0] Rs1D,
    input logic [4:0] Rs2D,
    input logic [4:0] RdD,
    input logic [4:0] Rs1E,
    input logic [4:0] Rs2E,
    input logic [4:0] RdE,
    input logic [2:0] PCSrcE,
    input logic MemReadE,
    input logic [4:0] RdM,
    input logic [4:0] RdW,
    input logic RegWriteM,
    input logic RegWriteW,
    output logic Stall,
    output logic FlushD,
    output logic FlushE,
    output logic [1:0] ForwardAE,
    output logic [1:0] ForwardBE,
    
    // new code.
    output logic [1:0] ForwardAD,
    output logic [1:0] ForwardBD,
    
    
    input logic mem_we_E,
    input logic mem_re_D,
    input logic [31:0] store_addr_E,  // Calculated store address in execute stage
    input logic [31:0] load_addr_D    // Calculated load address in decode stage
);

    logic load_use_stall;
    logic load_after_store_stall;
    logic branch_decode_stall;
    
    logic control_flush;
    logic stall_hazard;
    
    
always_comb begin
    // Default values
    ForwardAE = 2'b00;
    ForwardBE = 2'b00;
    Stall = 0;
    FlushE = 0;
    FlushD = 0;
    
//    // control hazard
//    if (PCSrcE != 3'b0) begin
//        //FlushE = 1;
//        FlushD = 1;
//    end 
      
//    load_use_stall = (MemReadE && RdE != 5'd0 && ((RdE == Rs1D) || (RdE == Rs2D)));
//    load_after_store_stall = (mem_we_E && mem_re_D && (store_addr_E == load_addr_D));
    
//    //new
//    branch_decode_stall = (RdE != 5'd0 && ((RdE == Rs1D) || (RdE == Rs2D)));

    
//    if (load_use_stall || load_after_store_stall || branch_decode_stall) begin
//        Stall   = 1;
//        FlushE  = 1;
//    end 

    // Control hazard (taken branch or jump): Flush D
    
    
    control_flush = (PCSrcE != 3'b0);
    
    // Stall due to data hazard
    load_use_stall        = (MemReadE && RdE != 5'd0 && ((RdE == Rs1D) || (RdE == Rs2D)));
    load_after_store_stall= (mem_we_E && mem_re_D && (store_addr_E == load_addr_D));
    branch_decode_stall   = (RdE != 5'd0 && ((RdE == Rs1D) || (RdE == Rs2D)));    
    stall_hazard = load_use_stall || load_after_store_stall || branch_decode_stall;
    Stall   = stall_hazard;
    FlushE  = stall_hazard;
    FlushD  = (!stall_hazard) && control_flush;

    
    
    
    // ForwardAE
    if (RegWriteM && RdM != 0 && RdM == Rs1E) begin
        ForwardAE = 2'b10;
    end else if (RegWriteW && RdW != 0 && RdW == Rs1E && !(RegWriteM && RdM == Rs1E)) begin
        ForwardAE = 2'b01;
    end else begin
        ForwardAE = 2'b00;
    end
    
    // ForwardBE
    if (RegWriteM && RdM != 0 && RdM == Rs2E) begin
        ForwardBE = 2'b10;
    end else if (RegWriteW && RdW != 0 && RdW == Rs2E && !(RegWriteM && RdM == Rs2E)) begin
        ForwardBE = 2'b01;
    end else begin
        ForwardBE = 2'b00;
    end
    
    
    
    
    // new logic
    
    // Decode stage forwarding (new logic)
    // ForwardAD - Forward to Rs1 in decode stage
    // Only forward from Memory or Writeback stages, not Execute (that would cause stall above)
    if (RegWriteM && RdM != 0 && RdM == Rs1D) begin
        ForwardAD = 2'b10;  // Forward from Memory stage (ex_mem.alu_res)
    end else if (RegWriteW && RdW != 0 && RdW == Rs1D && !(RegWriteM && RdM == Rs1D)) begin
        ForwardAD = 2'b01;  // Forward from Writeback stage (wb_data)
    end else begin
        ForwardAD = 2'b00;  // No forwarding, use register file output
    end
    
    // ForwardBD - Forward to Rs2 in decode stage
    // Only forward from Memory or Writeback stages, not Execute (that would cause stall above)
    if (RegWriteM && RdM != 0 && RdM == Rs2D) begin
        ForwardBD = 2'b10;  // Forward from Memory stage (ex_mem.alu_res)
    end else if (RegWriteW && RdW != 0 && RdW == Rs2D && !(RegWriteM && RdM == Rs2D)) begin
        ForwardBD = 2'b01;  // Forward from Writeback stage (wb_data)
    end else begin
        ForwardBD = 2'b00;  // No forwarding, use register file output
    end
   
   
end

    
    
    
//end      
endmodule
