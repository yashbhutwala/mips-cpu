`timescale 

module topModule();

   reg clk = 0;
   always clk = ~clk;

   PRW myPRW(clk, RegWriteM, MemtoRegM, RegWriteW, MemtoRegW);
   PRM myPRM(clk, RegWriteE, MemtoRegE, MemWriteE, RegWriteM, MemtoRegM, MemWriteM);
   PRE myPRE(clk, flush, RegWriteD, MemtoRegD, MemWriteD, ALUControlD, ALUSrcD, RegDstD, BranchD,RegWriteE, MemtoRegE, MemWriteE, ALUControlE);
   
   
   
   
   
   hazard myHazard(input clk
