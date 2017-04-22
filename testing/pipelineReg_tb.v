`timescale 1ns/1ns

module test;
   initial
     begin
	reg RegWriteM = 0;
	reg MemtoRegM = 0;
	reg clk = 0;
	
     end
   reg 	    MemtoRegW;
   reg 	    RegWriteW;

   always clk = !clk;
   
   
   PRW prw (clk, RegWriteM, MemtoRegM, MemtoRegW, RegWriteW);
   
