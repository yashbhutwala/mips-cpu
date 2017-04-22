`timescale 1ns/1ns
`include "mips.h"

module alu_tb;
   /*test ALU */
   //make inputs that change over time
   reg [31:0] in0;
   reg [31:0] in1;
   reg [31:0] in2;
   reg [31:0] in3;
   
   reg [2:0]  ALUop;
   
   initial begin
      ALUop = 000;
      in0 = 64;
      in1 = 72;
      in2 = 400;
      in3 = 300;
      
      #200 ALUop = 001;
      #200 ALUop = 010;
      #200 ALUop = 011;
      #200 ALUop = 100;
      #200 ALUop = 101;
      #200 ALUop = 110;
      #200 ALUop = 111;
   end // initial begin

   wire[31:0] output0;
   wire [31:0] output1;
   alu a0 (ALUop, in0, in1, output0);
   alu a1 (ALUop, in2, in3, output1);
   initial
     $monitor("At time %t \nALUop = %b \n in0 = %b (%0d)\n in1 = %b (%0d)\n  Output0 = %b (%0d) \n in2 = %b (%0d) \n in3 = %b (%0d) \n Output1 = %b (%0d)",
	      $time, ALUop,in0,in0, in1,in1, output0, output0,in2,in2, in3,in3,output1,output1);
endmodule // test
