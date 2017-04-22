`timescale 1ns/1ns
`include "mips.h"

module alu(input [2:0] aluOp, input [31:0] in0, input [31:0] in1, output reg [31:0] aluResult, output reg zero);
        initial begin
                aluResult <= 0;
                zero <= 0;
        end

        always@(aluOp, in0, in1) begin
                case (aluOp)
                        3'b000: aluResult = in0 & in1; //AND
                        3'b001: aluResult = in0 | in1; //OR
                        3'b010: aluResult = in0 + in1; //ADD
                        3'b110: aluResult = in0 + (~in1 + 1); //SUB
                        3'b111: aluResult = (in0 < in1)? 1:0;
                        default: aluResult <= 0;
                endcase // case (aluOp)
                zero <= (aluResult == 0)? 1:0;
        end // always@ begin
endmodule // alu


module alu_tb;
   /*test ALU */
   //make inputs that change over time
   reg [31:0] in0;
   reg [31:0] in1;
   reg [31:0] in2;
   reg [31:0] in3;
   wire zero;
   
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
   alu a0 (ALUop, in0, in1, output0, zero);
   alu a1 (ALUop, in2, in3, output1, zero);
   initial begin
     $monitor("At time %t \nALUop = %b \n in0 = %b (%0d)\n in1 = %b (%0d)\n  Output0 = %b (%0d) \n in2 = %b (%0d) \n in3 = %b (%0d) \n Output1 = %b (%0d)",
	      $time, ALUop,in0,in0, in1,in1, output0, output0,in2,in2, in3,in3,output1,output1);

   end
endmodule // test
