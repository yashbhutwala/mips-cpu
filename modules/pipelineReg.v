module PRW(input clk,input RegWriteM, input MemtoRegM, output RegWriteW, output MemtoRegW);
   initial
     begin
	RegWriteW = 0;
	MemtoRegW = 0;
     end
   always @(posedge clk)
     begin
	RegWriteW = RegWriteM;
	MemtoRegW = MemtoRegM;
     end
endmodule // plRegW


module PRM (input clk, input RegWriteE, input MemtoRegE, input MemWriteE, output RegWriteM, output MemtoRegM, output MemWriteM);
   initial
     begin
	RegWriteM = 0;
	MemtoRegM = 0;
	MemWriteM = 0;
     end
   always @(posedge clk)
     begin
	RegWriteM = RegWriteE;
	MemtoRegM = MemtoRegE;
	MemWriteM = MemWriteE;
     end
endmodule // PRM

module PRE(
	   input clk,
	   input flush, 	   
	   input RegWriteD, 
	   input MemtorRegD, 
	   input MemWriteD, 
	   input [2:0] ALUControlD, 
	   input ALUSrcD,
	   input RegDstD,
	   input BranchD,
	   output 	   RegWriteE,
	   output 	   MemtoRegE,
	   output 	   MemWriteE,
	   output [2:0]    ALUControlE
	   );
   initial
     begin
	RegWriteE = 0;
	MemtoRegE = 0;
	MemWriteE = 0;
	ALUControlE = 2'b0;
     end
   always @(posedge clk)
     if(flush)
       begin
	  RegWriteE = 0;
	  MemtoRegE = 0;
	  MemWriteE = 0;
	  ALUControlE = 2'b0;	  
       end
     begin
	RegWriteE = RegWriteD;
	MemtoRegE = MemtoRegD;
	MemWriteE = MemWriteD;
	ALUControlE = ALUControlD;
     end
endmodule // PRE

module PRD(input clk, input [31:0] PCPlus4F , input [31:0] InstrF, 
	   input Enable, input clear, 
	   output [31:0] InstrD, output [31:0] PCPlus4D);
   initial
     begin
	InstrD =32'b0;
	PCPlus4D = 32'b0;
     end
   always @(posedge clk)
     begin
	if(clear)
	  begin
	     InstrD = 32'b0;
	     PCPlus4D = 32'b0;
	  end
	if(Enable)
	  begin
	     InstrD = InstrF;
	     PCPlus4D = PCPlus4F;
	  end
     end // always @ (posedge clk)
endmodule // PRD
