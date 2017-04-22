/* Name: Yash Bhutwala
 * CS 320 Activity 6
 * 09/08/16
 */

`timescale 1ns/1ns
`include "mips.h"

/*
 * add4 module takes a 32 bit input and adds 4 to it.
 */
module add4(input[31:0] pcf, output reg[31:0] pcPlus4F);
	parameter pcInc = 4;
	always @(*) begin
		pcPlus4F <= pcf + pcInc;
	end
endmodule

/*
 * addBranch module.
 */
module addBranch(input[31:0] inAddr1, input[31:0] inAddr2, output[31:0] outAddr);
	assign outAddr = inAddr1 + inAddr2;
endmodule

/*
 * instMem module takes in an address and outputs an instruction.
 */
module instMem(input[29:0] readAddr, input isSyscallW, input[31:0] a0D, input[31:0] v0D, output reg[31:0] memOut);
	reg [31:0] mem[32'h100000 : 32'h100100];
	reg [29:0] a0Sliced;
	reg [32:0] tempbuffer1;
	reg [128:0] tempbuffer2;
	initial 
	begin
		memOut <= 0;
		//$readmemh("add_test.v", mem);	// for activity 6
		$readmemh("fib.v", mem);	// for helloWorld
	end
	always @(*)
	begin
		$monitor("read addr %08x", readAddr);	
		memOut = mem[readAddr];
		//$monitor("instruction = %32h", memOut);
		if (memOut ==0)
		begin
			$strobe("Found nll op at addr %08x %32h", readAddr,memOut);	
			//#150;
			//$finish();
		end
	end

	always@(*) begin
		if(isSyscallW == 1) begin
			//$strobe("syscall in instMem");
			//$strobe("v0 = %32h", v0D);
			//$strobe("a0 = %32h", a0D);
			if(v0D == 1)
				$display("a0 = %32h", a0D);
			if (v0D == 4)
				// parse through the string
				a0Sliced = a0D[31:2];

				while (mem[a0Sliced] != 0) begin
					tempbuffer2 = tempbuffer2 << 32;					
					tempbuffer1 = {mem[a0Sliced][7:0], mem[a0Sliced][15:8], mem[a0Sliced][23:16], mem[a0Sliced][32:24]};
					tempbuffer2 = tempbuffer2 + tempbuffer1;
					a0Sliced = a0Sliced + 1;
					$display("%s",tempbuffer2);				
				end
				$display("%s",tempbuffer2);
				$monitor("%s",tempbuffer2);
				$strobe("%s",tempbuffer2);

			if (v0D == 10) begin
				$display("exit");
				#100;
				$finish;
			end
		end	
	end

endmodule

/*
 * mux2To1 module takes in two inputs, a sel input and ouputs one output based
 * on the select bit.
 * in1 = 1
 * in2 = 0
 */
module mux2To1(input[31:0] in1, input[31:0] in2, input sel, output[31:0] out);
	assign out = (sel) ? in1 : in2;	
endmodule

/*
 * mux2To1_5Bits module is for 5 bit inputs
 */
module mux2To1_5Bits(input[4:0] in1, input[4:0] in2, input sel, output[4:0] out);
	assign out = (sel) ? in1 : in2;
endmodule

/*
 * mux3To1 takes in three inputs, a sel input and outputs 31 bit output.
 */
module mux3To1(input[31:0] in1, input[31:0] in2, input[31:0] in3, input[1:0] sel, output[31:0] out);
	reg[31:0] tempOut;
	assign out = tempOut;
	always @(sel or in1 or in2 or in3) begin
		if (sel == 2'b00)  tempOut <= in1;
		else if (sel == 2'b01) tempOut <= in2;
		else if (sel == 2'b10) tempOut <= in3;
		else tempOut <= 0;
	end
endmodule

/*
 * control module outputs the control signals. 
 */
module control(input[5:0] opcode, input[5:0] funct, 
		output reg jump, output reg regDst,
                output reg branch, output reg memRead, output reg memToReg,
                output reg [2:0] aluOp, output reg regWrite,
                output reg aluSrc, output reg memWrite,
		output reg syscall,
		output reg jumpReg,
		output reg jumpAndLink,
		output reg lui);
        
	initial begin
		jump <= 0;
		regDst <= 0;
		branch <= 0;
		memRead <= 0;
		memToReg <= 0;
		aluOp <= 0;
		regWrite <= 0;
		aluSrc <= 0;
		memWrite <= 0;
		syscall <= 0;
		jumpReg <= 0;
		jumpAndLink <= 0;
		lui <= 0;
	end

        always@(*)
        begin
	//$display("opcode: %32h", opcode);
	//$display("funct: %32h", funct);
                case(opcode)
			`SPECIAL: begin
				case (funct)
					`ADD, `ADDU: begin 
						aluOp <= 3'b010; 
						regDst <= 1; 
						regWrite <= 1;
						$display("ADD");

						jump <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpReg <= 0;
						jumpAndLink <= 0;
						lui <= 0; 
					end
					`JR: begin 
						jumpReg <= 1;
						jump <= 1; 
						$display("JR"); 
				
						regDst <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluOp <= 0;
						regWrite <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpAndLink <= 0;
						lui <= 0;
					end
					`SYSCALL: begin 
						syscall <= 1; 
						$display("SYSCALL");
						
						jump <= 0;
						regDst <= 0;
                                                branch <= 0;
                                                memRead <= 0;
                                                memToReg <= 0;
						aluOp <= 0;
						regWrite <= 0;
                                                aluSrc <= 0;
                                                memWrite <= 0;
                                                jumpReg <= 0;
                                                jumpAndLink <= 0;
                                                lui <= 0; 
					end
					/*TODO Later.
                        		`SUB: begin 
                        			aluOp <= 3'b110; 
                        			regDst <= 1; 
                        			regWrite <= 1; 
                        			$display("SUB");
                        			
                        			jump <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpReg <= 0;
						jumpAndLink <= 0;
						lui <= 0;
					end
                        		`AND: begin 
						aluOp <= 3'b000; 
						regDst <= 1; 
						regWrite <= 1; 
						$display("AND"); 
						
						jump <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluOp <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpReg <= 0;
						jumpAndLink <= 0;
						lui <= 0;					
					end
                        		`OR:  begin 
						aluOp <= 3'b001; 
						regDst <= 1; 
						regWrite <= 1; 
						$display("OR"); 
				
						jump <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpReg <= 0;
						jumpAndLink <= 0;
						lui <= 0;							
					end
                        		`SLT: begin 
						aluOp <= 3'b111; 
						regDst <= 1; 
						regWrite <= 1; 
						$display("SLT"); 
						
						jump <= 0;
						branch <= 0;
						memRead <= 0;
						memToReg <= 0;
						aluSrc <= 0;
						memWrite <= 0;
						syscall <= 0;
						jumpReg <= 0;
						jumpAndLink <= 0;
						lui <= 0;	
					end
					*/
					default: begin 
						$display("Default");
						jump <= 0;
                                		regDst <= 0;
                                		branch <= 0;
                                		memRead <= 0;
                                		memToReg <= 0;
                                		aluOp <= 0;
                                		regWrite <= 0;
                                		aluSrc <= 0;
                                		memWrite <= 0;
                                		syscall <= 0;
                                		jumpReg <= 0;
                                		jumpAndLink <= 0;
                                		lui <= 0; 
					end
				endcase
			end
                        `ADDI, `ADDIU: begin 
				aluOp <= 3'b010; 
				regWrite <= 1; 
				aluSrc <= 1; 
				$display("ADDI"); 
				
				jump <= 0; 
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                memWrite <= 0;
                                syscall <= 0;
				jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0;
			end
                        `ORI: begin 
				aluOp <= 3'b001; 
				regWrite <= 1; 
				aluSrc <= 1; 
				$display("ORI"); 
				
				jump <= 0;
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0;
			end
                        `LW: begin 
				memRead <= 1; 
				memToReg <= 1; 
				aluOp <= 3'b010; 
				regWrite <= 1; 
				aluSrc <= 1; 
				$display("LW"); 
				
				jump <= 0;
                                regDst <= 0;
                                branch <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0;
			end
                        `SW: begin 
				aluOp <= 3'b010; 
				aluSrc <= 1; 
				memWrite <= 1; 
				$display("SW"); 
				
				jump <= 0;
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                regWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0;
			end
			`J: begin 
				jump <= 1; 
				$display("J");
			
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                aluOp <= 0;
                                regWrite <= 0;
                                aluSrc <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0; 
			end
			`LUI: begin 
				lui <= 1; 
				aluOp <= 3'b010; 
				regWrite <= 1; 
				aluSrc <= 1; 
				$display("LUI");
				
				jump <= 0;
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
			end
			`JAL: begin 
				jump <= 1;
				jumpAndLink <= 1; 
				$display("JAL");
                                
				regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                aluOp <= 0;
                                regWrite <= 0;
                                aluSrc <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                lui <= 0;
			end
                        /* TODO Later. 
			`BEQ: begin 
				branch <= 1; 
				aluOp <= 110; 
				$display("BRANCH");
	
				jump <= 0;
                                regDst <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                regWrite <= 0;
                                aluSrc <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0; 
			end
			*/
                        default: begin 
				$display("Default");
				jump <= 0;
                                regDst <= 0;
                                branch <= 0;
                                memRead <= 0;
                                memToReg <= 0;
                                aluOp <= 0;
                                regWrite <= 0;
                                aluSrc <= 0;
                                memWrite <= 0;
                                syscall <= 0;
                                jumpReg <= 0;
                                jumpAndLink <= 0;
                                lui <= 0; 
			end
                endcase
        end
endmodule

/*
 * alu takes as input the alu opcode, and two 32 bit inputs, and outputs the aluResult.
 */
module alu(input [2:0] aluOp, input [31:0] in0, input [31:0] in1, output reg [31:0] aluResult, output reg zero);
	initial begin
		aluResult <= 0;
		zero <= 0;
	end

	always@(*) begin
		case (aluOp)
			3'b000: aluResult <= in0 & in1; //AND
			3'b001: aluResult <= in0 | in1; //OR
			3'b010: aluResult <= in0 + in1; //ADD
			3'b110: aluResult <= in0 + (~in1 + 1); //SUB
			3'b111: aluResult <= (in0 < in1)? 1:0;
			default: aluResult <= 0;
		endcase // case (aluOp)
		zero <= (aluResult == 0)? 1:0;
	end // always@ begin
endmodule // alu

/*
 * signExtend takes a 16 bit value and sign extends it to 32 bits.
 */
module signExtend(input [15:0] extend, output reg [31:0] extendedOut);
	initial begin
		extendedOut <= 0;
	end	

	always @(extend) begin
		extendedOut <= {{16{extend[15]}},extend[15:0]};
	end
endmodule

/*
 * dataMem takes as input the address, the data to write, memRead, and
 * memWrite control bits. It outputs the outData.
 */
module dataMem(input clk, 
	       input [31:0]  inAddr, input[31:0] writeData,
               input 	     memRead, input memWrite, 
	       input [31:0]  a0, input[31:0] v0, input isSyscall,
	       
	       output [31:0] outData);
        reg [31:0] mem[0:1023];
        reg [9:0] k;
	initial begin
		for (k = 0; k < 1023; k = k+1) begin
			mem[k] <= 0;
		end
	end


	always @(posedge clk) begin
                if(memWrite) begin
                        mem[inAddr] <= writeData;
			//$display(mem[inAddr]);
		end
		
        end
        assign outData = (memRead == 1)? (mem[inAddr]):0;
endmodule

/* 
 * regFile takes as input: clk, addresses of register 1 and register 2,
 * address of write register, regWrite control bit, and the data to write.
 * It outputs the data read from register 1 and register 2.
 */
module regFile(input clk, input [4:0] readReg1Addr, input [4:0] readReg2Addr, 
		input [4:0] writeRegAddr, input regWrite, input [31:0] writeData, input jumpAndLink,
		output reg [31:0] reg1Data, output reg [31:0] reg2Data,
		output reg [31:0] a0, output reg [31:0] v0);
        reg [31:0] regFile[0:31];
	reg [4:0] k;

	initial begin
		for (k = 0; k < 31; k = k+1) begin
			regFile[k] <= 0;
		end
		reg1Data <= 0;
		reg2Data <= 0;
		a0 <= 0;
		v0 <= 0;
	end

        always@(negedge clk) begin
                if (regWrite) begin
                        regFile[writeRegAddr] = writeData;
		end
		if(jumpAndLink) begin
			regFile[`ra] = writeData;
			//$monitor("ra value = %32h", regFile[`ra]);
		end
		reg1Data = regFile[readReg1Addr];
        	reg2Data = regFile[readReg2Addr];
		a0 = regFile[`a0];
		v0 = regFile[`v0];			
		//$monitor("t0: %32h t1: %32h a0: %32h v0: %32h", regFile[`t0], regFile[`t1], regFile[`a0], regFile[`v0]);
	end
endmodule

/*
 *
 */
module PRW (input clk,
	   input 	     RegWriteM, 
	   input 	     MemtoRegM, 
	   input [31:0]      ReadDataM,
	   input [31:0]      ALUOutM,
	   input [4:0]	     WriteRegM,
	   input [31:0] a0M,
	   input [31:0] v0M,
	   input isSyscallM,	  
 
	   output reg 	     RegWriteW, 
	   output reg 	     MemtoRegW,
	   output reg [31:0] ReadDataW,
	   output reg [31:0] ALUOutW,
	   output reg [4:0]  WriteRegW,
	   output reg [31:0] a0W,
	   output reg [31:0] v0W,
	   output reg 	     isSyscallW
	   );
	initial begin
	   RegWriteW <= 0;
	   MemtoRegW <= 0;
	   ReadDataW <= 32'b0;
	   ALUOutW <= 32'b0;
	   WriteRegW <= 5'b0;
	   a0W <= 32'b0;
	   v0W <= 32'b0;
	   isSyscallW <= 0;
	end
	always @(posedge clk) begin
	   RegWriteW <= RegWriteM;
	   MemtoRegW <= MemtoRegM;
	   ReadDataW <= ReadDataM;
	   ALUOutW <= ALUOutM;
	   WriteRegW <= WriteRegM;
	   a0W <= a0M;
	   v0W <= v0M;
	   isSyscallW <= isSyscallM;
	end
endmodule // plRegW

/*
 *
 */
module PRF (input clk,
	    input [31:0] PC,
	    input enable,
	    output reg [31:0] PCF
	   );
	initial begin
		//PCF = 32'h400020;	// for activity 6 
		PCF = 32'h400030;	// for helloWorld
	end
	always@(posedge clk) begin
		if (~enable)
			PCF <= PC;
	end
endmodule // PRF

/*
 *
 */
module PRM (input clk, 
	    input 	 RegWriteE, 
	    input 	 MemtoRegE, 
	    input 	 MemWriteE, 
	    input [31:0] ALUOutE,
	    input [31:0] WriteDataE,
	    input [4:0] WriteRegE,
	    input [31:0] a0E,
	    input [31:0] v0E,
	    input isSyscallE,
	    
	    output reg 	 RegWriteM, 
	    output reg 	 MemtoRegM, 
	    output reg 	 MemWriteM,
	    output reg [31:0] ALUOutM,
	    output reg [31:0] WriteDataM,
	    output reg [4:0] WriteRegM,
	    output reg [31:0] a0M,
	    output reg [31:0] v0M,
	    output reg isSyscallM
	    );
	initial begin
	   RegWriteM <= 0;
	   MemtoRegM <= 0;
	   MemWriteM <= 0;
	   ALUOutM <= 32'b0;
	   WriteDataM <= 32'b0;
	   WriteRegM <= 5'b0;
	   a0M <= 32'b0;
	   v0M <= 32'b0;
	   isSyscallM <= 0;
	end
	always @(posedge clk) begin
	   RegWriteM <= RegWriteE;
	   MemtoRegM <= MemtoRegE;
	   MemWriteM <= MemWriteE;
	   ALUOutM <= ALUOutE;
	   WriteDataM <= WriteDataE;
	   WriteRegM <= WriteRegE;
	   a0M <= a0E;
	   v0M <= v0E;
	   isSyscallM <= isSyscallE;
	end
endmodule // PRM

/*
 *
 */
module PRE(	      input 		clk,
	              input 		flush,
		      input 		RegWriteD, 
	              input 		MemtoRegD,
	              input 		MemWriteD,
	              input [2:0] 	ALUControlD,
	              input 		ALUSrcD,
	              input 		RegDstD,
		      input [31:0] 	reg1DataD,
		      input [31:0] 	reg2DataD,
		      input [4:0] 	rsD,
		      input [4:0] 	rtD,
		      input [4:0] 	rdD,
		      input [31:0] 	signImmD,
		      input [31:0]	a0D,
		      input [31:0]	v0D,
		      input 		isSyscallD,

	              output reg 	RegWriteE,
	              output reg 	MemtoRegE,
	              output reg 	MemWriteE,
	              output reg [2:0] 	ALUControlE,
		      output reg 	ALUSrcE,
		      output reg 	RegDstE,
		      output reg [31:0]	reg1DataE,
		      output reg [31:0]	reg2DataE,
		      output reg [4:0] 	rsE,
		      output reg [4:0] 	rtE,
		      output reg [4:0] 	rdE,
		      output reg [31:0] signImmE,
		      output reg [31:0] a0E,
		      output reg [31:0] v0E,
		      output reg 	isSyscallE
	   );

	initial begin
	     RegWriteE <= 0;
	     MemtoRegE <= 0;
	     MemWriteE <= 0;
	     ALUControlE <= 2'b0;
	     ALUSrcE <= 0;
	     RegDstE <= 0;
	     reg1DataE <= 32'b0;
	     reg2DataE <= 32'b0;
	     rsE <= 5'b0;
	     rtE <= 5'b0;
	     rdE <= 5'b0;
	     signImmE <= 32'b0;
	     a0E <= 32'b0;
	     v0E <= 32'b0;
	     isSyscallE <= 32'b0;
	end // initial begin

	always @(posedge clk) begin
		if(flush) begin
	     		RegWriteE <= 0;
	     		MemtoRegE <= 0;
	     		MemWriteE <= 0;
	     		ALUControlE <= 2'b0;
	     		ALUSrcE <= 0;
	     		RegDstE <= 0;
	     		reg1DataE <= 32'b0;
	     		reg2DataE <= 32'b0;
	     		rsE <= 5'b0;
	    		rtE <= 5'b0;
	     		rdE <= 5'b0;
	     		signImmE <= 32'b0;
	     		a0E <= 32'b0;
	     		v0E <= 32'b0;
	     		isSyscallE <= 32'b0;
	     	end // if (flush)
	     	
		else begin
      	     		RegWriteE <= RegWriteD;
      	     		MemtoRegE <= MemtoRegD;
      	     		MemWriteE <= MemWriteD;
      	     		ALUControlE <= ALUControlD;
	     		ALUSrcE <= ALUSrcD;
	     		RegDstE <= RegDstD;
	     		reg1DataE <= reg1DataD;
	     		reg2DataE <= reg2DataD;
	     		rsE <= rsD;
	     		rtE <= rtD;
	     		rdE <= rdD;
	     		signImmE <= signImmD;
	     		a0E <= a0D;
	     		v0E <= v0D;
	     		isSyscallE <= isSyscallD;	     
   		end
	end
endmodule // PRE

/*
 *
 */
module PRD(input clk, input [31:0] PCPlus4F , input  [31:0] InstrF,
	              input      Enable, input  clear,
		      output reg [31:0] InstrD, output reg [31:0] PCPlus4D);
      initial begin
	   InstrD <= 32'b0;
	   PCPlus4D <= 32'b0;
	end
      always @(posedge clk) begin
	           if(clear) begin
			InstrD <= 32'b0;
			PCPlus4D <= 32'b0;
		   end
	           if(~Enable) begin
			InstrD <= InstrF;
			PCPlus4D <= PCPlus4F;
		   end
	end // always @ (posedge clk)
endmodule // PRD

/*
 *
 */
module hazard(	input [4:0] rsD, input [4:0] rtD, input [4:0] rsE, input [4:0] rtE, 
		input [4:0] writeRegE, input [4:0] writeRegM, input [4:0] writeRegW, 
		input regWriteE, input regWriteM, input regWriteW, input memToRegE, 
		input memToRegM, input branchD, 

		output reg forwardAD, output reg forwardBD, output reg [1:0] forwardAE, 
		output reg [1:0] forwardBE, output reg stallF, 
		output reg stallD, output reg flushE);

   reg lwstallD; reg branchstallD;
   always@(*) begin
      //forwarding sources to D stage (branch equality)
      forwardAD <= (rsD != 0) & (rsD == writeRegM) & regWriteM;
      forwardBD <= (rtD != 0) & (rtD == writeRegM) & regWriteM;
   end
   always@(*) begin
     //forwarding sources to E stage (ALU)
	forwardAE <= 2'b00; forwardBE = 2'b00;
	if(rsE != 0)
          if (rsE == writeRegM & regWriteM)
	    forwardAE <= 2'b10;
	  else if (rsE == writeRegW & regWriteW)
	    forwardAE <= 2'b01;
	if (rtE != 0)
	  if(rtE == writeRegM & regWriteM)
	    forwardBE <= 2'b10;
	  else if (rtE == writeRegW & regWriteW)
	    forwardBE <= 2'b01;
   end

   always@(*) begin
      //stalls
      lwstallD <= memToRegE & (rtE == rsD | rtE == rtD);
      branchstallD <= branchD & (regWriteE & (writeRegE == rsD | writeRegE == rtD) | memToRegM & (writeRegM == rsD | writeRegM == rtD));
      stallD <= lwstallD | branchstallD;
      stallF <= stallD;
      flushE <= stallD;
   end
endmodule

/*
 *
 */
module topModule(input clk); 
	// FETCH Wires	
	wire stallF; 
	wire [31:0] instrF; 
	wire [31:0] pcPlus4F;
	
	// DECODE Wires
	wire stallD;
	wire forwardAD; 
	wire forwardBD;
	
	wire regWriteD; 
	wire memToRegD; 
	wire memWriteD; 
	wire [2:0] aluControlD;
   	wire aluSrcD;
	wire regDstD;

	wire pcSrcD;
	wire [31:0] pcBranchD;	
	wire branchD;

	wire [31:0] rd1D; 
	wire [31:0] rd2D;

	wire [4:0] rsD; 
	wire [4:0] rtD; 
	wire [4:0] rdD;

	wire [31:0] signImmD;

	wire [31:0] a0D; 
	wire [31:0] v0D; 
	wire isSyscallD;
	
	wire jumpD;
	wire [31:0] pcJumpD;
	// EXECUTE Wires
	wire flushE;
	wire [1:0] forwardAE; 
	wire [1:0] forwardBE; 
	
	wire regWriteE;
	wire memToRegE;
	wire memWriteE;
	wire [31:0] aluOutE;
	wire [31:0] writeDataE;
	
	wire [4:0]  rsE;
	wire [4:0]  rtE;
	wire [4:0]  writeRegE;
	wire [31:0] a0E; 
	wire [31:0] v0E; 
	wire isSyscallE;

	// MEMORY Wires
	wire regWriteM;
	wire memToRegM;
	wire [31:0] aluOutM;
	wire [4:0] writeRegM;
	wire [31:0] readDataM;	
	wire [31:0] a0M; 
	wire [31:0] v0M; 
	wire isSyscallM;

	// WRITEBACK Wires
	wire [4:0] writeRegW;
	wire [31:0] resultW;
 	wire regWriteW;
	wire [31:0] a0W;
	wire [31:0] v0W;
	wire isSyscallW;	
	
	fetch myFetch ( 
		// Inputs
		.clk(clk),
		.stallF(stallF),
		.pcSrcD(pcSrcD),
		.pcBranchD(pcBranchD),
	    	.a0D(a0D), .v0D(v0D), .isSyscallW(isSyscallW),
		.jumpD(jumpD),.pcJumpD(pcJumpD),
		// Outputs
		.instrF(instrF),
		.pcPlus4F(pcPlus4F)
		);
 
	decode myDecode(
		// Inputs
		.clk(clk),
		.stallD(stallD),
		.instrF(instrF),
	    	.pcPlus4F(pcPlus4F),
	    	.forwardAD(forwardAD), .forwardBD(forwardBD),
		.writeRegW(writeRegW),
	    	.resultW(resultW),
	    	.regWriteW(regWriteW),
	    	.aluOutM(aluOutM),
		// Outputs
	    	.regWriteD(regWriteD), .memToRegD(memToRegD), .memWriteD(memWriteD),
	    	.aluControlD(aluControlD), .aluSrcD(aluSrcD), .regDstD(regDstD),
	    	.pcSrcD(pcSrcD),
	    	.pcBranchD(pcBranchD), .branchD(branchD),
	    	.rd1D(rd1D), .rd2D(rd2D),
	    	.rsD(rsD), .rtD(rtD), .rdD(rdD),
	    	.signImmD(signImmD),
	    	.a0D(a0D), .v0D(v0D), .isSyscallD(isSyscallD),
		.jumpD(jumpD),.pcJumpD(pcJumpD)
		);	
	
	execute myExecute (
		// Inputs
		.clk(clk),
	     	.flushE(flushE),
	     	.regWriteD(regWriteD), .memToRegD(memToRegD), .memWriteD(memWriteD),
	     	.aluControlD(aluControlD), .aluSrcD(aluSrcD), .regDstD(regDstD),
	     	.rd1D(rd1D), .rd2D(rd2D),
	     	.rsD(rsD), .rtD(rtD), .rdD(rdD),
	     	.signImmD(signImmD),
	     	.forwardAE(forwardAE), .forwardBE(forwardBE),
	     	.resultW(resultW), .aluOutM(aluOutM),
	     	.a0D(a0D), .v0D(v0D), .isSyscallD(isSyscallD),
		// Outputs
	     	.regWriteE(regWriteE), .memToRegE(memToRegE), .memWriteE(memWriteE),
	     	.aluOutE(aluOutE), .writeDataE(writeDataE),
	     	.rsE(rsE), .rtE(rtE), .writeRegE(writeRegE),
	     	.a0E(a0E), .v0E(v0E), .isSyscallE(isSyscallE)
		);
	
	memory myMemory ( 
		// Inputs
		.clk(clk),
	    	.regWriteE(regWriteE), .memToRegE(memToRegE), .memWriteE(memWriteE),
	    	.aluOutE(aluOutE), .writeDataE(writeDataE),
	    	.writeRegE(writeRegE),
	    	.a0E(a0E), .v0E(v0E), .isSyscallE(isSyscallE),
		// Outputs
	    	.regWriteM(regWriteM), .memToRegM(memToRegM),
	    	.readDataM(readDataM), .aluOutM(aluOutM),
	    	.writeRegM(writeRegM),
	    	.a0M(a0M), .v0M(v0M), .isSyscallM(isSyscallM)
		);
	
	writeback myWriteback (
		// Inputs
		.clk(clk),
	       	.regWriteM(regWriteM), .memToRegM(memToRegM),
	       	.readDataM(readDataM), .aluOutM(aluOutM),
	       	.writeRegM(writeRegM),
	    	.a0M(a0M), .v0M(v0M), .isSyscallM(isSyscallM),
		// Outputs
	       	.writeRegW(writeRegW), .resultW(resultW),
	       	.regWriteW(regWriteW),
	    	.a0W(a0W), .v0W(v0W), .isSyscallW(isSyscallW)
		);
	 
	hazard myHazard (
		.rsD(rsD), .rtD(rtD), .rsE(rsE), .rtE(rtE),
		.writeRegE(writeRegE), .writeRegM(writeRegM), .writeRegW(writeRegW),
		.regWriteE(regWriteE), .regWriteM(regWriteM), .regWriteW(regWriteW),
		.memToRegE(memToRegE), .memToRegM(memToRegM), .branchD(branchD),

		.forwardAD(forwardAD), .forwardBD(forwardBD),
		.forwardAE(forwardAE), .forwardBE(forwardBE),
		.stallF(stallF), .stallD(stallD), .flushE(flushE)
		); 
endmodule

/*
 *
 */
module fetch(	
		input clk, 
		input stallF, 
		input pcSrcD, 
		input [31:0] pcBranchD, 
		input [31:0] a0D,
	    	input [31:0] v0D,
	    	input isSyscallW,
		input [31:0] pcJumpD,
		input jumpD,
		
		output[31:0] instrF, 
		output[31:0] pcPlus4F
		);
	
	wire [31:0] pcPlusOrBranch;
	wire [31:0] pcPrime; //Either pcPlus4F or pcBranchD
	wire  [31:0] pcf;  // comes from pipeline register. Changed if no stall
   
	//Mux to select the value of the PC
	mux2To1 jumpMux (
		.in1(pcJumpD),
		.in2(pcPlusOrBranch),
		.sel(jumpD),
			
		.out(pcPrime)
		);


	mux2To1 pcMux (
		 //Inputs
		 .in1(pcBranchD), 
		 .in2(pcPlus4F),
		 .sel(pcSrcD),
		 //Outputs
		 .out(pcPlusOrBranch)
		 );

	PRF myPRF (
		//Inputs
		.clk(clk),
		.PC(pcPrime),
		.enable(stallF),
		//Outputs
		.PCF(pcf)
		);
	
	add4 myAdd4 (
	    	//Input
	    	.pcf(pcf),
		//Output
	    	.pcPlus4F(pcPlus4F)
	    	);
   
	//inputs the high 30 bits of the PC register and outputs the instructions read from memory
	instMem myInstMem (
		//Input
		.readAddr(pcf[31:2]), 
		.a0D(a0D),
		.v0D(v0D),
		.isSyscallW(isSyscallW),
		//Output		
		.memOut(instrF)
		);
endmodule

/*
 *
 */
module decode(	
		input clk, 
		input stallD, 
		input [31:0] instrF, 
		input [31:0] pcPlus4F, 
		input forwardAD, input forwardBD, 
		input [4:0] writeRegW,
		input [31:0] resultW,
		input regWriteW,
		input [31:0] aluOutM,

		output regWriteD, output memToRegD, output memWriteD, 
		output [2:0] aluControlD, output aluSrcD, output regDstD,
		output pcSrcD,
		output [31:0] pcBranchD, output branchD, 
		output [31:0] rd1D, output[31:0] rd2D, 
		output [4:0] rsD, 
		output [4:0] rtD, 
		output [4:0] rdD, 
		output [31:0] signImmD,
		output [31:0] a0D, output [31:0] v0D, output isSyscallD,
		output jumpD, output [31:0] pcJumpD
		);

	wire [31:0] instrD;
	wire [31:0] pcPlus4D;
	assign rsD = instrD[`rs];
	assign rtD = instrD[`rt];
	assign rdD = instrD[`rd];

	wire [31:0] jumpAddr;
	assign jumpAddr = {pcPlus4D[31:28], instrD[`target], 2'b00};
	
	wire jumpAndLink;
	wire [31:0] jalMuxOut;
	wire jr;
	
	wire equalD;
	wire [31:0] rd1MuxOutD; wire [31:0] rd2MuxOutD;
	assign equalD = (rd1MuxOutD == rd2MuxOutD)? 1:0;
	assign pcSrcD = branchD & equalD;

	wire [31:0] extendD;	
	wire lui;

	PRD myPRD (
		// Inputs 
		.clk(clk), 
		.PCPlus4F(pcPlus4F) , 
		.InstrF(instrF),
		.Enable(stallD),
		.clear(pcSrcD),
		// Outputs
		.InstrD(instrD), //
		.PCPlus4D(pcPlus4D) //
		);

	regFile myRegFile (
		// Inputs
		.clk(clk), 
		.readReg1Addr(rsD), 
		.readReg2Addr(rtD), 
		.writeRegAddr(writeRegW), 
		.regWrite(regWriteW), 
		.writeData(jalMuxOut),
		.jumpAndLink(jumpAndLink),
		// Outputs  
		.reg1Data(rd1D), //
		.reg2Data(rd2D), //
		.a0(a0D), 
		.v0(v0D)
		);

	mux2To1 jalMux(
		.in1(pcPlus4D),
		.in2(resultW),
		.sel(jumpAndLink),
		.out(jalMuxOut)
		);
	
	mux2To1 jrMux (
		.in1 (rd1D),
		.in2 (jumpAddr),
		.sel (jr),
		
		.out (pcJumpD)
		);	

	mux2To1 rd1Mux (
		// Inputs
		.in1(aluOutM), //
		.in2(rd1D), 
		.sel(forwardAD), 
		// Outputs
		.out(rd1MuxOutD)
		);

	mux2To1 rd2Mux (
		// Inputs
		.in1(aluOutM), // 
		.in2(rd2D), 
		.sel(forwardBD), 
		// Outputs
		.out(rd2MuxOutD)
		);
	
	control myControl (
		// Inputs
		.opcode(instrD[`op]), 
		.funct(instrD[`function]),
		// Outputs 
		.jump(jumpD), 
		.regDst(regDstD),
                .branch(branchD), 
		//TODO: .memRead(), 
		.memToReg(memToRegD),
                .aluOp(aluControlD), 
		.regWrite(regWriteD),
                .aluSrc(aluSrcD), 
		.memWrite(memWriteD),
		.syscall(isSyscallD),
		.jumpReg(jr),
		.jumpAndLink(jumpAndLink),
		.lui(lui)
		);

	signExtend mySignExtend (
		// Input
		.extend(instrD[`immediate]),
		// Output
		.extendedOut(extendD)
		);
	
	mux2To1 luiMux (
		.in1({instrD[`immediate], 16'b0}),
		.in2(extendD),

		.sel(lui),
		.out(signImmD)
	);


	addBranch myAddBranch (
		// Inputs
		.inAddr1({signImmD[29:0], 2'b00}),
		.inAddr2(pcPlus4D),
		// Output
		.outAddr(pcBranchD)
		);
endmodule

/*
 *
 */
module execute(	
		input clk, 
		input flushE, 
		input regWriteD, input memToRegD, input memWriteD, 
		input [2:0] aluControlD, input aluSrcD, input regDstD, 
		input [31:0] rd1D, 
		input [31:0] rd2D, 
		input [4:0] rsD, 
		input [4:0] rtD, 
		input [4:0] rdD, 
		input [31:0] signImmD,
		input [1:0] forwardAE, input [1:0] forwardBE,
		input [31:0] resultW, input [31:0] aluOutM,
		input [31:0] a0D, input[31:0] v0D, input isSyscallD,
		
		output regWriteE, output memToRegE, output memWriteE,
		output [31:0] aluOutE, 
		output [31:0] writeDataE,
		output [4:0] rsE,
		output [4:0] rtE, 
		output [4:0] writeRegE,
		output [31:0] a0E, output[31:0] v0E, output isSyscallE		
		);

	wire aluSrcE;
	wire regDstE;
	wire [31:0] rd1E;
	wire [31:0] rd2E;
	wire [4:0] rdE;
	wire [31:0] signImmE;
	wire [31:0] srcAE;
	wire [31:0] srcBE;
	wire [2:0] aluControlE;

	PRE myPRE (
		// Inputs
		.clk(clk),
		.flush(flushE),
		.RegWriteD(regWriteD),
		.MemtoRegD(memToRegD),
		.MemWriteD(memWriteD),
		.ALUControlD(aluControlD),
		.ALUSrcD(aluSrcD),
		.RegDstD(regDstD),
		.reg1DataD(rd1D),
		.reg2DataD(rd2D),
		.rsD(rsD),
		.rtD(rtD),
		.rdD(rdD),
		.signImmD(signImmD),
		.a0D(a0D),
		.v0D(v0D), 
		.isSyscallD(isSyscallD),
		// Outputs
		.RegWriteE(regWriteE),
		.MemtoRegE(memToRegE),
		.MemWriteE(memWriteE),
		.ALUControlE(aluControlE), //
		.ALUSrcE(aluSrcE), //
		.RegDstE(regDstE), // 
		.reg1DataE(rd1E), //
		.reg2DataE(rd2E), //
		.rsE(rsE),
		.rtE(rtE),
		.rdE(rdE), //
		.signImmE(signImmE), //
		.a0E(a0E),
		.v0E(v0E),
		.isSyscallE(isSyscallE)
		); 
	
	mux3To1 forwardAEMux (
		// Inputs
		.in1(rd1E),
		.in2(resultW),
		.in3(aluOutM),
		.sel(forwardAE),
		// Outputs
		.out(srcAE) //
		);

	mux3To1 forwardBEMux (
		// Inputs
		.in1(rd2E),
		.in2(resultW),
		.in3(aluOutM),
		.sel(forwardBE),
		// Outputs
		.out(writeDataE)
		);

	mux2To1 srcBEMux(
		// Inputs
		.in1(signImmE), 
		.in2(writeDataE), 
		.sel(aluSrcE), 
		// Outputs
		.out(srcBE)
		);

	alu myAlu(
		// Inputs
		.aluOp(aluControlE), 
		.in0(srcAE), 
		.in1(srcBE),
		// Outputs 
		.aluResult(aluOutE) 
		//TODO: .zero()
		);

	//this is a 5 bit mux
	mux2To1_5Bits writeRegEMux(
		// Inputs
		.in1(rdE), 
		.in2(rtE), 
		.sel(regDstE), 
		// Outputs
		.out(writeRegE)
		);
endmodule

/*
 *
 */
module memory(	
		input clk,
		input regWriteE, input memToRegE, input memWriteE,
		input [31:0] aluOutE, 
		input [31:0] writeDataE, 
		input [4:0] writeRegE,
		input [31:0] a0E, input[31:0] v0E, input isSyscallE,

		output regWriteM, output memToRegM, 
		output [31:0] readDataM,		
		output [31:0] aluOutM,
		output [4:0] writeRegM,
		output [31:0] a0M,
	    	output [31:0] v0M,
	    	output isSyscallM
		
		);

	wire memWriteM;
	wire [31:0] writeDataM;
	wire [31:0] a0M;
	wire [31:0] v0M;

	PRM myPRM(
		// Inputs
		.clk(clk),
		.RegWriteE(regWriteE),
		.MemtoRegE(memToRegE),
		.MemWriteE(memWriteE),
		.ALUOutE(aluOutE),
		.WriteDataE(writeDataE),
		.WriteRegE(writeRegE),
		.a0E(a0E),
		.v0E(v0E),
		.isSyscallE(isSyscallE),
		// Outputs
		.RegWriteM(regWriteM), 
		.MemtoRegM(memToRegM), 
		.MemWriteM(memWriteM), //
		.ALUOutM(aluOutM),
		.WriteDataM(writeDataM),  //
		.WriteRegM(writeRegM),
		.a0M(a0M),
		.v0M(v0M),
		.isSyscallM(isSyscallM)
		);

	dataMem myDataMem( 
		// Inputs
		.clk(clk), 
		.inAddr(aluOutM), 
		.writeData(writeDataM), 
		.memRead(1'b1), 
		.memWrite(memWriteM), 
		// Outputs
		.outData(readDataM)
		);
endmodule

/*
 *
 */
module writeback(  
		input clk,
		input regWriteM, input memToRegM, 
		input [31:0] readDataM,		
		input [31:0] aluOutM,
		input [4:0] writeRegM, 
		input [31:0] a0M, input[31:0] v0M, input isSyscallM,
		
		output [4:0] writeRegW,
		output [31:0] resultW,
		output regWriteW,
		output [31:0] a0W, output[31:0] v0W, output isSyscallW
		);

	wire [31:0] readDataW;
	wire [31:0] aluOutW;
	wire memToRegW;

	PRW myPRW (	
		// Inputs
		.clk(clk), 
		.RegWriteM(regWriteM), 
		.MemtoRegM(memToRegM), 
		.ReadDataM(readDataM), 
		.ALUOutM(aluOutM), 
		.WriteRegM(writeRegM),
		.a0M(a0M),
		.v0M(v0M),
		.isSyscallM(isSyscallM),

		// Outputs 
		.RegWriteW(regWriteW), 
		.MemtoRegW(memToRegW), 
		.ReadDataW(readDataW), 
		.ALUOutW(aluOutW), 
		.WriteRegW(writeRegW),
		.a0W(a0W),
		.v0W(v0W),
		.isSyscallW(isSyscallW)
		);

	mux2To1 wrMux (
		// Inputs
		.in1(readDataW), 
		.in2(aluOutW), 
		.sel(memToRegW), 
		// Outputs
		.out(resultW)
		);
endmodule


