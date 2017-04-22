/* Name: Yash Bhutwala
 * CS 320 Activity 6
 * 09/08/16
 */

`timescale 1ns/1ns
`include "mips.h"


/*
 * topModule for activity 6
 *
 */
module topModule(input clk);
//module topModule(input clk, output[31:0] currAddr, output[31:0] nextAddr, output[31:0] instruction);
	wire[31:0] currAddr; wire[31:0] nextAddr; wire[31:0] instruction;
	wire[31:0] pcPlus4;
	wire jump; wire regDst;	wire branch; wire memRead; wire memToReg;
	wire[2:0] aluOp; wire regWrite; wire aluSrc; wire memWrite;
	wire syscall; wire[31:0] a0; wire[31:0] v0;
	wire jumpReg;
	wire jumpAndLink;
	wire lui;
	wire[31:0] writeRegAddr; wire[31:0] reg1Data; wire[31:0] reg2Data; wire[31:0] writeData;
	wire[31:0] signExtendOut;
	wire[31:0] signExtendMuxOut;
	wire[31:0] aluOut;
	wire[31:0] dataMemOut;
	wire[31:0] addBranchResult;
	wire[31:0] forJumpMux;
	wire zero;

	pc myPc(clk, nextAddr, currAddr);
	add4 myAdd4(currAddr, pcPlus4);
	instMem myMem(currAddr[31:2], instruction);
	
	control myControl(instruction[`op], instruction[`function], jump, regDst, branch, memRead, memToReg, aluOp, regWrite, aluSrc, memWrite, syscall, jumpReg, jumpAndLink, lui);
	mux2To1 regMux({27'b0, instruction[`rd]}, {27'b0, instruction[`rt]}, regDst, writeRegAddr); 
	regFile myRegFile(clk, instruction[`rs], instruction[`rt], writeRegAddr[4:0], regWrite, writeData, reg1Data, reg2Data, a0, v0);

	signExtend mySe(instruction[`immediate], signExtendOut);
	addBranch myAddBranch(pcPlus4, {signExtendOut[29:0], 2'b00}, addBranchResult);
	mux2To1 branchMux(addBranchResult, pcPlus4, (zero && branch), forJumpMux);
	mux2To1 jumpMux({pcPlus4[31:28], instruction[`target], 2'b00}, forJumpMux, jump, nextAddr);
	
	mux2To1 aluMux(signExtendOut, reg2Data, aluSrc, signExtendMuxOut);
	alu myAlu(aluOp, reg1Data, signExtendMuxOut, aluOut, zero);

	dataMem myDataMem(clk, aluOut, reg2Data, memRead, memWrite, a0, v0, syscall, dataMemOut);
	mux2To1 dataMemMux(dataMemOut, aluOut, memToReg, writeData);
endmodule


/*
 * pc module updates the program counter on the positive edge of the clock
 * with a 5 ns delay.
 *
 */
module pc(input clk, input[31:0] nextPC, output[31:0] currPC);
	reg[31:0] tmp;
	initial tmp = 32'h400020; 	// for activity 6
	//initial tmp = 32'h400030;	// for helloWorld
	assign #5 currPC = tmp;		// 5 ns latency
	always @(posedge clk)
		tmp = nextPC;
endmodule

/*
 * add4 module takes a 32 bit input and adds 4 to it with a 100 ns delay.
 */
module add4(input[31:0] in, output[31:0] out);
	parameter pcInc = 4;
	reg[31:0] tmp;
	assign #100 out = tmp;
	always @(*)
		tmp = in + pcInc;
endmodule

/*
 * addBranch module with 100 ns delay.
 */
module addBranch(input[31:0] inAddr1, input[31:0] inAddr2, output[31:0] outAddr);
	assign #100 outAddr = inAddr1 + inAddr2;
endmodule

/*
 * instMem module takes in an address and outputs an instruction with 250 ns
 * delay.
 */
module instMem(input[29:0] readAddr, output[31:0] memOut);
	reg [31:0] mem[32'h100000 : 32'h100100];
	reg [31:0] tmp;
	assign #250 memOut = tmp;
	initial 
	begin
		$readmemh("add_test.v", mem);	// for activity 6
		//$readmemh("hello.v", mem);		// for helloWorld
	end
	always @(*)
	begin
		tmp = mem[readAddr];
		if (memOut == 0)
		begin
			$strobe("Found null op at addr %08x.", readAddr);
			$finish();
		end
	end
endmodule

/*
 * mux2To1 module takes in two inputs, a sel input and ouputs one output based
 * on the select bit with 30 ns delay.
 */
module mux2To1(input[31:0] in1, input[31:0] in2, input sel, output[31:0] out);
	assign #30 out = (sel) ? in1 : in2;	
endmodule

/*
 * control module outputs the control signals with 100 ns delay 
 */
module control(input[5:0] opcode, input[5:0] funct, output jump, output regDst,
                output branch, output memRead, output memToReg,
                output [2:0] aluOp, output regWrite,
                output aluSrc, output memWrite,
		output syscall,
		output jumpReg,
		output jumpAndLink,
		output lui);
        reg tempJump = 1'bx;
	reg tempRegDst = 1'bx;
	reg tempBranch = 1'bx;
	reg tempMemRead = 1'bx;
	reg tempMemToReg = 1'bx;
	reg [2:0] tempAluOp = 3'bx;
	reg tempRegWrite = 1'bx;
	reg tempAluSrc = 1'bx;
	reg tempMemWrite = 1'bx;
	reg tempSyscall = 1'bx;
	reg tempJumpReg = 1'bx;
	reg tempJumpAndLink = 1'bx;
	reg tempLui = 1'bx;

	assign #100 jump = tempJump;
	assign #100 regDst = tempRegDst;
	assign #100 branch = tempBranch;
	assign #100 memRead = tempMemRead;
	assign #100 memToReg = tempMemToReg;
	assign #100 aluOp = tempAluOp;
	assign #100 regWrite = tempRegWrite;
	assign #100 aluSrc = tempAluSrc;
	assign #100 memWrite = tempMemWrite;
	assign #100 syscall = tempSyscall;
	assign #100 jumpReg = tempJumpReg;
	assign #100 jumpAndLink = tempJumpAndLink;
	assign #100 lui = tempLui;

        always@(*)
        begin
                case(opcode)
			`SPECIAL: 
			begin
				case (funct)
					`ADD: begin tempAluOp = 3'b010; tempRegDst = 1; tempRegWrite = 1; $display("ADD"); end
                        		//`SUB: begin tempAluOp = 3'b110; tempRegDst = 1; tempRegWrite = 1; $display("SUB"); end
                        		//`AND: begin tempAluOp = 3'b000; tempRegDst = 1; tempRegWrite = 1; $display("AND"); end
                        		//`OR:  begin tempAluOp = 3'b001; tempRegDst = 1; tempRegWrite = 1; $display("OR"); end
                        		//`SLT: begin tempAluOp = 3'b111; tempRegDst = 1; tempRegWrite = 1; $display("SLT"); end
					`JR: begin tempJumpReg = 1; $display("JR"); end
					`SYSCALL: begin tempSyscall = 1; $display("SYSCALL"); end
				endcase
			end
                        `ADDI, `ADDIU: begin tempAluOp = 3'b010; tempRegWrite = 1; tempAluSrc = 1; $display("ADDI"); end
                        `ORI: begin tempAluOp = 3'b001; tempRegWrite = 1; tempAluSrc = 1; $display("ORI"); end
                        `LW: begin tempMemRead = 1; tempMemToReg = 1; tempAluOp = 3'b010; tempRegWrite = 1; tempAluSrc = 1; $display("LW"); end
                        `LUI: begin tempLui = 1; tempAluOp = 3'b010; tempRegWrite = 1; tempAluSrc = 1; $display("LUI"); end
                        `SW: begin tempAluOp = 3'b010; tempAluSrc = 1; tempMemWrite = 1; $display("SW"); end
                        //`BEQ, `BNE: begin tempBranch = 1; tempAluOp = 110; $display("BRANCH"); end
                        //`J: begin tempJump = 1; $display("J"); end
			`JAL: begin tempJumpAndLink = 1; $display("JAL"); end
                        default: begin tempJump = 0; tempRegDst = 0; tempBranch = 0; tempMemRead = 0; tempMemToReg = 0; tempAluOp = 0; tempRegWrite = 0; tempAluSrc = 0; tempMemWrite = 0; tempSyscall = 0; tempJumpReg = 0; tempJumpAndLink = 0; tempLui = 0; end
                endcase
        end
endmodule



/*
 * alu takes as input the alu opcode, and two 32 bit inputs, and outputs the
 * aluResult with latency of 120 ns.
 *
 */
module alu(input [2:0] aluOp, input [31:0] in0, input [31:0] in1, output [31:0] aluResult, output zero);
	reg[31:0] tempAluResult;
	reg tempZero = 1'bx;
	assign #120 aluResult = tempAluResult;
	assign #120 zero = tempZero;
	always@(*) begin
		case (aluOp)
			000: tempAluResult = in0 & in1; //AND
			001: tempAluResult = in0 | in1; //OR
			010: tempAluResult = in0 + in1; //ADD
			110: tempAluResult = in0 + (~in1 + 1); //SUB
			111: tempAluResult = (in0 < in1)? 1:0;
			default: tempAluResult = 0;
		endcase // case (aluOp)
		tempZero = (tempAluResult == 0)? 1:0;
	end // always@ begin
endmodule // alu

/*
 * signExtend takes a 16 bit value and sign extends it to 32 bits
 */
module signExtend(input [15:0] extend, output [31:0] extendedOut);
	reg [31:0] tmp;
	assign extendedOut = tmp;
	always @(extend) begin
		tmp = {{16{extend[15]}},extend[15:0]};		
	end
endmodule

/*
 * dataMem takes as input the address, the data to write, memRead, and
 * memWrite control bits. It outputs the outData with 350 ns delay.
 */
module dataMem(input clk, input[31:0] inAddr, input[31:0] writeData,
               input memRead, input memWrite, 
	       input[31:0] a0, input[31:0] v0, input isSyscall,
	       output[31:0] outData);
        reg[31:0] mem[0:1023];
        always @(posedge clk)
        begin
                if(memWrite) begin
                        #350 mem[inAddr] = writeData;
			//$display(mem[inAddr]);
		end
		if(isSyscall == 1) begin
			if(v0 == 1) begin
				$display(a0);
			end
			if (v0 == 4) begin
				$display(mem[a0]);
			end
			if (v0 == 10) begin
				$display("exit");
				$finish;
			end
		end

        end

        assign #350 outData = (memRead == 1)? (mem[inAddr]):0;
endmodule

/* 
 * regFile takes as input: clk, addresses of register 1 and register 2,
 * address of write register, regWrite control bit, and the data to write.
 * 
 * It outputs the data read from register 1 and register 2 with latency of 200ns.
 */
module regFile(input clk, input [4:0] readReg1Addr, input [4:0] readReg2Addr, 
		input [4:0] writeRegAddr, input regWrite, input [31:0] writeData, 
		output [31:0] reg1Data, output [31:0] reg2Data,
		output [31:0] a0, output [31:0] v0);
        reg [31:0] regFile[0:31];
        always@(posedge clk)
        begin
                if (regWrite)
                begin
                        #200 regFile[writeRegAddr] <= writeData;
                end
        end

        assign #200 reg1Data = regFile[readReg1Addr];
        assign #200 reg2Data = regFile[readReg2Addr];
	assign a0 = regFile[`a0];
	assign v0 = regFile[`v0];
endmodule

