
module topModule(input clk); 
   wire stallF;
   wire pcSrcD;
   wire [31:0] pcBranchD;
   wire [31:0] instrF;
   wire [31:0] pcPlus4F;

   wire        regWriteD; wire regWriteE;wire regWriteM; wire regWriteW;
   wire        memToRegD; wire memToRegE;wire memToRegM;
   wire        memWriteD; wire memWriteE;
  
   wire [2:0]  aluControlD;
   wire        aluSrcD;
   wire [31:0] aluOutE; wire [31:0] aluOutM;
   
       
   wire        regDstD;
   wire        pcSrcD;
   wire [31:0] pcBranchD;
   wire [31:0] rd1MuxOutD; wire[31:0] rd2MuxOutD;
   wire [4:0]  rsD; wire [4:0] rtD; wire [4:0] rdD;
   wire [31:0] signImmD;
   
   wire [31:0] writeDataE;

   wire [4:0]  rsE;
   wire [4:0]  rtE;
   wire [4:0]  writeRegE; wire [4:0] writeRegM;wire [4:0] writeRegW;
   wire [31:0] resultW;
   

   wire [31:0] a0D; wire [31:0] v0D; wire isSyscallD;
   wire [31:0] a0E; wire [31:0] v0E; wire isSyscallE;
   
   Hazard myHazard(.rdD(rsD),.rtD(rtD),.rsE(rsE),.rtE(rtE),
		 .writeregE(writeRegE),.writeregM(writeRegM),.writeregW(writeRegW),
		 .regwriteE(regWriteE),.regwriteM(regWriteM),.regwriteW(regWriteW),
		 .memtoregE(memToRegE),.memtoregM(memToRegM),.branchD(branchD),

		 .forwardaD(forwardAD),.forwardbD(forwardBD),
		 .forwardaE(forwardAE),.forwardbE(forwardBE),
		 .stallF(stallF),.stallD(stallD),.flushE(flushE));
   
   fetch F(.clk(clk),
	   .stallF(stallF),
	   .pcSrcD(pcSrcD),
	   .pcBranchD(pcBranchD),

	   .instrF(instrF),
	   .pcPlus4F(pcPlus4F));
   
   decode D(.clk(clk),
	    .stallD(stallD),
	    .instrF(instrF),
	    .pcPlus4F(pcPlus4F),
	    .forwardaD(forwardaD),.forward(forwardbD),
	    .writeRegW(writeRegW),
	    .resultW(resultW),
	    .regWriteW(regWriteW),
	    .aluOutM(aluOutM),

	    .regWriteD(regWriteD),.memToRegD(memToRegD),.memWriteD(memWriteD),
	    .aluControlD(aluControlD),.aluSrcD(aluSrcD),.regDstD(regDstD),
	    .pcSrcD(pcSrcD),
	    .pcBranchD(pcBranchD),
	    .rd1MuxOutD(rd1MuxOutD),.rd2MuxOutD(rd2MuxOutD),
	    .rsD(rsD),.rtD(rtD),.rdE(rdE),
	    .signImmD(signImmD),
	    .a0D(.a0D), .v0D(.v0D), .isSyscallD(isSyscallD)
	);

   execute E(.clk(clk),
	     .flushE(flushE),
	     .regWriteD(regWriteD),.memToRegD(memToRegD),.memWriteD(memWriteD),
	     .aluControlD(aluControlD),.aluSrcD(aluSrcD),.regDstD(regDstD),
	     .rd1MuxOutD(rd1MuxOutD),.rd2MuxOutD(rd2MuxOutD),
	     .rsD(rsD),.rtD(rtD),.rdD(rdD),
	     .signImmD(signImmD),
	     .forwardAE(forwardAE),.forwardBE(forwardBE),
	     .resultW(resultW),.aluOutM(aluOutM),
	     .a0D(.a0D), .v0D(.v0D), .isSyscallD(isSyscallD),

	     .regWriteE(regWriteE),.memToRegE(memToRegE),.memWriteE(memWriteE),
	     .aluOutE(aluOutE),.writeDataE(writeDataE),
	     .rsE(rsE),.rtE(rtE),.writeRegE(writeRegE),
	     .a0E(.a0E), .v0E(.v0E), .isSyscallE(isSyscallE)
	);

   memory M(.clk(clk),
	    .regWriteE(regWriteE),.memToRegE(memToRegE),.memWriteE(memWriteE),
	    .aluOutE(aluOutE),.writeDataE(writeDataE),
	    .writeRegE(writeRegE),
	    .a0E(.a0E), .v0E(.v0E), .isSyscallE(isSyscallE),

	    .regWriteM(regWriteM),.memToRegM(memToRegM),
	    .readDataM(readDataM),.aluOutM(aluOutM),
	    .writeRegM(writeRegM));

   writeback W(.clk(clk),
	       .regWriteM(regWriteM),.memToRegM(memToRegM),
	       .readDataM(readDataM),.aluOutM(aluOutM),
	       .writeRegM(writeRegM),

	       .writeRegW(writeRegW),.resultW(resultW),
	       .regWriteW(regWriteW));
         
endmodule


module fetch(	
		input clk, 
		input stallF, 
		input pcSrcD, 
		input [31:0] pcBranchD, 
		
		output[31:0] instrF, 
		output[31:0] pcPlus4F

		);
	
	wire [31:0] pcPrime; //Either pcPlus4F or pcBranchD
	wire  [31:0] pcf;  // comes from pipeline register. Changed if no stall
   
	//Mux to select the value of the PC
	mux2To1 pcMux (
		 //Inputs
		 .in1(pcBranchD), 
		 .in2(pcPlus4),
		 .sel(pcSrcD),
		 //Outputs
		 .out(pcPrime)
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
	    	.in(pcf),
		//Output
	    	.out(pcPlus4F)
	    	);
   
	//inputs the high 30 bits of the PC register and outputs the instructions read from memory
	instMem myInstMem (
		//Input
		.readAddr(pcf[31:2]), 
		//Output		
		.memOut(instrF)
		);

endmodule

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
		output [31:0] pcBranchD, 
		output [31:0] rd1MuxOutD, output[31:0] rd2MuxOutD, 
		output [4:0] rsD, 
		output [4:0] rtD, 
		output [4:0] rdD, 
		output [31:0] signImmD,
		output [31:0] a0D, output [31:0] v0D, output isSyscallD
		
		);

	wire [31:0] instrD;
	wire [31:0] pcPlus4D;
	wire [31:0] rd1;
	wire [31:0] rd2;
	assign rsD = instrD[`rs];
	assign rtD = instrD[`rt];
	assign rdD = instrD[`rd];

	wire equalD; wire branchD;
	assign equalD = (rd1MuxOutD == rd2MuxOutD)? 1:0;
	assign pcSrcD = branchD & equalD;

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
		.writeData(resultW),
		// Outputs  
		.reg1Data(rd1), //
		.reg2Data(rd2) //
		.a0(a0D), 
		.v0(v0D)
		);

	mux2To1 rd1Mux (
		// Inputs
		.in1(aluOutM), //
		.in2(rd1), 
		.sel(forwardAD), 
		// Outputs
		.out(rd1MuxOutD)
		);

	mux2To1 rd2Mux (
		// Inputs
		.in1(aluOutM), // 
		.in2(rd2), 
		.sel(forwardBD), 
		// Outputs
		.out(rd2MuxOutD)
		);
	
	control myControl (
		.opcode(instrD[`op]), 
		.funct(instrD[`function]), 
		//TODO: .jump(), 
		.regDst(regDstD),
                .branch(branchD), 
		//TODO: .memRead(), 
		.memToReg(memToRegD),
                .aluOp(aluControlD), 
		.regWrite(regWriteD),
                .aluSrc(aluSrcD), 
		.memWrite(memWriteD),
		.syscall(isSyscallD),
		//TODO: .jumpReg(),
		//TODO: .jumpAndLink(),
		//TODO: .lui()
		);

	signExtend mySignExtend (
		.extend(instrD[`immediate]),
		.extendedOut(signImmD)
		);

	addBranch myAddBranch (
		.inAddr1({signImmD[29:0], 2'b00}),
		.inAddr2(pcPlus4D),
		.outAddr(pcBranchD)
		);
endmodule

module execute(	
		input clk, 
		input flushE, 
		input regWriteD, input memToRegD, input memWriteD, 
		input [2:0] aluControlD, input aluSrcD, input regDstD, 
		input [31:0] rd1MuxOutD, 
		input [31:0] rd2MuxOutD, 
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
		output [31:0] a0E, output[31:0] v0E, input isSyscallE		

		);

	wire aluSrcE;
	wire regDstE;
	wire rd1MuxOutE;
	wire rd2MuxOutE;
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
		.MemtoRegD(memToRegD,
		.MemWriteD(memWriteD,
		.ALUControlD(aluControlD),
		.ALUSrcD(aluSrcD),
		.RegDstD(regDstD),
		.reg1DataD(rd1MuxOutD),
		.reg2DataD(rd2MuxOutD),
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
		.reg1DataE(rd1MuxOutE), //
		.reg2DataE(rd2MuxOutE), //
		.rsE(rsE),
		.rtE(rtE),
		.rdE(rdE), //
		.signImmE(signImmE) //
		.a0E(a0E),
		.v0E(v0E),
		.isSyscallE(isSyscallE)
		); 
	
	mux3To1 forwardAEMux (
		// Inputs
		.in1(rd1MuxOutE),
		.in2(resultW),
		.in3(aluOutM),
		.sel(forwardAE),
		// Outputs
		.out(srcAE) //
		);

	mux3To1 forwardBEMux (
		// Inputs
		.in1(rd2MuxOutE),
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

module memory(	
		input clk
		input regWriteE, input memToRegE, input memWriteE,
		input [31:0] aluOutE, 
		input [31:0] writeDataE, 
		input [4:0] writeRegE,
		input [31:0] a0E, input[31:0] v0E, input isSyscallE,

		output regWriteM, output memToRegM, 
		output [31:0] readDataM,		
		output [31:0] aluOutM,
		output [4:0] writeRegM
		
		);

	wire memWriteM;
	wire writeDataM;
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
		.a0(a0M), 
		.v0(v0M), 
		.syscall(isSyscallM), 
		// Outputs
		.outData(readDataM)
		);
endmodule

module writeback(  
		input clk,
		input regWriteM, input memToRegM, 
		input [31:0] readDataM,		
		input [31:0] aluOutM,
		input [4:0] writeRegM, 

		output [4:0] writeRegW,
		output [31:0] resultW,
		output regWriteW

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
		// Outputs 
		.RegWriteW(regWriteW), 
		.MemtoRegW(memToRegW), 
		.ReadDataW(readDataW), 
		.ALUOutW(aluOutW), 
		.WriteRegW(.writeRegW)
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

/*
module topModule(input clk);
   wire[31:0] currAddr; wire[31:0] nextAddr; wire[31:0] instruction;
   wire[31:0] pcPlus4;
   wire jump; wire regDst; wire branch; wire memRead;
   wire memToReg; wire memToRegM; wire memToRegW;
   wire [2:0] aluOp; wire[2:0] aluControlE;
   wire regWrite; wire regWriteE; wire regWriteM; wire regWriteW;
   wire aluSrc;
   wire memWrite; wire memWriteM;
   wire syscall; wire[31:0] a0; wire[31:0] v0;
   wire jumpReg;
   wire jumpAndLink;
   wire lui;
   wire [31:0] writeRegAddr; wire [4:0] writeRegE; wire [4:0] writeRegM; wire [4:0] writeRegW;
   wire [31:0] reg1Data;
   wire [31:0] reg2Data; wire [31:0] reg2DataM;
   wire [31:0] writeData; wire[31:0] writeDataE; wire [31:0] writeDataM;
   wire [31:0] signExtendOut; wire [31:0] signImmE;
   wire [31:0] signExtendMuxOut;
   wire [31:0] aluOut; wire[31:0] aluOutM; wire[31:0] aluOutW;
   wire [31:0] dataMemOut; wire [31:0] readDataW;
   wire [31:0] addBranchResult;
   wire [31:0] forJumpMux;
   wire zero;
   wire [4:0] rsE; wire [4:0] rtE; wire [4:0] rdE;

   PRE myPRE(clk,flushE,regWrite,memToReg,memWrite,aluOP,aluSrc,regDst,reg1Data,reg2Data,instructionD[`rs],instructionD[`rt],instructionD[`rd],signExtendOut, regWriteE,memToRegE,memWriteE,aluControlE,aluSrcE, regDstE, reg1DataE,reg2DataE,rsE,rtE,rdE,signImmE);

   PRM myPRM(clk,regWrite,memtoReg,memWrite,aluOut,reg2Data,writeRegAddr[4:0],regWriteM, memToRegM, memWriteM,aluOutM,reg2DataM,writeRegM);
   PRW myPRW(clk,regWriteM,memToRegM,dataMemOut,aluOutM,writeRegM,regWriteW,memToRegW,readDataW,aluOutW,writeRegW);

   pc myPc(clk, nextAddr, currAddr);
   add4 myAdd4(currAddr, pcPlus4);
   instMem myMem(currAddr[31:2], instruction);

   control myControl(instruction[`op], instruction[`function], jump, regDst, branch, memRead, memToReg, aluOp, regWrite, aluSrc, memWrite, syscall, jumpReg, jumpAndLink, lui);
   mux2To1 regMux({27'b0, instruction[`rd]}, {27'b0, instruction[`rt]}, regDst, writeRegAddr);
   regFile myRegFile(clk, instruction[`rs], instruction[`rt], writeRegW, regWriteW, writeData, reg1Data, reg2Data, a0, v0);

   signExtend mySe(instruction[`immediate], signExtendOut);
   addBranch myAddBranch(pcPlus4, {signExtendOut[29:0], 2'b00}, addBranchResult);
   mux2To1 branchMux(addBranchResult, pcPlus4, (zero && branch), forJumpMux);
   mux2To1 jumpMux({pcPlus4[31:28], instruction[`target], 2'b00}, forJumpMux, jump, nextAddr);

   mux2To1 aluMux(signExtendOut, reg2Data, aluSrc, signExtendMuxOut);
   alu myAlu(aluOp, reg1Data, signExtendMuxOut, aluOut, zero);

   dataMem myDataMem(clk, aluOutM, reg2DataM, 1'b1, memWriteM, a0, v0, syscall, dataMemOut);
   mux2To1 dataMemMux(readDataW, aluOutW, memToRegW, writeData);
   wire [1:0] 	   forwardAE;
   wire [1:0] 	   forwardBE;

   hazard myHazard (instructionD[`rs], instructionD[`rt], rsE, rtE, writeRegE, writeRegM, writeRegW,                     regWriteE, regWriteM, regWriteW, memToRegE, memToRegM, branchD,
                    forwardAD, forwardBD, forwardAE, forwardBE, stallF, stallD, flushE);

   mux3To1 forwardAEMux3To1(reg1DataE, resultW, aluOutM, forwardAE, srcAE);
   mux3To1 forwardBEMux3To1(reg2DataE, resultW, aluOutM, forwardBE, srcBEMux);
   mux2To1 srcBEMux2To1(signImmE, srcBEMux, aluSrcE, srcBE);
//insert a name for the mux below
   mux2To1 insertNameHere(rdE, rtE, regDstE, writeRegE);
endmodule
*/

