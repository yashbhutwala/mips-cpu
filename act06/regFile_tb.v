`timescale 1ns/1ns
module regFile_tb;

reg clk = 0;
reg [4:0] readReg1Addr;
reg [4:0] readReg2Addr;
reg [4:0] writeRegAddr;
reg regWrite;
reg [31:0] writeData;
wire [31:0] reg1Data;
wire [31:0] reg2Data; 
  
always #1 clk = !clk;

regFile rf(clk, readReg1Addr, readReg2Addr, writeRegAddr, regWrite, writeData, reg1Data, reg2Data);

initial
begin
	regWrite = 1;
	writeRegAddr = 1;
	writeData = 13;
	#210 writeRegAddr = 5; writeData = 47;
	#210 writeRegAddr = 2;
	writeData = 47;

	#210 writeRegAddr = 3;
	writeData = 4;

	#210 writeRegAddr = 4;
	writeData = 56;

	#210 writeRegAddr = 5;
	writeData = 42;

	#210 writeRegAddr = 6;
	writeData = 7;

	#210 writeRegAddr = 7;
	writeData = 84;
	regWrite = 0;
	#210 writeRegAddr = 5;
	writeData = 74;
	#210 readReg1Addr = 1;
	readReg2Addr = 2;
	#210 readReg1Addr = 1;

	readReg2Addr = 2;
	#210 readReg1Addr = 2;

	readReg2Addr = 3;
	#210 readReg1Addr = 3;

	readReg2Addr = 4;
	#210 readReg1Addr = 4;

	readReg2Addr = 5;
	#210 readReg1Addr = 5;

	readReg2Addr = 6;
	#210 readReg1Addr = 6;

	readReg2Addr = 7;	
end

initial
begin
	$monitor("reg1Data = %0d, reg2Data = %0d", reg1Data, reg2Data);
end
endmodule
