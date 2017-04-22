/* regFile takes as input: clk, addresses of register 1 and register 2,
 * address of write register, regWrite control bit, and the data to write.
 *
 * It outputs the data read from register 1 and register 2 with latency of 200
 * ns
 */
module regFile(input clk, input [4:0] readReg1Addr, input [4:0] readReg2Addr, input [4:0] writeRegAddr, input regWrite, input [31:0] writeData, output [31:0] reg1Data, output [31:0] reg2Data);

	reg [31:0] regFile[0:31];
	/*initial 
	begin
		$readmemh("reg.in", regFile);
	end */
	
	always@(posedge clk)
	begin
		if (regWrite) 
		begin 
			#200 regFile[writeRegAddr] <= writeData;
		end
	end

	assign #200 reg1Data = regFile[readReg1Addr];
	assign #200 reg2Data = regFile[readReg2Addr];
endmodule
