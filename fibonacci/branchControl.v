

module jumpControl (
	input jump,
	input [5:0] opcode,
	input [4:0] rtD,
	input [5:0] funct,
	input [31:0] signImmD,
	input [31:0] jumpAddr,
	input [31:0] pcPlus4D, 
	input [31:0] pcPlus4F,
	input [31:0] rd1,
	input [31:0] rd2,
	output [31:0] pcPrime,
	output pcSrcD,
	output jal
	);

	wire equalD, branchD;
	wire [31:0] pcBranchD, shiftedSignImmD;

	// J, JR, JAL, BEQ, BNE, B, BLEZ, BGEZ
	
	always @(*)
		case (opcode)
			`BEQ: begin
			if (equalD)
				pcPrime <= pcBranchD;
			else
				pcPrime <= pcPlus4F;
			branchD <= 1'b1;
			

endmodule
