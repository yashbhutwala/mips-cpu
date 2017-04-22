module fetch(	input clk, 
		input stallF, 
		input pcSrcD, 
		input [31:0] pcBranchD, 
		
		output[31:0] instrF, 
		output[31:0] pcPlus4F

		);
   wire [31:0] pc_prime;
   reg [31:0] pcf;
   wire [2:0]  empty;
   reg [31:0]  value_four = 4;
   

   mux2To1 pcMux(
		 //Inputs
		 .in1(pcBranchD), 
		 .in2(pcPlus4F),
		 //selector
		 .sel(pcSrcD),
		 //Outputs
		 .out(pc_prime)
		 );
   
   always@ (posedge clk) begin
      if (stallF == 0)
	pcf = pc_prime;
      end

   alu myAlu(
	    //inputs
	    .aluOp(empty),
	    .in0(pcf),
	    .in1(value_four),
	    .aluResult(pcPlus4F)
	    );

   instMem myInstMem(pcf[31:2],instrF);
   
   
	
	
   

endmodule
