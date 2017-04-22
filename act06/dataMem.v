/*
 * dataMem takes as input the address, the data to write, memRead, and
 * memWrite control bits. It outputs the outData with 350 ns delay.
 */
module dataMem(input clk, input[31:0] inAddr, input[31:0] writeData,
	       input memRead, input memWrite, output[31:0] outData);
        reg[31:0] mem[0:1023];
	always @(posedge clk) 
	begin
		if(memWrite) 
		begin
			#350 mem[inAddr] = writeData;
			//$display(mem[inAddr]);
		end
	end

        assign #350 outData = (memRead == 1)? (mem[inAddr]):0;
endmodule
