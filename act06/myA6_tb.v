/*
 * Name: Yash Bhutwala
 * CS 320 Activity 4
 */
module testMyA6;
reg clk;

/*
wire[31:0] currAddr;
wire[31:0] nextAddr;
wire[31:0] instruction;
*/

topModule myTop(clk);
//topModule myTop(clk, currAddr, nextAddr, instruction);

initial begin
	clk = 0;
end
initial 
begin
	//$monitor($time, " in %m, currPC = %08x, nextPC = %08x, instruction = %08x.", currAddr, nextAddr, instruction);
	//$monitor($time, " in %m, clk = %08x", clk);
	#20000 $finish;
end

always #500 clk = ~clk;

endmodule
