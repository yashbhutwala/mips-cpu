
/*
 * Name: Yash Bhutwala
 * CS 320 Activity 4
 */
module mips_tb;
reg clk;

topModule myTop(clk);

initial begin
	clk = 0;
end
initial 
begin
	//$monitor($time, " in %m, currPC = %08x, nextPC = %08x, instruction = %08x.", currAddr, nextAddr, instruction);
	//$monitor($time, " in %m, clk = %08x", clk);
	$dumpfile("testData.vcd");
	$dumpvars;
	#800;
	$finish;
end

always #10 clk = ~clk;

endmodule
