
/*
 * Name: Yash Bhutwala
 * CS 320 Activity 4
 */
module mips_tb;
reg clk;

topModule myTop(clk);
integer cyclecount;
initial begin
	clk = 0;
	cyclecount = 0;
end
initial 
begin
	//$monitor($time, " in %m, currPC = %08x, nextPC = %08x, instruction = %08x.", currAddr, nextAddr, instruction);
	//$monitor($time, " in %m, clk = %08x", clk);
	$dumpfile("testData.vcd");
	$dumpvars;
	#540;
	$display("exit");
	$finish;
end

always 
	#10 clk = ~clk;
always@(posedge clk)
	begin

		cyclecount = cyclecount + 1;
		$display("%d ", cyclecount);

	end

endmodule
