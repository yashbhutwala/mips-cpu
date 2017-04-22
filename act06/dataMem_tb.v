module dataMem_tb;

reg clk = 0;
reg[31:0] aluOutAddr;
reg[31:0] writeData;
reg memRead = 0;
reg memWrite = 0;
wire[31:0] outData;

always #10 clk = ~clk;

dataMem dm(clk, aluOutAddr, writeData, memRead, memWrite, outData);

initial
begin
	#200 memWrite = 1; aluOutAddr = 4; writeData = 69;
	#500 memWrite = 1; aluOutAddr = 6; writeData = 42;
	memWrite = 0;
	#500 memRead = 1; aluOutAddr = 4;
end

initial
begin
	$monitor($time, ": memWrite = %b, memRead = %b, aluOutAddr = %x, writeData = %x, outData = %x.", memWrite, memRead, aluOutAddr, writeData, outData);
	#10000 $finish;
end

endmodule
