module mux3To1(input[31:0] in1, input[31:0] in2, input[31:0] in3, input[1:0] sel, output[31:0] out);
	reg[31:0] tempOut;
	assign out = tempOut;
	always @(sel or in1 or in2 or in3) begin
		if (sel == 2'b00)  tempOut = in1;
		else if (sel == 2'b01) tempOut = in2;
		else if (sel == 2'b10) tempOut = in3;
		else tempOut = 0;
	end
endmodule

