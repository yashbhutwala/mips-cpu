module signExtend_tb;

reg[15:0] extend;
wire[31:0] extended;
signExtend mySignExtend(extend, extended);

initial begin
$monitor("Extend %16b, Extended %32b", extend, extended);
 extend = 16'b1011001100110011;
end
endmodule

