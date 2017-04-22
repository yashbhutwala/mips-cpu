module hazard(input [4:0] rsD, input [4:0] rtD, input [4:0] rsE, input [4:0] rtE, input [4:0] writeregE, input [4:0] writeregM, input [4:0] writeregW, input regwriteE, input regwriteM, input regwriteW, input memtoregE, input memtoregM, input branchD, output reg forwardaD, output reg forwardbD, output reg [1:0] forwardaE, output reg [1:0] forwardbE, output reg [1:0] stallF, output reg [1:0] stallD, output reg [1:0] flushE);

   reg lwstallD; reg branchstallD;
   always@(*) begin
      //forwarding sources to D stage (branch equality)
      forwardaD = (rsD != 0) & (rsD == writeregM) & regwriteM;
      forwardbD = (rtD != 0) & (rtD == writeregM) & regwriteM;
   end
   always@(*)
     //forwarding sources to E stage (ALU)
     begin
	forwardaE = 2'b00; forwardbE = 2'b00;
	if(rsE != 0)
          if (rsE == writeregM & regwriteM)
	    forwardaE = 2'b10;
	  else if (rsE == writeregW & regwriteW)
	    forwardaE = 2'b01;
	if (rtE != 0)
	  if(rtE == writeregM & regwriteM)
	    forwardbE = 2'b10;
	  else if (rtE == writeregW & regwriteW)
	    forwardbE = 2'b01;
     end

   always@(*) begin
      //stalls
      lwstallD = memtoregE & (rtE == rsD | rtE == rtD);
      branchstallD = branchD & (regwriteE & (writeregE == rsD | writeregE == rtD) | memtoregM & (writeregM == rsD | writeregM == rtD));
      stallD = lwstallD | branchstallD;
      stallF = stallD;
      flushE = stallD;
      end
   endmodule
