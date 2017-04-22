module hazard(input clk, input rsd, input rtd, input rdd, input rse, input rte, input rde, input [4:0] writeRegE, input [4:0] writeRegM,input [4:0] writeRegW, input BranchD, input memToRegE, input RegWriteE, input memToRegM, input regWriteM, input regWriteW, output reg ForwardAD, output reg ForwardBD, output reg FlushE, output reg ForwardAE, output reg ForwardBE, output reg stallF, output reg stallD);
   reg lwstall;
   reg branchstall;
   
   
// Forwarding logic for SrcA
   always @(*)begin
      if ((rse !=0) && (rse == writeRegM) && regWriteM)
	ForwardAE = 2'b10;
     else if ((rse!=0) && (rse == writeRegW) && regWriteW)
       ForwardAE = 2'b01;
     else
       ForwardAE = 2'b00;


// Forwarding logic for SrcB
 
      if ((rte !=0) && (rte == writeRegM) && regWriteM)
	ForwardBE = 2'b10;
     else if ((rte!=0) && (rte == writeRegW) && regWriteW)
       ForwardBE = 2'b01;
     else
       ForwardBE = 2'b00;
   end
   
  always @(*)begin
     lwstall = ((rsd == rte) || (rtd == rte)) && memToRegE;
     FlushE = lwstall;
     stallD = FlushE;
     stallF = stallD;
     end

//Decode stage forwarding logic

   always @(*)begin
   ForwardAD = ((rsd != 0) && (rsd == writeRegM) && regWriteM);
   ForwardBD = ((rtd != 0) && (rtd == writeRegM) && regWriteM);
      end
 
//Stall detection logic
   always @(*) begin
   branchstall = (BranchD && RegWriteE && (writeRegE == rsd || writeRegE == rtd)) || (BranchD && memToRegM && (writeRegM == rsd || writeRegM == rtd));
   FlushE = lwstall || branchstall;
   stallD = FlushE;
   stallF = stallD;
      

   end


   endmodule

   
    
