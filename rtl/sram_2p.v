`timescale 1 ns/1 ps

module sram_2p (
  clkA,
  iWrA,
  iAddrA,
  oDataA,
  iDataA,
  clkB,
  resetnB,
  iRdB,
  iAddrB,
  oDataB,
  iDataB
);

  parameter                AW = 5;
  parameter                DW = 32;
  parameter                DEPTH = 1<<AW;

  output [DW-1:0]  oDataA;
  input            clkA;
  input [AW-1:0]   iAddrA;
  input [DW-1:0]   iDataA;
  input            iWrA;
  output [DW-1:0]  oDataB;
  input            clkB;
  input            resetnB;
  input [AW-1:0]   iAddrB;
  input [DW-1:0]   iDataB;
  input            iRdB;

  // Models no-change memory behavior, Q changes only when a valid read enable occurs
  reg [DW-1:0] mem [DEPTH-1:0] /* synthesis syn_ramstyle = block_ram */;
  reg [DW-1:0]           oDataA;
  reg [DW-1:0]           oDataB;

integer k;
initial
begin
for (k = 0; k < DEPTH - 1; k = k + 1)
begin
    mem[k] = 0;
end
end

  // read with WEN
  // write with WEN and CEN
  always @(posedge clkA) begin
    if (iWrA ) mem[iAddrA] <= iDataA;
  end

  // read with WEN
  always @(posedge clkB or negedge resetnB) begin
    if (~resetnB)
        oDataB <= {DW{1'b0}};
    else 
        if (iRdB ) oDataB <= mem[iAddrB];
  end
endmodule
