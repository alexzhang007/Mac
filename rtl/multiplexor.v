//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : May. 11. 2014
//Description : Implement the multiplexor with 4-1 and 2-1
//              Fix Bug18: Mux output has X
module mux_2 (
iZeroBranch,
iOneBranch,
iSel,
oMux
);
input iZeroBranch;
input iOneBranch;
input iSel;
output oMux;
parameter DATA_WIDTH = 32;
wire [DATA_WIDTH-1:0] iZeroBranch;
wire [DATA_WIDTH-1:0] iOneBranch;
wire iSel;
reg  [DATA_WIDTH-1:0] oMux;

always @(*) begin 
    case (iSel) 
        1'b0 : oMux = iZeroBranch;
        1'b1 : oMux = iOneBranch;
    endcase 
end 

endmodule 

module mux_3 (
iZeroBranch,
iOneBranch,
iTwoBranch,
iSel,
oMux
);
input iZeroBranch;
input iOneBranch;
input iTwoBranch;
input iSel;
output oMux;
parameter DATA_WIDTH = 32;
wire [DATA_WIDTH-1:0] iZeroBranch;
wire [DATA_WIDTH-1:0] iOneBranch;
wire [1:0]            iSel;
reg  [DATA_WIDTH-1:0] oMux;

always @(*) begin 
    case (iSel) 
        2'b00 : oMux = iZeroBranch;
        2'b01 : oMux = iOneBranch;
        2'b11 : oMux = iOneBranch;
        default : oMux = iZeroBranch;
    endcase 
end 

endmodule 

module mux_4 (
iZeroBranch,
iOneBranch,
iTwoBranch,
iThreeBranch,
iSel,
oMux
);
input iZeroBranch;
input iOneBranch;
input iTwoBranch;
input iThreeBranch;
input iSel;
output oMux;

parameter DATA_WIDTH = 32;
wire [DATA_WIDTH-1:0] iZeroBranch;
wire [DATA_WIDTH-1:0] iOneBranch;
wire [DATA_WIDTH-1:0] iTwoBranch;
wire [DATA_WIDTH-1:0] iThreeBranch;
wire [1:0] iSel;
reg [DATA_WIDTH-1:0] oMux;

//If always sensity list only has iSel, it will cause the oMux X
always @(*) begin 
    case (iSel) 
      2'b00 : oMux = iZeroBranch;
      2'b01 : oMux = iOneBranch;
      2'b10 : oMux = iTwoBranch;
      2'b11 : oMux = iThreeBranch;
    endcase
end 

endmodule 
