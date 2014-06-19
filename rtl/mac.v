//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.17.2014
//Description : Implement the memory_access_controller (MAC)
`define NOP_CMD       3'b000
`define READA_CMD     3'b001
`define WRITEA_CMD    3'b010
`define REFRESH_CMD   3'b011
`define PRECHARGE_CMD 3'b100
`define LOAD_MODE_CMD 3'b101
`define LOAD_REG1_CMD 3'b110
`define LOAD_REG2_CMD 3'b111


module memory_access_controller(
clk, 
resetn,
iMAC_ValidRd,
iMAC_AddrRd,
iMAC_TagRd,
iMAC_IdRd,
iMAC_LenRd,
iMAC_QoSRd,
oMAC_ReadyRd,
oMAC_ValidRsp,
oMAC_TagRsp,
oMAC_DataRsp,
oMAC_StatusRsp,
oMAC_EoD,
iMAC_ReadyRsp,
iMAC_ValidWr,
iMAC_AddrWr,
iMAC_TagWr,
iMAC_IdWr,
iMAC_LenWr,
iMAC_QoSWr,
oMAC_ReadyWr,
iMAC_DataWr,
iMAC_MaskWr,
iMAC_EoD,
ioDq,
oAddr,
oBank,
oCsn,
oRasn,
oCasn,
oWen,
oDqm
);
parameter REQ_IDLE       = 3'b000;
parameter REQ_FETCH_REQ  = 3'b001;
parameter REQ_FETCH_DATA = 3'b010;
parameter REQ_LAST_DATA  = 3'b100;
input    clk;
input    resetn;
input    iMAC_ValidRd;
input    iMAC_AddrRd;
input    iMAC_TagRd;
input    iMAC_IdRd;
input    iMAC_LenRd;
input    iMAC_QoSRd;
output   oMAC_ReadyRd;
output   oMAC_ValidRsp;
output   oMAC_TagRsp;
output   oMAC_DataRsp;
output   oMAC_StatusRsp;
output   oMAC_EoD;
input    iMAC_ReadyRsp;
input    iMAC_ValidWr;
input    iMAC_AddrWr;
input    iMAC_TagWr;
input    iMAC_IdWr;
input    iMAC_LenWr;
input    iMAC_QoSWr;
output   oMAC_ReadyWr;
input    iMAC_DataWr;
input    iMAC_MaskWr;
input    iMAC_EoD;
inout    ioDq;
output   oAddr;
output   oBank;
output   oCsn;
output   oRasn;
output   oCasn;
output   oWen;
output   oDqm;

wire         iMAC_ValidRd;
wire [31:0]  iMAC_AddrRd;
wire [3:0]   iMAC_TagRd;
wire [2:0]   iMAC_IdRd;
wire [1:0]   iMAC_LenRd;
wire [3:0]   iMAC_QoSRd;
wire         oMAC_ReadyRd;
reg          oMAC_ValidRsp;
reg  [3:0]   oMAC_TagRsp;
reg  [31:0]  oMAC_DataRsp;
reg  [1:0]   oMAC_StatusRsp;
reg          oMAC_EoD;
wire         iMAC_ReadyRsp;
wire         iMAC_ValidWr;
wire [31:0]  iMAC_AddrWr;
wire [3:0]   iMAC_TagWr;
wire [2:0]   iMAC_IdWr;
wire [1:0]   iMAC_LenWr;
wire [3:0]   iMAC_QoSWr;
wire         oMAC_ReadyWr;
wire [31:0]  iMAC_DataWr;
wire [3:0]   iMAC_MaskWr;
wire         iMAC_EoD;

reg  [31:0]  ioDq;
reg  [10:0]  oAddr;
reg  [1:0]   oBank;
reg          oCsn;
reg          oRasn;
reg          oCasn;
reg          oWen;
reg  [3:0]   oDqm;

assign      oMAC_ReadyWr = ~wFull0;
assign      oMAC_ReadyRd = ~wFull2;

always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rMACWrInfo     <= 45'b0;
        ppMAC_DataWr   <= 32'b0;
        pp2MAC_DataWr  <= 32'b0;
        ppMAC_MaskWr   <= 4'b0;
        pp2MAC_MaskWr  <= 4'b0;
    end else begin  
        //Save the info
        if (iMAC_ValidWr) begin 
            rMACWrInfo <= {iMAC_AddrWr, iMAC_TagWr, iMAC_IdWr, iMAC_LenWr, iMAC_QoSWr}; 
        end
        if (iMAC_ValidRd) begin 
            rMACRdInfo <= {iMAC_AddrRd, iMAC_TagRd, iMAC_IdRd, iMAC_LenRd, iMAC_QoSRd}; 
        end
    end
end
//Write Channel FSM 
always @(posedge clk or negedge resetn ) begin 
    if (~resetn) begin 
        sMacWr  <= REQ_IDLE;   
    end else begin 
        sMacWr  <= nsMacWr;
    end 
end 

always @(*) begin 
    nsMacWr = sMacWr;
    case(sMacWr) 
        REQ_IDLE      : begin 
                              if (iMAC_ValidWr) begin 
                                  nsMacWr = EQ_FETCH_REQ;
                              end else 
                                  nsMacWr = REQ_IDLE;
                          end
        REQ_FETCH_REQ : begin 
                              if (iMAC_EoD) 
                                  nsMacWr = REQ_LAST_DATA;                    
                          end 
        REQ_LAST_DATA : begin 
                              nsMacWr = REQ_IDLE;
                          end
    endcase
end 

always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rWr0     <= 1'b0;
        rWrData0 <= 45'b0;
        rWr1     <= 1'b0;
        rWrData1 <= 36'b0;
    end else begin 
        case (sMacWr)
            REQ_IDLE      : begin 
                                  rWr0     <= 1'b0;
                                  rWrData0 <= 45'b0;
                                  rWr1     <= 1'b0;
                                  rWrData1 <= 36'b0;
                              end 
            REQ_FETCH_REQ : begin  
                                  rWr0     <= 1'b1;
                                  rWrData0 <= rMACWrInfo ;
                              end 
            REQ_FETCH_DATA: begin 
                                  rWr0     <= 1'b0; //FIXME: Performance is very low for data 
                                  rWrData0 <= 45'b0;
                                  rWr1     <= 1'b1;
                                  rWrData1 <= {pp2MAC_DataWr, pp2MAC_MaskWr} ; 
                                  rWrAddr1 <= ppMacWrAddr[6:2];
                              end 
            REQ_LAST_DATA: begin 
                                  rWr1     <= 1'b1;
                                  rWrData1 <= {pp2MAC_DataWr, pp2MAC_MaskWr} ; 
                              end 
        endcase
    end 
end 


//Read channel FSM
always @(posedge clk or negedge resetn ) begin 
    if (~resetn) begin 
        sMacRd  <= REQ_IDLE;   
    end else begin 
        sMacRd  <= nsMacRd;
    end 
end 

always @(*) begin 
    nsMacRd = sMacRd;
    case(sMacRd) 
        REQ_IDLE      : begin 
                              if (iMAC_ValidRd) begin 
                                  nsMacRd = REQ_FETCH_REQ;
                              end else 
                                  nsMacRd = REQ_IDLE;
                          end
        REQ_FETCH_REQ : begin 
                              if (iMac_ValidRd)
                                  nsMacWr = REQ_FETCH_REQ;                    
                              end else 
                                  nsMacRd = REQ_IDLE;
                          end 
    endcase
end 

always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rWr2     <= 1'b0;
        rWrData2 <= 45'b0;
    end else begin 
        case (sMacRd)
            REQ_IDLE      : begin 
                                  rWr2     <= 1'b0;
                                  rWrData2 <= 45'b0;
                            end 
            REQ_FETCH_REQ : begin  
                                  rWr2     <= 1'b1;
                                  rWrData2 <= rMACRdInfo ;
                            end 
        endcase
    end 
end 



fifo #(.DSIZE(45), .ASIZE(5) ) wrReqQueue (
  .wclk(clk), 
  .wrst_n(resetn),
  .wr(rWr0),
  .rclk(clk),
  .rrst_n(resetn),
  .rd(rRd0),
  .wdata(rWrData0),
  .rdata(rRdData0),
  .wfull(wFull0),
  .rempty(wEmpty0)
);

sram_2p #( .AW(5), .DW(32)) wrDataQueue (
  .clkA(clk),
  .iWrA(rWr1),
  .iAddrA(rWrAddr1),
  .iDataA(rWrData1),
  .clkB(clk),
  .iRdB(rRd1),
  .iAddrB(rRdAddr1),
  .oDataB(wRdData1)
);

fifo #(.DSIZE(45), .ASIZE(5) ) rdReqQueue (
  .wclk(clk), 
  .wrst_n(resetn),
  .wr(rWr2),
  .rclk(clk),
  .rrst_n(resetn),
  .rd(rRd2),
  .wdata(rWrData2),
  .rdata(rRdData2),
  .wfull(wFull2),
  .rempty(wEmpty2)
);



endmodule 
