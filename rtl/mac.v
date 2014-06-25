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
`define REQWR_INFO_W  45
`define REQWR_DATA_W  36
`define ROB_ITEM_W 24
`define ROW_W      11
`define BANK_W     2
`define COL_W      8
//Interleave with Bank
`define ADDR_ROW_RANGE  42:32
`define ADDR_BANK_RANGE 22:21
`define ADDR_COL_RANGE  20:13
`define REQ_SIZE_RANGE  5:4

module mem_access_controller(
clk, 
resetn,
sclk,
sresetn,
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
input    sclk;
input    sresetn;
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

wire                       iMAC_ValidRd;
wire [31:0]                iMAC_AddrRd;
wire [3:0]                 iMAC_TagRd;
wire [2:0]                 iMAC_IdRd;
wire [1:0]                 iMAC_LenRd;
wire [3:0]                 iMAC_QoSRd;
wire                       oMAC_ReadyRd;
reg                        oMAC_ValidRsp;
reg  [3:0]                 oMAC_TagRsp;
reg  [31:0]                oMAC_DataRsp;
reg  [1:0]                 oMAC_StatusRsp;
reg                        oMAC_EoD;
wire                       iMAC_ReadyRsp;
wire                       iMAC_ValidWr;
wire [31:0]                iMAC_AddrWr;
wire [3:0]                 iMAC_TagWr;
wire [2:0]                 iMAC_IdWr;
wire [1:0]                 iMAC_LenWr;
wire [3:0]                 iMAC_QoSWr;
wire                       oMAC_ReadyWr;
wire [31:0]                iMAC_DataWr;
wire [3:0]                 iMAC_MaskWr;
wire                       iMAC_EoD;
wire [31:0]                ioDq;
reg  [10:0]                oAddr;
reg  [1:0]                 oBank;
reg                        oCsn;
reg                        oRasn;
reg                        oCasn;
reg                        oWen;
reg  [3:0]                 oDqm;
wire                       wFull0;
wire                       wEmpty0;
wire                       wFull2;
wire                       wEmpty2;
reg  [2:0]                 sMacWr;
reg  [2:0]                 nsMacWr;
reg  [2:0]                 sMacRd;
reg  [2:0]                 nsMacRd;
reg  [`REQWR_INFO_W-1:0 ]  rMACWrInfo;
reg  [`REQWR_INFO_W-1:0 ]  rMACRdInfo;
reg  [`REQWR_INFO_W-1:0 ]  rWrData0;
reg  [`REQWR_INFO_W-1:0 ]  rWrData2;
reg  [`REQWR_DATA_W-1:0 ]  ppMAC_DataWr;
reg  [`REQWR_DATA_W-1:0 ]  pp2MAC_DataWr;
reg  [`REQWR_DATA_W-1:0 ]  rWrData1;
reg  [3:0]                 ppMAC_MaskWr;
reg  [3:0]                 pp2MAC_MaskWr;
reg                        rWr2;
reg                        rWr1;
reg                        rWr0;
reg  [31:0]                ppMACAddrWr;
reg  [4:0]                 rWrAddr1;
reg  [1:0]                 rRoundRobin;
reg                        rRd0;
reg                        rRd2;
reg                        rLoS;
reg                        rSelBank0;
reg                        rSelBank1;
reg                        rSelBank2;
reg                        rSelBank3;
reg                        ppLoS;
reg  [`REQWR_INFO_W-1:0]   ppReqQItem; //45
wire [`REQWR_INFO_W-1:0]   wReqQItem; //45
wire [`REQWR_INFO_W-1:0]   wRdData0; //45
wire [`REQWR_INFO_W-1:0]   wRdData2; //45
wire                       wRdValid0;
wire                       wRdValid2;
wire [1:0]                 wSelA;
wire [2:0]                 wROB_Way0;
wire                       wROB_WayWr0;
wire [2:0]                 wROB_Way1;
wire                       wROB_WayWr1;
wire [2:0]                 wROB_Way2;
wire                       wROB_WayWr2;
wire [2:0]                 wROB_Way3;
wire                       wROB_WayWr3;
reg  [1:0]                 ppReqBank;
reg  [1:0]                 pp1ReqBank;
reg  [1:0]                 pp2ReqBank;
reg  [`ROW_W-1:0]          ppReqRow;
reg  [`ROW_W-1:0]          pp1ReqRow;
reg  [`ROW_W-1:0]          pp2ReqRow;
wire [2+3+`ROW_W-1:0]      wWrData4;
wire [2+3+`ROW_W-1:0]      wIOBEntry0;
wire [2+3+`ROW_W-1:0]      wIOBEntry1;
wire [2+3+`ROW_W-1:0]      wIOBEntry2;
wire [2+3+`ROW_W-1:0]      wIOBEntry3;
reg                        rWr5;
reg  [`ROB_ITEM_W-1:0]     rWrData5;
reg                        rRd5;
wire [`ROB_ITEM_W-1:0]     wRdData5;
wire                       wRdValid5;
wire                       wFull5;
wire                       wEmpty5;

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
            rMACWrInfo  <= {iMAC_AddrWr, iMAC_TagWr, iMAC_IdWr, iMAC_LenWr, iMAC_QoSWr}; 
            ppMACAddrWr <= iMAC_AddrWr;
        end
        if (iMAC_ValidRd) begin 
            rMACRdInfo  <= {iMAC_AddrRd, iMAC_TagRd, iMAC_IdRd, iMAC_LenRd, iMAC_QoSRd}; 
        end
        ppMAC_DataWr   <= iMAC_DataWr;
        pp2MAC_DataWr  <= ppMAC_DataWr;
        ppMAC_MaskWr   <= iMAC_MaskWr;
        pp2MAC_MaskWr  <= ppMAC_MaskWr;
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
                                  nsMacWr = REQ_FETCH_REQ;
                              end else 
                                  nsMacWr = REQ_IDLE;
                          end
        REQ_FETCH_REQ : begin 
                              if (iMAC_EoD) 
                                  nsMacWr = REQ_LAST_DATA;                    
                              else 
                                  nsMacWr = REQ_FETCH_DATA;
                          end 
        REQ_FETCH_DATA: begin 
                              if (iMAC_EoD) 
                                  nsMacWr = REQ_LAST_DATA;                    
                              else 
                                  nsMacWr = REQ_FETCH_DATA;
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
                                  rWrAddr1 <= ppMACAddrWr[6:2];
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
                              end else begin 
                                  nsMacRd = REQ_IDLE;
                              end
                          end
        REQ_FETCH_REQ : begin 
                              if (iMAC_ValidRd) begin
                                  nsMacWr = REQ_FETCH_REQ;                    
                              end else begin 
                                  nsMacRd = REQ_IDLE;
                              end
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


//FIXME : adding a register to control, read and write swithing number
always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rRoundRobin <= 2'b00;
        rRd0        <= 1'b0;
        rRd2        <= 1'b0;
        rLoS        <= 1'b0; //Write Queue in default - Store->1'b0
    end else begin 
        if (~wEmpty0 & wEmpty2 & rRoundRobin ==2'b00) begin 
            rRoundRobin <= 2'b01;
            rRd0        <= 1'b1;
            rRd2        <= 1'b0;
            rLoS        <= 1'b0; //Write Queue - Store->1'b0
        end else if (wEmpty0 & ~wEmpty2 & rRoundRobin == 2'b00) begin 
            rRoundRobin <= 2'b10;
            rRd0        <= 1'b0;
            rRd2        <= 1'b1;
            rLoS        <= 1'b1; //Read Queue - Load->1'b0
        end else if (~wEmpty0 &rRoundRobin == 2'b10 ) begin 
            rRoundRobin <= 2'b01;
            rRd0        <= 1'b1;
            rRd2        <= 1'b0;
            rLoS        <= 1'b0; //Write Queue - Load->1'b0
        end else if (~wEmpty0 &wEmpty2 &rRoundRobin == 2'b01 ) begin 
            rRoundRobin <= 2'b01;
            rRd0        <= 1'b1;
            rRd2        <= 1'b0;
            rLoS        <= 1'b0; //Write Queue - Load->1'b0
        end else if (~wEmpty2 &rRoundRobin == 2'b01) begin 
            rRoundRobin <= 2'b10;
            rRd0        <= 1'b0;
            rRd2        <= 1'b1;
            rLoS        <= 1'b1; //Read Queue - Load->1'b0
        end else if (wEmpty0 & ~wEmpty2 &rRoundRobin == 2'b10) begin 
            rRoundRobin <= 2'b10;
            rRd0        <= 1'b0;
            rRd2        <= 1'b1;
            rLoS        <= 1'b1; //Read Queue - Load->1'b0
        end else begin 
            rRoundRobin <= 2'b00;
            rRd0        <= 1'b0;
            rRd2        <= 1'b0;
            rLoS        <= 1'b0; //Write Queue in default - Store->1'b0
        end 
    end 
end  



always @(posedge clk or negedge resetn) begin 
    if(~resetn) begin 
        rSelBank0  <= 1'b0;
        rSelBank1  <= 1'b0;
        rSelBank2  <= 1'b0;
        rSelBank3  <= 1'b0;
        ppLoS      <= 1'b0;
        ppReqQItem <= 45'b0;
        ppReqBank  <= 2'b0;
        pp1ReqBank <= 2'b0;
        pp2ReqBank <= 2'b0;
        ppReqRow   <= 11'b0;
        pp1ReqRow  <= 11'b0;
        pp2ReqRow  <= 11'b0;
    end else begin 
        ppLoS          <= rLoS;
        ppReqQItem     <= wReqQItem;
        ppReqBank      <= wReqQItem[`ADDR_BANK_RANGE];
        pp1ReqBank     <= ppReqBank;
        pp2ReqBank     <= pp1ReqBank;
        ppReqRow       <= wReqQItem[`ADDR_ROW_RANGE];
        pp1ReqRow      <= ppReqRow;
        pp2ReqRow      <= pp1ReqBank;
  
        if( (|wSelA) && (wReqQItem[`ADDR_BANK_RANGE] == 2'b00) ) begin 
            rSelBank0  <= 1'b1;
            rSelBank1  <= 1'b0;
            rSelBank2  <= 1'b0;
            rSelBank3  <= 1'b0;
        end else if ( (|wSelA) && (wReqQItem[`ADDR_BANK_RANGE] == 2'b01) ) begin 
            rSelBank0  <= 1'b0;
            rSelBank1  <= 1'b1;
            rSelBank2  <= 1'b0;
            rSelBank3  <= 1'b0;
        end else if ( (|wSelA)&& (wReqQItem[`ADDR_BANK_RANGE] == 2'b10)) begin 
            rSelBank0  <= 1'b0;
            rSelBank1  <= 1'b0;
            rSelBank2  <= 1'b1;
            rSelBank3  <= 1'b0;
        end else if ( (|wSelA) && (wReqQItem[`ADDR_BANK_RANGE] == 2'b11) ) begin 
            rSelBank0  <= 1'b0;
            rSelBank1  <= 1'b0;
            rSelBank2  <= 1'b0;
            rSelBank3  <= 1'b1;
        end else begin 
            rSelBank0  <= 1'b0;
            rSelBank1  <= 1'b0;
            rSelBank2  <= 1'b0;
            rSelBank3  <= 1'b0;
        end 
    end 
end 

assign wSelA = (wRdValid0|wRdValid2)? rRoundRobin : 2'b0 ;

mux_4 #(.DATA_WIDTH(45)) muxFourA (
  .iZeroBranch(45'b0),
  .iOneBranch(wRdData0),
  .iTwoBranch(wRdData2),
  .iThreeBranch(45'b0),
  .iSel(wSelA),
  .oMux(wReqQItem)
);

reorder_processor ROP_Bank0 (
  .clk(clk),
  .resetn(resetn),
  .iReqItem(ppReqQItem),
  .iLoS(ppLoS),
  .iValid(rSelBank0),
  .iROB_Rd(rROB_Rd),
  .oROB_Item(wItemBank0),
  .oROB_Way(wROB_Way0),
  .oROB_WayWr(wROB_WayWr0),
  .oROB_Full(wROB_Full0),
  .oROB_Empty(wROB_Empty0)
);

reorder_processor ROP_Bank1 (
  .clk(clk),
  .resetn(resetn),
  .iReqItem(ppReqQItem),
  .iLoS(ppLoS),
  .iValid(rSelBank1),
  .iROB_Rd(rROB_Rd),
  .oROB_Item(wItemBank1),
  .oROB_Way(wROB_Way1),
  .oROB_WayWr(wROB_WayWr1),
  .oROB_Full(wROB_Full0),
  .oROB_Empty(wROB_Empty0)
);

reorder_processor ROP_Bank2 (
  .clk(clk),
  .resetn(resetn),
  .iReqItem(ppReqQItem),
  .iLoS(ppLoS),
  .iValid(rSelBank2),
  .iROB_Rd(rROB_Rd),
  .oROB_Item(wItemBank2),
  .oROB_Way(wROB_Way2),
  .oROB_WayWr(wROB_WayWr2),
  .oROB_Full(wROB_Full2),
  .oROB_Empty(wROB_Empty2)
);

reorder_processor ROP_Bank3 (
  .clk(clk),
  .resetn(resetn),
  .iReqItem(ppReqQItem),
  .iLoS(ppLoS),
  .iValid(rSelBank3),
  .iROB_Rd(rROB_Rd),
  .oROB_Item(rItemBank3),
  .oROB_Way(wROB_Way3),
  .oROB_WayWr(wROB_WayWr3),
  .oROB_Full(wROB_Full3),
  .oROB_Empty(wROB_Empty3)
);


sync_fifo #(.DW(45), .AW(5) ) queueWrReq (
  .clk(clk), 
  .reset_n(resetn),
  .flush(1'b0),
  .wr(rWr0),
  .rd(rRd0),
  .wdata(rWrData0),
  .rdata(wRdData0),
  .rdata_valid(wRdValid0),
  .wfull(wFull0),
  .rempty(wEmpty0)
);

sram_2p #( .AW(5), .DW(32)) queueWrData (
  .clkA(clk),
  .iWrA(rWr1),
  .iAddrA(rWrAddr1),
  .iDataA(rWrData1),
  .clkB(clk),
  .iRdB(rRd1),
  .iAddrB(rRdAddr1),
  .oDataB(wRdData1)
);

sync_fifo #(.DW(45), .AW(5) ) queueRdReq (
  .clk(clk), 
  .reset_n(resetn),
  .flush(1'b0),
  .wr(rWr2),
  .rd(rRd2),
  .wdata(rWrData2),
  .rdata(wRdData2),
  .rdata_valid(wRdValid2),
  .wfull(wFull2),
  .rempty(wEmpty2)
);


mux_4 #(.DATA_WIDTH(1)) muxFourB (
  .iZeroBranch(wROB_WayWr0),
  .iOneBranch(wROB_WayWr1),
  .iTwoBranch(wROB_WayWr2),
  .iThreeBranch(wROB_WayWr3),
  .iSel(pp2ReqBank),
  .oMux(wWr4)
);
assign wIOBEntry0 = {2'b00, wROB_Way0, pp2ReqRow};
assign wIOBEntry1 = {2'b01, wROB_Way1, pp2ReqRow};
assign wIOBEntry2 = {2'b10, wROB_Way2, pp2ReqRow};
assign wIOBEntry3 = {2'b11, wROB_Way3, pp2ReqRow};

mux_4 #(.DATA_WIDTH(16)) muxFourC (
  .iZeroBranch (wIOBEntry0),
  .iOneBranch  (wIOBEntry1),
  .iTwoBranch  (wIOBEntry2),
  .iThreeBranch(wIOBEntry3),
  .iSel(pp2ReqBank),
  .oMux(wWrData4)
);


async_fifo #(.DW(16), .AW(7) ) bufferInOrder (
  .wclk(clk), 
  .wrst_n(resetn),
  .wr(wWr4),
  .wdata(wWrData4),
  .rclk(sclk),
  .rrst_n(resetn),
  .rd(rRd4),
  .rdata(wRdData4),
  .wfull(wFull4),
  .rempty(wEmpty4)
);

sync_fifo #(.DW(`ROB_ITEM_W), .AW(7) ) bufferReOrder (
  .clk(sclk),
  .reset_n(resetn),
  .flush(1'b0),
  .wr(rWr5),
  .wdata(rWrData5),
  .rd(rRd5),
  .rdata(wRdData5),
  .rdata_valid(wRdValid5),
  .wfull(wFull5),
  .rempty(wEmpty5)
);
endmodule 

module reorder_processor (
clk, 
resetn,
iReqItem,
iLoS,
iValid,
iROB_Rd,
oROB_Item,
oROB_Way,
oROB_WayWr,
oROB_Full,
oROB_Empty
);

input  clk;
input  resetn;
input  iReqItem;
input  iLoS;
input  iValid;
input  iROB_Rd;
output oROB_Item;
output oROB_Way;
output oROB_WayWr;
output oROB_Full;
output oROB_Empty;

wire [`REQWR_INFO_W-1:0] iReqItem;
wire                     iLoS;
wire                     iValid;
wire                     iROB_Rd;
reg  [`ROB_ITEM_W-1:0]   oROB_Item;
reg  [2:0]               oROB_Way;
reg                      oROB_WayWr;
reg                      oROB_Full;
reg                      oROB_Empty;
reg  [`ROW_W-1: 0 ]      rReqRow;
reg  [`COL_W-1: 0 ]      rReqCol;
reg  [1:0]               rReqSize; 
reg                      ppValid;
reg                      pp2Valid;
reg                      ppLoS;
reg  [`ROW_W-1: 0 ]      rWrRow;
reg  [`ROW_W-1: 0 ]      ppWrRow;
reg  [23:0]              rWrData;
reg  [23:0]              ppWrData;
reg                      rWrWay0;
reg                      rWrWay1;
reg                      rWrWay2;
reg                      rWrWay3;
reg                      rWrWay4;
reg                      rWrWay5;
reg                      rWrWay6;
reg                      rWrWay7;
reg  [2:0]               rRdDataIndex;
wire [2:0]               wRdDataIndex;


always @(*) begin 
    rReqRow = iReqItem[`ADDR_ROW_RANGE]; 
    rReqCol = iReqItem[`ADDR_COL_RANGE]; 
    rReqSize= iReqItem[`REQ_SIZE_RANGE];
end 
always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        ppValid     <= 1'b0;
        pp2Valid    <= 1'b0;
        ppLoS       <= 1'b0;
        rWrRow      <= 11'b0;
        ppWrRow     <= 11'b0;
        rRdDataIndex<= 4'b0;
        rWrData     <= 24'b0;
        ppWrData    <= 24'b0;
        rWrWay0     <= 1'b0;
        rWrWay1     <= 1'b0;
        rWrWay2     <= 1'b0;
        rWrWay3     <= 1'b0;
        rWrWay4     <= 1'b0;
        rWrWay5     <= 1'b0;
        rWrWay6     <= 1'b0;
        rWrWay7     <= 1'b0;
    end else begin 
        ppValid  <= iValid;
        pp2Valid <= ppValid; 
        ppLoS    <= iLoS;
        ppWrRow  <= rWrRow;
        ppWrData <= rWrData;
        if (ppValid) begin 
            rWrRow       <= rReqRow;
            rRdDataIndex <= wRdDataIndex + 4'b1; //FIXME: when larger than 7, need a flush 
            oROB_Way     <= wRdDataIndex;
            oROB_WayWr   <= 1'b1; //Actually ahead of the way write
            rWrData      <= {rReqCol[6:2], rReqSize, ppLoS, rReqCol, 1'b1 };
            case (wRdDataIndex) 
                 3'b000: begin 
                             rWrWay0 <= 1'b1;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b001: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b1;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b010: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b1;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b011: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b1;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b100: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b1;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b101: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b1;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end
                 3'b110: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b1;
                             rWrWay7 <= 1'b0;
                         end
                 3'b111: begin
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b1;
                         end
                default: begin 
                             rWrWay0 <= 1'b0;
                             rWrWay1 <= 1'b0;
                             rWrWay2 <= 1'b0;
                             rWrWay3 <= 1'b0;
                             rWrWay4 <= 1'b0;
                             rWrWay5 <= 1'b0;
                             rWrWay6 <= 1'b0;
                             rWrWay7 <= 1'b0;
                         end 
            endcase 
        end else begin  
            oROB_Way     <= 3'b000;
            oROB_WayWr   <= 1'b0; 
        end  //end ppValid
    end 
end 


sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way0 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay0),
  .iAddrB(rRdRow),
  .oDataB(wRdData0),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way1 (
  .clkA(clk), 
  .iWrA(rWrWay1),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay1),
  .iAddrB(rRdRow),
  .oDataB(wRdData1),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way2 (
  .clkA(clk), 
  .iWrA(rWrWay2),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay2),
  .iAddrB(rRdRow),
  .oDataB(wRdData2),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way3 (
  .clkA(clk), 
  .iWrA(rWrWay3),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay3),
  .iAddrB(rRdRow),
  .oDataB(wRdData3),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way4 (
  .clkA(clk), 
  .iWrA(rWrWay4),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay4),
  .iAddrB(rRdRow),
  .oDataB(wRdData4),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way5 (
  .clkA(clk), 
  .iWrA(rWrWay5),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay5),
  .iAddrB(rRdRow),
  .oDataB(wRdData5),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way6 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay6),
  .iAddrB(rRdRow),
  .oDataB(wRdData6),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROB_ITEM_W), .AW(`ROW_W)) rob_way7 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(ppWrData),
  .oDataA(), // NO connect
  .clkB(sclk),
  .iRdB(rRdWay7),
  .iAddrB(rRdRow),
  .oDataB(wRdData7),
  .iDataB()  // NO connect
);


//To record which way when the same row has hit
sram_2p #(.DSIZE(4), .ASIZE(`ROW_W)) way_index (
  .clkA(clk), 
  .iWrA(pp2Valid),
  .iAddrA(rReqRow),
  .iDataA(rWrDataIndex),
  .clkB(clk),
  .iRdB(iValid),
  .iAddrB(rReqRow),
  .oDataB(wRdDataIndex)
);

endmodule 
