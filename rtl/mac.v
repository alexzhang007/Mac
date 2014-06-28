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
`define ROP_ITEM_W 24
`define ROB_ITEM_W 36
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
oClkEn,
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

parameter IOB_IDLE = 2'b00;
parameter IOB_FIRST= 2'b01;
parameter IOB_WAIT = 2'b10; 
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
output   oClkEn;
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
reg                        oClkEn;
reg  [10:0]                oAddr;
reg  [1:0]                 oBank;
reg                        oCsn;
reg                        oRasn;
reg                        oCasn;
reg                        oWen;
reg  [3:0]                 oDqm;
wire [31:0]                wDq;
reg  [31:0]                rDq;
wire                       wClkEn;
wire [10:0]                wAddr;
wire [1:0]                 wBank;
wire                       wCsn;
wire                       wRasn;
wire                       wCasn;
wire                       wWen;
wire [3:0]                 wDqm;
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
reg                        rRd4;
wire [2+3+`ROW_W-1:0]      wIOBEntry0;
wire [2+3+`ROW_W-1:0]      wIOBEntry1;
wire [2+3+`ROW_W-1:0]      wIOBEntry2;
wire [2+3+`ROW_W-1:0]      wIOBEntry3;
reg                        rWr5;
reg  [`ROB_ITEM_W-1:0]     rWrData5;
wire                       wRd5;
wire [`ROB_ITEM_W-1:0]     wRdData5;
wire                       wRdValid5;
wire                       wFull5;
wire                       wEmpty5;
reg  [1 :0]                rBankSel;
reg  [10:0]                rRowSel;
reg  [10:0]                ppRowSel;
reg                        ppRdValid4;
reg                        rROB_Rd0;
reg                        rROB_Rd1;
reg                        rROB_Rd2;
reg                        rROB_Rd3;
wire                       wEmpty4;
wire                       wRdValid4;
wire [15:0]                wRdData4;
wire [`ROP_ITEM_W-1:0]     wItemBank0;
wire [`ROP_ITEM_W-1:0]     wItemBank1;
wire [`ROP_ITEM_W-1:0]     wItemBank2;
wire [`ROP_ITEM_W-1:0]     wItemBank3;
reg  [1:0]                 sIOB;
reg  [1:0]                 nsIOB;
reg                        rSucROB0;
reg                        rSucROB1;
reg                        rSucROB2;
reg                        rSucROB3;
wire                       wROB_ItemEnd0;
wire                       wROB_ItemEnd1;
wire                       wROB_ItemEnd2;
wire                       wROB_ItemEnd3;
wire                       wROB_ItemValid0;
wire                       wROB_ItemValid1;
wire                       wROB_ItemValid2;
wire                       wROB_ItemValid3;



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
        pp2ReqRow      <= pp1ReqRow;
  
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


//Control the IOB
//Pop one item from IOB, until it has been processed.
//oROB_ItemEnd can be used as an indicator.


always @(posedge  clk or negedge resetn ) begin 
    if (~resetn) begin 
        sIOB <= IOB_IDLE; 
    end else begin 
        sIOB <= nsIOB;
    end 
end 

always @(*) begin 
    nsIOB = sIOB;
    case (sIOB) 
        IOB_IDLE : begin  
                       if (~wEmpty4) 
                           nsIOB = IOB_FIRST;
                       else 
                           nsIOB = IOB_IDLE;
                   end
        IOB_FIRST: begin 
                       nsIOB = IOB_WAIT;
                   end  
        IOB_WAIT : begin 
                       if (~wEmpty4&&(wROB_ItemEnd0 | wROB_ItemEnd1 | wROB_ItemEnd2 | wROB_ItemEnd3))
                           nsIOB = IOB_FIRST;
                       else 
                           nsIOB = IOB_WAIT;
                   end 
    endcase
end 
//Output rRd4
always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rRd4       <= 1'b0;
    end else begin 
        case (sIOB) 
            IOB_IDLE : rRd4 <= 1'b0;
            IOB_FIRST: rRd4 <= 1'b1;
            IOB_WAIT : rRd4 <= 1'b0;
        endcase
    end 
end 

always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rBankSel   <= 2'b00;
        ppRdValid4 <= 1'b0;
        rRowSel    <= 11'b0;
        ppRowSel   <= 11'b0;
        rROB_Rd0   <= 1'b0;
        rROB_Rd1   <= 1'b0;
        rROB_Rd2   <= 1'b0;
        rROB_Rd3   <= 1'b0;
    end else begin 
        ppRdValid4 <= wRdValid4;
        if (wRdValid4) begin 
             rBankSel <= wRdData4[15:14];
             rRowSel  <= wRdData4[10:0];
        end else begin 
             rBankSel <= 2'b00;
             rRowSel  <= 11'b0;
        end 
        if (ppRdValid4) begin 
             ppRowSel <= rRowSel;
             case (rBankSel) 
                 2'b00 : rROB_Rd0 <= 1'b1;
                 2'b01 : rROB_Rd1 <= 1'b1;
                 2'b10 : rROB_Rd2 <= 1'b1;
                 2'b11 : rROB_Rd3 <= 1'b1;
             endcase 
        end else begin 
             rROB_Rd0 <= 1'b0;
             rROB_Rd1 <= 1'b0;
             rROB_Rd2 <= 1'b0;
             rROB_Rd3 <= 1'b0;
        end 
    end 
end 


//Process the output from the reorder_processor and IOB
//Output to the ROB
//wROB_ItemValid0, wROB_ItemValid1, wROB_ItemValid2, and 
//wROB_ItemValid3 will be triggered one frame by frame. 
always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        rWrData5 <= `ROB_ITEM_W'b0;
        rWr5     <= 1'b0;
        rSucROB0 <= 1'b0;
        rSucROB1 <= 1'b0;
        rSucROB2 <= 1'b0;
        rSucROB3 <= 1'b0;
    end else begin 
        if (wROB_ItemValid0 | rSucROB0) begin 
            rWrData5 <= {wItemBank0[23:9],wRdData4[15:14],wRdData4[10:0],wItemBank0[8:1]};
            rWr5     <= 1'b1;
            rSucROB0 <= wROB_ItemEnd0 ? 1'b0 : 1'b1;
        end else if (wROB_ItemValid1 | rSucROB1) begin 
            rWrData5 <= {wItemBank1[23:9],wRdData4[15:14],wRdData4[10:0],wItemBank1[8:1]};
            rWr5     <= 1'b1;
            rSucROB1 <= wROB_ItemEnd1 ? 1'b0 : 1'b1;
        end else if (wROB_ItemValid2 | rSucROB2) begin 
            rWrData5 <= {wItemBank2[23:9],wRdData4[15:14],wRdData4[10:0],wItemBank2[8:1]};
            rWr5     <= 1'b1;
            rSucROB2 <= wROB_ItemEnd2 ? 1'b0 : 1'b1;
        end else if (wROB_ItemValid3 | rSucROB3) begin 
            rWrData5 <= {wItemBank3[23:9],wRdData4[15:14],wRdData4[10:0],wItemBank3[8:1]};
            rWr5     <= 1'b1;
            rSucROB3 <= wROB_ItemEnd3 ? 1'b0 : 1'b1;
        end else begin
            rWr5     <= 1'b0;
            rWrData5 <= `ROB_ITEM_W'b0;
        end 
    end 
end 

always @(posedge sclk) begin 
    oClkEn<= wClkEn;
    oAddr <= wAddr;
    oBank <= wBank;
    oCsn  <= wCsn;
    oRasn <= wRasn;
    oCasn <= wCasn;
    oWen  <= wWen;
    oDqm  <= wDqm;
    rDq   <= wDq;
end 
assign ioDq = rDq;

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
  .iROB_Rd(rROB_Rd0),
  .iROB_Row(ppRowSel),
  .oROB_Item(wItemBank0),
  .oROB_ItemValid(wROB_ItemValid0),
  .oROB_ItemEnd(wROB_ItemEnd0),
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
  .iROB_Rd(rROB_Rd1),
  .iROB_Row(ppRowSel),
  .oROB_Item(wItemBank1),
  .oROB_ItemValid(wROB_ItemValid1),
  .oROB_ItemEnd(wROB_ItemEnd1),
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
  .iROB_Rd(rROB_Rd2),
  .iROB_Row(ppRowSel),
  .oROB_Item(wItemBank2),
  .oROB_ItemValid(wROB_ItemValid2),
  .oROB_ItemEnd(wROB_ItemEnd2),
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
  .iROB_Rd(rROB_Rd3),
  .iROB_Row(ppRowSel),
  .oROB_Item(wItemBank3),
  .oROB_ItemValid(wROB_ItemValid3),
  .oROB_ItemEnd(wROB_ItemEnd3),
  .oROB_Way(wROB_Way3),
  .oROB_WayWr(wROB_WayWr3),
  .oROB_Full(wROB_Full3),
  .oROB_Empty(wROB_Empty3)
);

command_generate CMD_Gen (
  .sclk(sclk),
  .sresetn(resetn),
  .iROB_Empty(wEmpty5),
  .iROB_Full(wFull5),
  .oROB_Rd(wRd5),
  .iROB_RdData(wRdData5),
  .oQWD_Rd(wRd1), //Rd QueueWriteData
  .oQWD_RdAddr(wRdAddr1), 
  .iQWD_RdData(wRdData1),
  .oDq(wDq),
  .oClkEn(wClkEn),
  .oAddr(wAddr),
  .oBank(wBank),
  .oCsn(wCsn),
  .oRasn(wRasn),
  .oCasn(wCasn),
  .oWen(wWen),
  .oDqm(wDqm)
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

sram_2p #( .AW(5), .DW(36)) queueWrData (
  .clkA(clk),
  .iWrA(rWr1),
  .iAddrA(rWrAddr1),
  .iDataA(rWrData1),
  .clkB(sclk),
  .iRdB(wRd1),
  .iAddrB(wRdAddr1),
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


sync_fifo #(.DW(16), .AW(7) ) bufferInOrder (
  .clk(clk), 
  .reset_n(resetn),
  .flush(1'b0),
  .wr(wWr4),
  .wdata(wWrData4),
  .rd(rRd4),
  .rdata(wRdData4),
  .rdata_valid(wRdValid4),
  .wfull(wFull4),
  .rempty(wEmpty4)
);

async_fifo #(.DW(`ROB_ITEM_W), .AW(7) ) bufferReOrder (
  .wclk(clk),
  .wrst_n(resetn),
  .wr(rWr5),
  .wdata(rWrData5),
  .rclk(sclk),
  .rrst_n(resetn),
  .rd(wRd5),
  .rdata(wRdData5),
  .wfull(wFull5),
  .rempty(wEmpty5)
);
endmodule 

