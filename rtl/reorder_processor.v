//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.26.2014
//Description : Implement the memory_access_controller (MAC)
module reorder_processor (
clk, 
resetn,
iReqItem,
iLoS,
iValid,
iROB_Rd,
iROB_Row,
oROB_ItemValid,
oROB_Item,
oROB_ItemEnd,
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
input  iROB_Row;
output oROB_Item;
output oROB_ItemValid;
output oROB_ItemEnd;
output oROB_Way;
output oROB_WayWr;
output oROB_Full;
output oROB_Empty;

wire [`REQWR_INFO_W-1:0] iReqItem;
wire [1:0]               iLoS;
wire                     iValid;
wire                     iROB_Rd;
wire [`ROW_W-1:0]        iROB_Row;
reg  [`ROP_ITEM_W-1:0]   oROB_Item;
reg                      oROB_ItemEnd;
reg                      oROB_ItemValid;
reg  [2:0]               oROB_Way;
reg                      oROB_WayWr;
reg                      oROB_Full;
reg                      oROB_Empty;
reg  [`ROW_W-1: 0 ]      rReqRow;
reg  [`ROW_W-1: 0 ]      rWrRow;
reg  [`COL_W-1: 0 ]      rReqCol;
reg  [`COL_W-1: 0 ]      rWrCol;
reg  [1:0]               rReqSize; 
reg  [1:0]               rWrSize; 
reg                      ppValid;
reg                      pp2Valid;
reg  [1:0]               ppLoS;
reg  [`ROW_W-1: 0 ]      rRdRow;
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
reg  [2:0]               rWrDataIndex;
wire [2:0]               wRdDataIndex;
reg                      ppROB_Rd;
reg                      pp2ROB_Rd;
reg                      rStartItem;
reg                      rRdWay;
wire [7:0]               wWaySel; 
reg  [2:0]               timer;
wire [`ROP_ITEM_W-1:0]   wRdData0; 
wire [`ROP_ITEM_W-1:0]   wRdData1; 
wire [`ROP_ITEM_W-1:0]   wRdData2; 
wire [`ROP_ITEM_W-1:0]   wRdData3; 
wire [`ROP_ITEM_W-1:0]   wRdData4; 
wire [`ROP_ITEM_W-1:0]   wRdData5; 
wire [`ROP_ITEM_W-1:0]   wRdData6; 
wire [`ROP_ITEM_W-1:0]   wRdData7; 




always @(*) begin 
    rReqRow = iReqItem[`ADDR_ROW_RANGE]; 
    rReqCol = iReqItem[`ADDR_COL_RANGE]; 
    rReqSize= iReqItem[`REQ_SIZE_RANGE];
end 
always @(posedge clk or negedge resetn) begin 
    if (~resetn) begin 
        ppValid        <= 1'b0;
        pp2Valid       <= 1'b0;
        ppLoS          <= 2'b0;
        rWrRow         <= 11'b0;
        ppWrRow        <= 11'b0;
        rWrDataIndex   <= 3'b0;
        rWrData        <= 24'b0;
        oROB_Item      <= 24'b0;
        ppWrData       <= 24'b0;
        rWrWay0        <= 1'b0;
        rWrWay1        <= 1'b0;
        rWrWay2        <= 1'b0;
        rWrWay3        <= 1'b0;
        rWrWay4        <= 1'b0;
        rWrWay5        <= 1'b0;
        rWrWay6        <= 1'b0;
        rWrWay7        <= 1'b0;
        ppROB_Rd       <= 1'b0;
        pp2ROB_Rd      <= 1'b0;
        oROB_ItemValid <= 1'b0;
        oROB_ItemEnd   <= 1'b0;
        rStartItem     <= 1'b0; 
    end else begin 
        ppValid      <= iValid;
        pp2Valid     <= ppValid; 
        ppLoS        <= iLoS;
        ppWrRow      <= rWrRow;
        ppWrData     <= rWrData;
        ppROB_Rd     <= iROB_Rd;
        pp2ROB_Rd    <= ppROB_Rd;
        oROB_Way     <= ppROB_Rd & ppValid ? 3'b000: 
                        ppValid            ? wRdDataIndex : 3'b000;
        rWrDataIndex <= ppROB_Rd & ppValid ? 3'b001 :
                        ppValid            ? wRdDataIndex + 3'b1  : 3'b000; //FIXME: when larger than 7, need a flush 
        if (iValid) begin 
            rWrRow       <= rReqRow;
            rWrCol       <= rReqCol;
            rWrSize      <= rReqSize;
        end else begin 
            rWrRow       <= 11'b0;
            rWrCol       <= 8'b0;
            rWrSize      <= 2'b0;
        end
        if (ppValid) begin 
            oROB_WayWr   <= 1'b1; //Actually ahead of the way write
            rWrData      <= {rWrCol[6:2], rWrSize, ppLoS, rWrCol, 1'b1 };
            way_decode;
        end else begin  
            oROB_Way     <= 3'b000;
            oROB_WayWr   <= 1'b0; 
            rWrData      <= 24'b0;
            rWrWay0      <= 1'b0;
            rWrWay1      <= 1'b0;
            rWrWay2      <= 1'b0;
            rWrWay3      <= 1'b0;
            rWrWay4      <= 1'b0;
            rWrWay5      <= 1'b0;
            rWrWay6      <= 1'b0;
            rWrWay7      <= 1'b0;
        end  //end ppValid
 
        ppROB_Rd <= iROB_Rd;
        if (iROB_Rd) begin 
           rRdWay <= 1'b1;
           rRdRow <= iROB_Row;
        end else begin 
           rRdWay <= 1'b0;
           rRdRow <= 11'b0;
        end 
        oROB_ItemValid <= pp2ROB_Rd ? 1'b1 : 1'b0;
        if (ppROB_Rd) begin 
            rStartItem     <= 1'b1; 
            case (wWaySel) 
                8'b1000_0000: timer<=0;
                8'b1100_0000: timer<=1;
                8'b1110_0000: timer<=2;
                8'b1111_0000: timer<=3;
                8'b1111_1000: timer<=4;
                8'b1111_1100: timer<=5;
                8'b1111_1110: timer<=6;
                8'b1111_1111: timer<=7;
                default :     timer<=0;
            endcase  
        end  
        if (rStartItem) begin 
            if (timer == 3'b0) begin 
                oROB_ItemEnd <= 1'b1;
                rStartItem   <= 1'b0;
            end else begin 
                timer <= timer - 3'b001; 
            end 
            case (timer) 
                3'b000: oROB_Item  <= wRdData0;
                3'b001: oROB_Item  <= wRdData1;
                3'b010: oROB_Item  <= wRdData2;
                3'b011: oROB_Item  <= wRdData3;
                3'b100: oROB_Item  <= wRdData4;
                3'b101: oROB_Item  <= wRdData5;
                3'b110: oROB_Item  <= wRdData6;
                3'b111: oROB_Item  <= wRdData7;
            endcase 
        end else begin 
            oROB_ItemEnd <= 1'b0;
            oROB_Item    <= 24'b0;
        end 
    end 
end 

assign wWaySel = {wRdData0[0], wRdData1[0], wRdData2[0], wRdData3[0], wRdData4[0], wRdData5[0], wRdData6[0], wRdData7[0]};


sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way0 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .iRdB(rRdWay),
  .resetnB(resetn),
  .iAddrB(rRdRow),
  .oDataB(wRdData0),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way1 (
  .clkA(clk), 
  .iWrA(rWrWay1),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData1),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way2 (
  .clkA(clk), 
  .iWrA(rWrWay2),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData2),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way3 (
  .clkA(clk), 
  .iWrA(rWrWay3),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData3),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way4 (
  .clkA(clk), 
  .iWrA(rWrWay4),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData4),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way5 (
  .clkA(clk), 
  .iWrA(rWrWay5),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData5),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way6 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData6),
  .iDataB()  // NO connect
);
sram_2p #(.DW(`ROP_ITEM_W), .AW(`ROW_W)) rob_way7 (
  .clkA(clk), 
  .iWrA(rWrWay0),
  .iAddrA(ppWrRow),
  .iDataA(rWrData),
  .oDataA(), // NO connect
  .clkB(clk),
  .resetnB(resetn),
  .iRdB(rRdWay),
  .iAddrB(rRdRow),
  .oDataB(wRdData7),
  .iDataB()  // NO connect
);


//To record which way when the same row has hit
//When there is write and read to the reprocessor, will there be a conflict to the way_index.
sram_2p #(.DW(4), .AW(`ROW_W)) way_index (
  .clkA(clk), 
  .iWrA(pp2Valid| pp2ROB_Rd),
  .iAddrA(rReqRow),
  .iDataA(rWrDataIndex),
  .clkB(clk),
  .iRdB(iValid|iROB_Rd),
  .resetnB(resetn),
  .iAddrB(rReqRow),
  .oDataB(wRdDataIndex)
);
task way_decode ;
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

endtask

endmodule 
