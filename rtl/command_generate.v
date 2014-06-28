//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.27.2014
//Description : command_generate module is to generate the sram command with read the data from ROB
//              Notice: need to add a cmd_fifo that has no cycle concept, when push out the cycle is added according to the command type. 
`define CMD_DADDR_RANGE 35:25
`define CMD_SIZE_RANGE  24:23
`define CMD_LOS_RANGE   22:21
`define CMD_BANK_RANGE  20:19
`define CMD_ROW_RANGE   18:8
`define CMD_COL_RANGE    7:0


module command_generate (
sclk,
sresetn, 
iROB_Empty,
iROB_Full,
oROB_Rd,
iROB_RdData,
oQWD_Rd,
oQWD_RdAddr,
iQWD_RdData,
oDq,
oClkEn,
oAddr,
oBank,
oCsn,
oRasn,
oCasn,
oWen,
oDqm
);
input    sclk;
input    sresetn;
input    iROB_Empty;
input    iROB_Full;
output   oROB_Rd;
input    iROB_RdData;
output   oQWD_Rd;
output   oQWD_RdAddr;
input    iQWD_RdData;
output   oDq;
output   oClkEn;
output   oAddr;
output   oBank;
output   oCsn;
output   oRasn;
output   oCasn;
output   oWen;
output   oDqm;

wire                       iROB_Empty;
wire                       iROB_Full;
reg                        oROB_Rd;
wire [`ROB_ITEM_W-1:0]     iROB_RdData;
reg                        oQWD_Rd;
reg  [4:0]                 oQWD_RdAddr;
wire [35:0]                iQWD_RdData;
wire [31:0]                oDq;
reg  [31:0]                rDq;
reg  [10:0]                oAddr;
reg  [1 :0]                oBank;
reg                        oClkEn;
reg                        oCsn;
reg                        oRasn;
reg                        oCasn;
reg                        oWen;
reg  [3:0]                 oDqm;
reg  [1:0]                 sCmdRd;
reg  [1:0]                 nsCmdRd;

//Only Four precharged Bank-Row
reg  [1:0]                 rActiveBank[0:3];
//Each bank has a current active Row.
reg  [`ROW_W-1:0]          rActiveRow[0:3];
reg  [1:0]                 rOpenBankRow[0:3];
reg                        rROB_Valid;
reg  [2:0]                 rCmd;
reg  [2:0]                 rPrevCmd;
wire [`ROB_ITEM_W-1:0]     pp0ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp1ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp2ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp3ROB_RdData;

parameter tPR  = 3; //Precharging needs 3 cylces
parameter tACT = 2; //Activate one row needs 2 cylces
parameter numFlush = 4;
parameter tFlush = 8; //Flush pending request within 8cycles


parameter hi_z=32'bz;


parameter CMD_NOP         = 3'b000;
parameter CMD_PRE_ALLBANK = 3'b100;
parameter CMD_PRE_ONEBANK = 3'b101;
parameter CMD_ACT         = 3'b001;
parameter CMD_RD          = 3'b010;
parameter CMD_WR          = 3'b011;
parameter CMD_WRA         = 3'b111;
parameter CMD_RDA         = 3'b110;

parameter CMDRD_IDLE = 2'b00;
parameter CMDRD_EN   = 2'b01;
parameter CMDRD_WAIT = 2'b11;
parameter CMDRD_ACK  = 2'b10;


assign oDq = rDq;

//Logic to control the popped data from ROB
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        sCmdRd <= CMDRD_IDLE;
    end else begin 
        sCmdRd <= nsCmdRd;
    end 
end 
reg rCmdDone;
always @(*) begin 
    nsCmdRd = sCmdRd;
    case (sCmdRd) 
        CMDRD_IDLE : begin 
                         if (~iROB_Empty )
                             nsCmdRd = CMDRD_EN;
                         else 
                             nsCmdRd = CMDRD_IDLE;
                     end 
        CMDRD_EN   : begin 
                         nsCmdRd = CMDRD_IDLE; //ACK is not needed
                     end
        CMDRD_ACK  : begin 
                             nsCmdRd = CMDRD_IDLE;
                     end 
    endcase 
end 

reg  [2:0] rFlushCount;
reg  [4:0] rFlushTimer;
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
            oROB_Rd   <= 1'b0;
            rROB_Valid<= 1'b0;
    end else begin 
        case (sCmdRd) 
            CMDRD_IDLE : begin   
                             oROB_Rd    <= 1'b0;
                             rROB_Valid <= 1'b0;
                         end 
            CMDRD_EN   : begin 
                             oROB_Rd    <= 1'b1;
                             rROB_Valid <= 1'b1;
                         end 
            CMDRD_ACK  : begin 
                             oROB_Rd    <= 1'b0;
                             rROB_Valid <= 1'b0;
                         end 
        endcase 
    end 
end 
//Generator will wait 12 cycles to get 4 CmdData to decide which row and bank to be access.
//If pending request number cannnot be equal to 4, flush signal will be toggled.
reg  [10:0]    rCmdDAddr;
reg  [1:0]     rCmdSize;
reg  [1:0]     rCmdLoS;
reg  [1:0]     rCmdBank;
reg  [10:0]    rCmdRow;
reg  [7:0]     rCmdCol;

always @(*) begin
    rCmdDAddr    =  iROB_RdData[`CMD_DADDR_RANGE];
    rCmdSize     =  iROB_RdData[`CMD_SIZE_RANGE];
    rCmdLoS      =  iROB_RdData[`CMD_LOS_RANGE];
    rCmdBank     =  iROB_RdData[`CMD_BANK_RANGE];
    rCmdRow      =  iROB_RdData[`CMD_ROW_RANGE];
    rCmdCol      =  iROB_RdData[`CMD_COL_RANGE];
end 

//It is better to have 4 queueCmd for each bank, and there is a 
//bank selection algorithm. Now we just have one.
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rActiveBank[0] <= 2'b00;
        rActiveBank[1] <= 2'b00;
        rActiveBank[2] <= 2'b00;
        rActiveBank[3] <= 2'b00;
        rActiveRow[0]  <= 11'b0;
        rActiveRow[1]  <= 11'b0;
        rActiveRow[2]  <= 11'b0;
        rActiveRow[3]  <= 11'b0;
        rWrCmd         <= 1'b0;
        rWrDataCmd     <= 35'b0;
        rOpenBankRow[0]<= 2'b00;
        rOpenBankRow[1]<= 2'b00;
        rOpenBankRow[2]<= 2'b00;
        rOpenBankRow[3]<= 2'b00;
    end else begin 
        if (rROB_Valid) begin 
             if (rCmdBank == rActiveBank[0] && rCmdRow == rActiveRow[0]) begin 
                 plru_bank_row0;
                 if (rCmdLos==2'b01) begin //Write
                     rWrDataCmd <= {rCmdDAddr, rCmdBank, rCmdRow, rCmdCol ,CMD_WR};
                     rWrCmd     <= 1'b1;
                 end else if (rCmdLos==2'b10) begin
                     rWrDataCmd <= {11'b0, rCmdBank, rCmdRow, rCmdCol ,CMD_RD};
                     rWrCmd     <= 1'b1;
                 end 
             end else if (rCmdBank == rActiveBank[1] && rCmdRow == rActiveRow[1]) begin 
                 plru_bank_row1;
                 if (rCmdLos==2'b01) begin //Write
                     rWrDataCmd <= {rCmdDAddr, rCmdBank, rCmdRow, rCmdCol ,CMD_WR};
                     rWrCmd     <= 1'b1;
                 end else if (rCmdLos==2'b10) begin
                     rWrDataCmd <= {11'b0, rCmdBank, rCmdRow, rCmdCol ,CMD_RD};
                     rWrCmd     <= 1'b1;
                 end 
             end else if (rCmdBank == rActiveBank[2] && rCmdRow == rActiveRow[2]) begin 
                 plru_bank_row2;
                 if (rCmdLos==2'b01) begin //Write
                     rWrDataCmd <= {rCmdDAddr, rCmdBank, rCmdRow, rCmdCol ,CMD_WR};
                     rWrCmd     <= 1'b1;
                 end else if (rCmdLos==2'b10) begin
                     rWrDataCmd <= {11'b0, rCmdBank, rCmdRow, rCmdCol ,CMD_RD};
                     rWrCmd     <= 1'b1;
                 end 
             end else if (rCmdBank == rActiveBank[3] && rCmdRow == rActiveRow[3]) begin 
                 plru_bank_row3;
                 if (rCmdLos==2'b01) begin //Write
                     rWrDataCmd <= {rCmdDAddr, rCmdBank, rCmdRow, rCmdCol ,CMD_WR};
                     rWrCmd     <= 1'b1;
                 end else if (rCmdLos==2'b10) begin
                     rWrDataCmd <= {11'b0, rCmdBank, rCmdRow, rCmdCol ,CMD_RD};
                     rWrCmd     <= 1'b1;
                 end 
             end else begin 
                 if (rCmdLos==2'b01) begin //Write
                     rWrDataCmd <= {rCmdDAddr, rCmdBank, rCmdRow, rCmdCol ,CMD_WRA};
                     rWrCmd     <= 1'b1;
                 end else if (rCmdLos==2'b10) begin
                     rWrDataCmd <= {11'b0, rCmdBank, rCmdRow, rCmdCol ,CMD_RDA};
                     rWrCmd     <= 1'b1;
                 end 
                 if (rOpenBankRow[0]==2'b00) begin 
                      rActiveBank[0] = rCmdBank;
                      rActiveRow[0]  = rCmdRow;
                      plru_bank_row0;
                 end else if (rOpenBankRow[1]==2'b00) begin 
                      rActiveBank[1] = rCmdBank;
                      rActiveRow[1]  = rCmdRow;
                      plru_bank_row1;
                 end else if (rOpenBankRow[2]==2'b00) begin 
                      rActiveBank[2] = rCmdBank;
                      rActiveRow[2]  = rCmdRow;
                      plru_bank_row2;
                 end else if (rOpenBankRow[3]==2'b00) begin 
                      rActiveBank[3] = rCmdBank;
                      rActiveRow[3]  = rCmdRow;
                      plru_bank_row3;
                 end
             end 
        end else begin 
             rWrCmd <= 1'b0;
             rWrDataCmd <= 35'b0;
        end 
    end
end 

reg [3:0]  timer;
wire       timerExpired; 
assign timerExpired = ~(|timer);
//Command control logic 
always @(*) begin 
    rOpCmd    = rRdDataCmd[`CMD_OP_RANGE] ;
    rBankCmd  = rRdDataCmd[`CMD_BANK_RANGE];
    rRowCmd   = rRdDataCmd[`CMD_ROW_RANGE];
    rColCmd   = rRdDataCmd[`CMD_COL_RANGE];
    rDAddrCmd = rRdDataCmd[`CMD_DADDR_RANGE]; 
    rDataCmd  = iQWD_RdData[`CMD_DATA_RANGE];
    rMaskCmd  = iQWD_RdData[`CMD_MASK_RANGE];
end 

//timer controller
always @(posedge sclk or negedge sresetn) begin 
    if(~sresetn) begin 
        rDoWrP <= 1'b0;
    end else begin 
       if (wRdValidCmd) begin 
           case (rOpCmd)
              CMD_WRA :  begin 
                             rDoWrP    <= 1'b1;
                             rFetchWrData <= 1'b1;
                         end 
           endcase  
       end 
    end 
end 

always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rCmd <= CMD_PRE_ALLBANK;
    end else begin 
        if (rDoWrP) begin 
             timer <= timer - 1'b1;
        end else begin 
             timer <= 4'b0000;
        end 
        if (rDoWrP) begin 
             rCmd <= CMD_ACT;
             timer <= tACT;
        end  else if (rDoWrP & timerExpired & rCmd==CMD_PRE_ACT) begin 
             rCmd <= CMD_WR;
             timer <= tWR;
        end  else if (rDoWrP & timerExpired & rCmd==CMD_WR) begin 
             rCmd <= CMD_AP; 
             timer <= tAP; 
        end 
        rFetchWrData <= rFetchWrData ? 1'b0 : 1'b0;
        oQWD_Rd      <= rFetchWrData;
        oQWD_Addr    <= rColCmd[6:2];
    end  
end 

always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        oClkEn<= 1'b0;
        rDq   <= 32'b0;
        oCasn <= 1'b1;
        oRasn <= 1'b1;
        oCasn <= 1'b1;
        oWen  <= 1'b0;
        oDqm  <= 4'b0;
        oBank <= 2'b0;
        oAddr <= 10'b0;            
    end else begin 
        case (rCmd) 
            CMD_PRE_ALLBANK: precharge_all_bank(0, hi_z);
            CMD_NOP        : nop(0, hi_z);
            CMD_ACT        : active(rBankCmd, rRowCmd, hi_z);
            CMD_WR         : write(rBankCmd, rColCmd, rDataCmd, rMaskCmd );
        endcase

    end 
end 

sync_fifo#(.DW(), .AW())  queueCmd (
  .clk(sclk),
  .resetn_n(sresetn),
  .flush(1'b0),
  .wr(rWrCmd),
  .rd(rRdCmd),
  .wdata(rWrDataCmd),
  .rdata(rRdDataCmd),
  .rdata_valid(wRdValidCmd),
  .wfull(wFullCmd),
  .rempty(wEmptyCmd)

);


task precharge_all_bank;
    input  [3 : 0] dqm_in;
    input [31 : 0] dq_in;
    begin
        oClkEn<= 1'b1;
        oCsn  <= 1'b0;
        oRasn <= 1'b0;
        oCasn <= 1'b0;
        oWen  <= 1'b0;
        oDqm  <= dqm_in;
        oBank <= 2'b0;
        oAddr <= 1024;            // A10 = 1
        rDq   <= dq_in;
    end
endtask 
task nop;
    input  [3 : 0] dqm_in;
    input [31 : 0] dq_in;
    begin
        oClkEn <= 1'b1;
        oCsn   <= 1'b0;
        oRasn  <= 1'b1;
        oCasn  <= 1'b1;
        oWen   <= 1'b1;
        oDqm   <= dqm_in;
        //ba    = 0;
        //addr  = 0;
        rDq    = dq_in;
    end
endtask
task active;
    input  [1 : 0] bank;
    input [10 : 0] row;
    input [31 : 0] dq_in;
    begin
        oClkEn   <= 1'b1;
        oCsn     <= 1'b0;
        oRasn    <= 1'b0;
        OCasn    <= 1'b1;
        oWen     <= 1'b1;
        oDqm     <= 1'b0;
        oBank    <= bank;
        oAddr    <= row;
        rDq      <= dq_in;
    end
endtask
task write;
    input  [1 : 0] bank;
    input [10 : 0] column;
    input [31 : 0] dq_in;
    input  [3 : 0] dqm_in;
    begin
        cke   = 1;
        cs_n  = 0;
        ras_n = 1;
        cas_n = 0;
        we_n  = 0;
        dqm   = dqm_in;
        ba    = bank;
        addr  = column;
        dq    = dq_in;
    end
endtask

task plru_bank_row0; 
    rOpenBankRow[0] <= 2'b11;
    rOpenBankRow[1] <= rOpenBankRow[1]==2'b00 ? rOpenBankRow[1]: rOpenBankRow[1]-2'b1 ; 
    rOpenBankRow[2] <= rOpenBankRow[2]==2'b00 ? rOpenBankRow[2]: rOpenBankRow[2]-2'b1 ; 
    rOpenBankRow[3] <= rOpenBankRow[3]==2'b00 ? rOpenBankRow[3]: rOpenBankRow[3]-2'b1 ; 
endtask
task plru_bank_row1; 
    rOpenBankRow[1] <= 2'b11;
    rOpenBankRow[0] <= rOpenBankRow[0]==2'b00 ? rOpenBankRow[0]: rOpenBankRow[0]-2'b1 ; 
    rOpenBankRow[2] <= rOpenBankRow[2]==2'b00 ? rOpenBankRow[2]: rOpenBankRow[2]-2'b1 ; 
    rOpenBankRow[3] <= rOpenBankRow[3]==2'b00 ? rOpenBankRow[3]: rOpenBankRow[3]-2'b1 ; 
endtask
task plru_bank_row2; 
    rOpenBankRow[2] <= 2'b11;
    rOpenBankRow[1] <= rOpenBankRow[1]==2'b00 ? rOpenBankRow[1]: rOpenBankRow[1]-2'b1 ; 
    rOpenBankRow[0] <= rOpenBankRow[0]==2'b00 ? rOpenBankRow[0]: rOpenBankRow[0]-2'b1 ; 
    rOpenBankRow[3] <= rOpenBankRow[3]==2'b00 ? rOpenBankRow[3]: rOpenBankRow[3]-2'b1 ; 
endtask
task plru_bank_row3; 
    rOpenBankRow[3] <= 2'b11;
    rOpenBankRow[1] <= rOpenBankRow[1]==2'b00 ? rOpenBankRow[1]: rOpenBankRow[1]-2'b1 ; 
    rOpenBankRow[2] <= rOpenBankRow[2]==2'b00 ? rOpenBankRow[2]: rOpenBankRow[2]-2'b1 ; 
    rOpenBankRow[0] <= rOpenBankRow[0]==2'b00 ? rOpenBankRow[0]: rOpenBankRow[0]-2'b1 ; 
endtask
endmodule 
