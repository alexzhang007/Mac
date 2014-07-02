//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.27.2014
//Description : command_generate module is to generate the sram command with read the data from ROB
//              Notice: need to add a cmd_fifo that has no cycle concept, when push out the cycle is added according to the command type. 
//              Notice: DRAM controller has a 64ms, 4096-cycle refresh requirement, the device must have a REFRESH command issued to it at lea
//                      every 64ms/4096 = 15.62509 us.
//                      The DRAM is driven by sclk (300MHz) the value of tRFC should be 15.62509us/3.334ns=4686
//              Notice: Not support the different bank precharging simultaneously. 
//              Notice: Row management is adaptive policy: When there is same row hit, select Open Row Policy; 
//                      Otherwise it is Close Row Policy.
//
`define ROB_DADDR_RANGE 35:25
`define ROB_SIZE_RANGE  24:23
`define ROB_LOS_RANGE   22:21
`define ROB_BANK_RANGE  20:19
`define ROB_ROW_RANGE   18:8
`define ROB_COL_RANGE    7:0

`define CMD_DADDR_RANGE 34:24
`define CMD_BANK_RANGE  23:22
`define CMD_ROW_RANGE   21:11
`define CMD_COL_RANGE   10:3
`define CMD_OP_RANGE     3:0
`define CMD_DATA_RANGE  35:4
`define CMD_MASK_RANGE   3:0


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
reg  [2:0]                 sCmdRd;
reg  [2:0]                 nsCmdRd;

//Only Four precharged Bank-Row
reg  [1:0]                 rActiveBank[0:3];
//Each bank has a current active Row.
reg  [`ROW_W-1:0]          rActiveRow[0:3];
reg  [1:0]                 rOpenBankRow[0:3];
reg                        rROB_Valid;
reg  [3:0]                 rCmd;
reg  [2:0]                 rPrevCmd;
wire [`ROB_ITEM_W-1:0]     pp0ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp1ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp2ROB_RdData;
wire [`ROB_ITEM_W-1:0]     pp3ROB_RdData;
reg  [10:0]                rRobDAddr;
reg  [1:0]                 rRobSize;
reg  [1:0]                 rRobLoS;
reg  [1:0]                 rRobBank;
reg  [10:0]                rRobRow;
reg  [7:0]                 rRobCol;
reg                        rWrCmd;
reg                        rRdCmd;
reg  [35:0]                rWrDataCmd;
reg  [35:0]                rActiveDataCmd[0:3];
wire                       wEmptyCmd;
wire                       wFullCmd;
wire                       wRdValidCmd;
reg  [3:0]                 rOpCmd;
reg  [10:0]                rRowCmd;
reg  [7:0]                 rColCmd;
reg  [1:0]                 rBankCmd;
reg  [10:0]                rDAddrCmd;
reg  [31:0]                rDataCmd;
reg  [3:0]                 rMaskCmd;
reg  [3:0]                 timer;
wire                       timerExpired; 
reg  [15:0]                timerRefresh;
wire                       timerExpiredRefresh;
wire [35:0]                wRdDataCmd;
reg                        rDoWrP;
reg                        rFetchWrData;
reg                        rHitBtB;
reg  [2:0]                 rFlushCount;
reg  [4:0]                 rFlushTimer;
wire                       wFlushExpired ;
reg                        rCmdDone;
reg  [3:0]                 timerROB;
wire                       wROBExpired ;
reg                        rDoPreAllBank;
reg                        rDoLoadReg;

reg  [31:0]                rBurstMode;

parameter tPR  = 3;   //Precharging duration is 10ns, needs 3 cylces
parameter tWR  = 2;
parameter tRFC = 4686; 
parameter tCBR = 9 ;  //CAS Before RAS, AutoRefresh duration is 9 cycles. 
parameter tRRD = 3 ;  //Row to Row active delay is 10ns, NA in this version   
parameter tRCD = 3;   //Row Activation duration is 10ns. 
parameter tRAS = 12;  //Row active to Auto Precharging 40ns;  
parameter tCAS = 3 ;  //Column Access Strobe latency. Duration between column access command and data return by DRAM device. Also known as tCL
parameter tRC  = 15;  //Row cycle. Time duration accesses to different rows in same bank. tRC = tRAS + tRP;
parameter tCP  = 6 ;  //Colum access to Auto Precharging tCP= tRAS-tRCD-tCAS <Self Define Parameter>
parameter tCWD = 1 ;  //Column Wite Delay. Time interval between issuuance of column write command and placement of data on data bus by the DRAM controller. Here it is 1 cycle for DDR SDRAM. 
parameter tLD  = 5;   //LoaD registers into SDRAM
parameter tBL  = 8;   //Read Data Burst length. 

parameter nFlush = 4;
parameter tFlush = 15; //Flush pending request within tRC cycles


parameter hi_z=32'bz;


parameter CMD_NOP         = 4'b0000;
parameter CMD_PRE_ALLBANK = 4'b0100;
parameter CMD_PRE_ONEBANK = 4'b0101;
parameter CMD_ACT         = 4'b0001;
parameter CMD_RD          = 4'b0010;
parameter CMD_WR          = 4'b0011;
parameter CMD_WRA         = 4'b0111;
parameter CMD_RDA         = 4'b0110;
parameter CMD_SELF_REFRESH= 4'b0111;
parameter CMD_DATA        = 4'b1000;
parameter CMD_LOAD_REG    = 4'b1111;

parameter CMDRD_INIT = 3'b100;
parameter CMDRD_INIT2= 3'b110;
parameter CMDRD_LOAD = 3'b101;
parameter CMDRD_LOAD2= 3'b111;
parameter CMDRD_IDLE = 3'b000;
parameter CMDRD_EN   = 3'b001;
parameter CMDRD_WAIT = 3'b011;
parameter CMDRD_ACK  = 3'b010;


assign oDq = rDq;

//Logic to control the popped data from ROB
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        sCmdRd <= CMDRD_INIT;
        rBurstMode <= { 24'b0,3'b11,1'b0,1'b0,3'b000};
    end else begin 
        sCmdRd <= nsCmdRd;
    end 
end 
always @(*) begin 
    nsCmdRd = sCmdRd;
    case (sCmdRd) 
        CMDRD_INIT : begin 
                         nsCmdRd = CMDRD_INIT2;
                     end
        CMDRD_INIT2: begin 
                         if (wROBExpired)
                             nsCmdRd = CMDRD_LOAD;
                         else 
                             nsCmdRd = CMDRD_INIT2;
                     end
        CMDRD_LOAD : begin 
                         nsCmdRd = CMDRD_LOAD2;
                     end
        CMDRD_LOAD2: begin 
                         if (wROBExpired)
                             nsCmdRd = CMDRD_IDLE;
                         else 
                             nsCmdRd = CMDRD_LOAD2;
                     end
        CMDRD_IDLE : begin 
                         if (~iROB_Empty )
                             nsCmdRd = CMDRD_EN;
                         else 
                             nsCmdRd = CMDRD_IDLE;
                     end 
        CMDRD_EN   : begin 
                         nsCmdRd = CMDRD_ACK; //ACK is needed since the Empty is holden for 3 cycles
                     end
        CMDRD_ACK  : begin 
                             nsCmdRd = CMDRD_IDLE;
                     end 
    endcase 
end 

assign wROBExpired = ~(|timerROB) ;

always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
            oROB_Rd       <= 1'b0;
            rROB_Valid    <= 1'b0;
            timerROB      <= 4'b1111;
            rDoPreAllBank <= 1'b0;
            rDoLoadReg    <= 1'b0;
    end else begin 
        timerROB <= (rDoPreAllBank|rDoLoadReg) ? timerROB - 4'b1 : timerROB;
        case (sCmdRd) 
            CMDRD_INIT : begin  //precharging all the banks
                             timerROB      <= tCBR -1;
                             rDoPreAllBank <= 1'b1;
                             rDoLoadReg    <= 1'b0;
                         end
            CMDRD_INIT2: begin  //precharging all the banks
                             rDoPreAllBank <= 1'b1;
                             rDoLoadReg    <= 1'b0;
                         end
            CMDRD_LOAD : begin 
                             timerROB      <= tLD -1;
                             rDoPreAllBank <= 1'b0;
                             rDoLoadReg    <= 1'b1;
                         end 
            CMDRD_LOAD : begin 
                             rDoPreAllBank <= 1'b0;
                             rDoLoadReg    <= 1'b1;
                         end
            CMDRD_IDLE : begin   
                             oROB_Rd       <= 1'b0;
                             rROB_Valid    <= 1'b0;
                             rDoPreAllBank <= 1'b0;
                             rDoLoadReg    <= 1'b0;
                         end 
            CMDRD_EN   : begin 
                             oROB_Rd       <= 1'b1;
                             rROB_Valid    <= 1'b1;
                         end 
            CMDRD_ACK  : begin 
                             oROB_Rd       <= 1'b0;
                             rROB_Valid    <= 1'b0;
                         end 
        endcase 
    end 
end 


//When rROB_Valid is asserted, rFlushCount++; when rFlushCount==2, it will push the ActiveBank/Row/Col w/o AutoPrecharge into CmdQueue. 
//Otherwise, it will wait 6 cycles to flush the ActiveBank/Row/Col w/ AutoPrecharge into CmdQueue; 
assign wFlushExpired = ~(|rFlushTimer);
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rFlushCount <= 3'b0;
        rFlushTimer <= 5'b11111;
    end else begin 
        rFlushTimer <= (rROB_Valid| wFlushExpired) ? tFlush :
                       (rFlushCount >3'b000)       ? rFlushTimer - 5'b1 : rFlushTimer ;
        if (rROB_Valid) begin 
            rFlushCount <= rFlushCount + 3'b1;
        end  else if (wFlushExpired |rHitBtB) begin  
            rFlushCount <= rFlushCount - 3'b1;
        end  else if (rFlushCount == nFlush) begin 
            rFlushCount <= rFlushCount - 3'b1;
        end  
    end 
end

//Generator will wait 12 cycles to get 4 CmdData to decide which row and bank to be access.
//If pending request number cannnot be equal to 4, flush signal will be toggled.

always @(*) begin
    rRobDAddr    =  iROB_RdData[`ROB_DADDR_RANGE];
    rRobSize     =  iROB_RdData[`ROB_SIZE_RANGE];
    rRobLoS      =  iROB_RdData[`ROB_LOS_RANGE];
    rRobBank     =  iROB_RdData[`ROB_BANK_RANGE];
    rRobRow      =  iROB_RdData[`ROB_ROW_RANGE];
    rRobCol      =  iROB_RdData[`ROB_COL_RANGE];
end 

//It is better to have 4 queueCmd for each bank, and there is a 
//bank selection algorithm. Now we just have one.
//
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rActiveBank[0]    <= 2'b00;
        rActiveBank[1]    <= 2'b00;
        rActiveBank[2]    <= 2'b00;
        rActiveBank[3]    <= 2'b00;
        rActiveRow[0]     <= 11'b0;
        rActiveRow[1]     <= 11'b0;
        rActiveRow[2]     <= 11'b0;
        rActiveRow[3]     <= 11'b0;
        rWrCmd            <= 1'b0;
        rHitBtB           <= 1'b0;
        rWrDataCmd        <= 36'b0;
        rOpenBankRow[0]   <= 2'b00;
        rOpenBankRow[1]   <= 2'b00;
        rOpenBankRow[2]   <= 2'b00;
        rOpenBankRow[3]   <= 2'b00;
        rActiveDataCmd[0] <= 36'b0;
        rActiveDataCmd[1] <= 36'b0;
        rActiveDataCmd[2] <= 36'b0;
        rActiveDataCmd[3] <= 36'b0;
    end else begin 
        if (rROB_Valid ) begin 
            if (rRobBank == rActiveBank[0] && rRobRow == rActiveRow[0]) begin 
                rWrDataCmd        <= rActiveDataCmd[0][3:0] == CMD_WRA ? {rActiveDataCmd[0][35:4], CMD_WR } :
                                     rActiveDataCmd[0][3:0] == CMD_RDA ? {rActiveDataCmd[0][35:4], CMD_RD } :  36'b0   ;
                rActiveDataCmd[0] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                     rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                plru_bank_row0;
                rWrCmd            <= 1'b1;
                rHitBtB           <= 1'b1;  //Hit Back to Back Read or Write
            end else if (rRobBank == rActiveBank[1] && rRobRow == rActiveRow[1]) begin 
                rWrDataCmd        <= rActiveDataCmd[1][3:0] == CMD_WRA ? {rActiveDataCmd[1][35:4], CMD_WR } :
                                     rActiveDataCmd[1][3:0] == CMD_RDA ? {rActiveDataCmd[1][35:4], CMD_RD } :  36'b0   ;
                rActiveDataCmd[1] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                     rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                plru_bank_row1;
                rWrCmd            <= 1'b1;
                rHitBtB           <= 1'b1;  //Hit Back to Back Read or Write
            end else if (rRobBank == rActiveBank[2] && rRobRow == rActiveRow[2]) begin 
                rWrDataCmd        <= rActiveDataCmd[2][3:0] == CMD_WRA ? {rActiveDataCmd[2][35:4], CMD_WR } :
                                     rActiveDataCmd[2][3:0] == CMD_RDA ? {rActiveDataCmd[2][35:4], CMD_RD } :  36'b0   ;
                rActiveDataCmd[2] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                     rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                plru_bank_row2;
                rWrCmd            <= 1'b1;
                rHitBtB           <= 1'b1;  //Hit Back to Back Read or Write
            end else if (rRobBank == rActiveBank[3] && rRobRow == rActiveRow[3]) begin 
                rWrDataCmd        <= rActiveDataCmd[3][3:0] == CMD_WRA ? {rActiveDataCmd[3][35:4], CMD_WR } :
                                     rActiveDataCmd[3][3:0] == CMD_RDA ? {rActiveDataCmd[3][35:4], CMD_RD } :  36'b0   ;
                rActiveDataCmd[3] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                     rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                plru_bank_row3;
                rWrCmd            <= 1'b1;
                rHitBtB           <= 1'b1;  //Hit Back to Back Read or Write
            end else begin 
                if (rOpenBankRow[0]==2'b00) begin 
                     rActiveBank[0]    <= rRobBank;
                     rActiveRow[0]     <= rRobRow;
                     rActiveDataCmd[0] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                          rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                     plru_bank_row0;
                end else if (rOpenBankRow[1]==2'b00) begin 
                     rActiveBank[1]    <= rRobBank;
                     rActiveRow[1]     <= rRobRow;
                     rActiveDataCmd[1] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                          rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                     plru_bank_row1;
                end else if (rOpenBankRow[2]==2'b00) begin 
                     rActiveBank[2]    <= rRobBank;
                     rActiveRow[2]     <= rRobRow;
                     rActiveDataCmd[2] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
                                          rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                     plru_bank_row2;
                end else if (rOpenBankRow[3]==2'b00) begin 
                     rActiveBank[3]    <= rRobBank;
                     rActiveRow[3]     <= rRobRow;
	             rActiveDataCmd[3] <= rRobLoS==2'b01 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_WRA } :
	            		      rRobLoS==2'b10 ? {rRobDAddr, rRobBank, rRobRow, rRobCol, CMD_RDA } : 36'b0;
                     plru_bank_row3;
                end
             end
        end else if (rFlushCount == nFlush ) begin 
             if (rOpenBankRow[0]==2'b00) begin 
                 rWrDataCmd <= rActiveDataCmd[0];
                 rWrCmd     <= 1'b1;
             end else if (rOpenBankRow[1]==2'b00) begin 
                 rWrDataCmd <= rActiveDataCmd[1];
                 rWrCmd     <= 1'b1;
             end else if (rOpenBankRow[2]==2'b00) begin 
                 rWrDataCmd <= rActiveDataCmd[2];
                 rWrCmd     <= 1'b1;
             end else if (rOpenBankRow[3]==2'b00) begin 
                 rWrDataCmd <= rActiveDataCmd[3];
                 rWrCmd     <= 1'b1;
             end
        end else if (wFlushExpired ) begin 
             if (rOpenBankRow[0]!=2'b00) begin 
                 rWrDataCmd     <= rActiveDataCmd[0];
                 rWrCmd         <= 1'b1;
                 rOpenBankRow[0]<= 2'b00;
             end else if (rOpenBankRow[1]!=2'b00) begin 
                 rWrDataCmd     <= rActiveDataCmd[1];
                 rWrCmd         <= 1'b1;
                 rOpenBankRow[1]<= 2'b00;
             end else if (rOpenBankRow[2]!=2'b00) begin 
                 rWrDataCmd     <= rActiveDataCmd[2];
                 rWrCmd         <= 1'b1;
                 rOpenBankRow[2]<= 2'b00;
             end else if (rOpenBankRow[3]!=2'b00) begin 
                 rWrDataCmd     <= rActiveDataCmd[3];
                 rWrCmd         <= 1'b1;
                 rOpenBankRow[3]<= 2'b00;
             end
        end else begin 
             rWrCmd     <= 1'b0;
             rHitBtB    <= 1'b0;  //Not Hit Back to Back Read or Write
             rWrDataCmd <= 36'b0;
        end 
    end
end 

assign timerExpired = ~(|timer);

always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rRdCmd <= 1'b0;
    end else begin 
        if (~wEmptyCmd) begin 
            rRdCmd <= 1'b1;
        end else begin 
            rRdCmd <= 1'b0;
        end
    end 
end 
//Command control logic 
always @(*) begin 
    rOpCmd    = wRdDataCmd[`CMD_OP_RANGE] ;
    rBankCmd  = wRdDataCmd[`CMD_BANK_RANGE];
    rRowCmd   = wRdDataCmd[`CMD_ROW_RANGE];
    rColCmd   = wRdDataCmd[`CMD_COL_RANGE];
    rDAddrCmd = wRdDataCmd[`CMD_DADDR_RANGE]; 
    rDataCmd  = iQWD_RdData[`CMD_DATA_RANGE];
    rMaskCmd  = iQWD_RdData[`CMD_MASK_RANGE];
end 

reg   rDoRdP;
//timer controller
always @(posedge sclk or negedge sresetn) begin 
    if(~sresetn) begin 
        rDoWrP       <= 1'b0;
        rDoRdP       <= 1'b0;
        rFetchWrData <= 1'b0;
    end else begin 
       if (wRdValidCmd) begin 
           case (rOpCmd)
              CMD_WRA :  begin 
                             rDoWrP       <= 1'b1;
                             rFetchWrData <= 1'b1;
                         end 
              CMD_RDA :  begin
                             rDoRdP       <= 1'b1;
                             rFetchWrData <= 1'b0;
                         end
           endcase  
       end 
       if (rCmdDone) begin 
          rDoWrP <= 1'b0;
          rDoRdP <= 1'b0;
       end 
    end 
end 
reg       rTimerEn; 
reg [3:0] rTimerInit;
reg [3:0] rTimerCycle;
reg       ppRdValidCmd; 
reg       rDoRefresh;

//Auto Refresh timer logic
assign timerExpiredRefresh = ~(|timerRefresh);
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        timerRefresh <= tRFC;
        rDoRefresh   <= 1'b0;
    end else begin 
        timerRefresh <= timerRefresh - 16'b1;
        rDoRefresh   <= timerRefresh > tRC ? 1'b0 : 1'b1;
    end 
end
reg rStartTxn;
reg rStartCmd;
always @(posedge sclk or negedge sresetn) begin 
    if (~sresetn) begin 
        rCmd         <= CMD_NOP;
        rStartTxn    <= 1'b0;
        rStartCmd    <= 1'b0;
        rTimerEn     <= 1'b0;
        rTimerInit   <= 4'b0000;
        rTimerCycle  <= 4'b0;
        timer        <= 4'b0;
        ppRdValidCmd <= 1'b0;
        oQWD_Rd      <= 1'b0;
        oQWD_RdAddr  <= 5'b0;
        rCmdDone     <= 1'b0;
    end else begin 
        ppRdValidCmd <= wRdValidCmd;
        timer <= rTimerEn          ? (rTimerInit ) : 
                 (rDoWrP|rDoRdP)   ? (timer - 4'b1): 4'b1111; 
        if (rDoPreAllBank) begin 
             rStartTxn <=1'b0;  
             rCmd      <= CMD_PRE_ALLBANK;
        end else if (rDoLoadReg) begin 
             rStartTxn <=1'b0;  
             rCmd      <= CMD_LOAD_REG;
        end else if (~rDoPreAllBank && ~rDoLoadReg && ~rStartTxn) begin 
             rCmd      <= CMD_NOP;
             rStartTxn <= 1'b1;
             rStartCmd <= 1'b1;
        end else if ((rDoWrP|rDoRdP) & ~timerExpired &rStartCmd & rCmd == CMD_NOP) begin 
             rTimerEn    <= ppRdValidCmd; 
             rTimerCycle <= rTimerCycle + 4'b1;
             rCmd        <= CMD_ACT;
             rTimerInit  <= tRCD-1;
             rStartCmd   <= 1'b0;
        end  else if (rDoRdP & timerExpired & rCmd==CMD_ACT) begin 
             rCmd        <= CMD_RD;
             rTimerInit  <= tCAS-1;
             rTimerEn    <= timerExpired;
        end  else if (rDoWrP & timerExpired & rCmd==CMD_ACT) begin 
             rCmd        <= CMD_WR;
             rTimerInit  <= tWR-1;
             rTimerEn    <= timerExpired;
             oQWD_Rd     <= timerExpired;
             oQWD_RdAddr <= rColCmd[6:2];
        end  else if (rDoWrP & timerExpired & rCmd==CMD_WR) begin
             rCmd        <= CMD_NOP; 
             oQWD_Rd     <= 1'b0;
             rTimerInit  <= tCP-1; 
             rTimerEn    <= timerExpired;
        end  else if (rDoRdP & timerExpired & rCmd==CMD_RD) begin
             rCmd        <= CMD_DATA; 
             rTimerInit  <= tBL -1;
             //Add a signal to indicate to read the data
             rTimerEn    <= timerExpired;
        end  else if (rDoRdP & timerExpired & rCmd==CMD_DATA) begin
             rCmd        <= CMD_NOP; 
             rTimerInit  <= tCP-1; 
             rTimerEn    <= timerExpired;
        end  else if ((rDoWrP|rDoRdP) & timerExpired & rCmd==CMD_NOP) begin 
             rCmd        <= CMD_PRE_ONEBANK; 
             rTimerInit  <= tPR-1; 
             rTimerEn    <= timerExpired;
        end  else if ((rDoWrP|rDoRdP) & timerExpired & rCmd==CMD_PRE_ONEBANK) begin 
             rCmd        <= CMD_NOP; 
             rTimerInit  <= 1;
             rTimerEn    <= timerExpired;     
             rCmdDone    <= timerExpired;     
        end  else if (rDoRefresh) begin
             rCmd        <= CMD_SELF_REFRESH;
             rTimerInit  <= tCBR-1;
             rTimerEn    <= timerExpired; //? 
        end else begin 
             rTimerEn    <= 1'b0;
             rCmdDone    <= 1'b0;
             oQWD_Rd     <= 1'b0;
        end 
    end  
end 

always @(*) begin 
    case (rCmd)
        CMD_PRE_ALLBANK: precharge_all_bank(0, hi_z);
        CMD_PRE_ONEBANK: precharge_bank(rBankCmd, 0, hi_z);
        CMD_LOAD_REG   : load_mode_reg(rBurstMode[10:0]);
        CMD_NOP        : nop(0, hi_z);
        CMD_ACT        : active(rBankCmd, rRowCmd, hi_z);
        CMD_WR         : write(rBankCmd, rColCmd, rDataCmd, rMaskCmd );
    endcase
end 

sync_fifo#(.DW(35), .AW(10))  queueCmd (
  .clk(sclk),
  .reset_n(sresetn),
  .flush(1'b0),
  .wr(rWrCmd),
  .rd(rRdCmd),
  .wdata(rWrDataCmd),
  .rdata(wRdDataCmd),
  .rdata_valid(wRdValidCmd),
  .wfull(wFullCmd),
  .rempty(wEmptyCmd)

);


task precharge_all_bank;
    input  [3 : 0] dqm_in;
    input [31 : 0] dq_in;
    begin
        oClkEn= 1'b1;
        oCsn  = 1'b0;
        oRasn = 1'b0;
        oCasn = 1'b0;
        oWen  = 1'b0;
        oDqm  = dqm_in;
        oBank = 2'b0;
        oAddr = 1024;            // A10 = 1
        rDq   = dq_in;
    end
endtask 
task precharge_bank;
    input  [1 : 0] bank;
    input  [3 : 0] dqm_in;
    input [31 : 0] dq_in;
    begin
        oClkEn= 1'b1;
        oCsn  = 1'b0;
        oRasn = 1'b0;
        oCasn = 1'b0;
        oWen  = 1'b0;
        oDqm  = dqm_in;
        oBank = bank; 
        oAddr = 1024;            // A10 = 1
        rDq   = dq_in;
    end
endtask 


task nop;
    input  [3 : 0] dqm_in;
    input [31 : 0] dq_in;
    begin
        oClkEn = 1'b1;
        oCsn   = 1'b0;
        oRasn  = 1'b1;
        oCasn  = 1'b1;
        oWen   = 1'b1;
        oDqm   = dqm_in;
        //ba   = 0;
        //addr = 0;
        rDq    = dq_in;
    end
endtask
task active;
    input  [1 : 0] bank;
    input [10 : 0] row;
    input [31 : 0] dq_in;
    begin
        oClkEn   = 1'b1;
        oCsn     = 1'b0;
        oRasn    = 1'b0;
        oCasn    = 1'b1;
        oWen     = 1'b1;
        oDqm     = 1'b0;
        oBank    = bank;
        oAddr    = row;
        rDq      = dq_in;
    end
endtask
task write;
    input  [1 : 0] bank;
    input [10 : 0] column;
    input [31 : 0] dq_in;
    input  [3 : 0] dqm_in;
    begin
        oClkEn   = 1'b1;
        oCsn     = 1'b0;
        oRasn    = 1'b1;
        oCasn    = 1'b0;
        oWen     = 1'b0;
        oDqm     = dqm_in;
        oBank    = bank;
        oAddr    = column;
        rDq      = dq_in;
    end
endtask
task load_mode_reg;
    input [10 : 0] op_code;
    begin
        oClkEn   = 1'b1;
        oCsn     = 1'b0;
        oRasn    = 1'b0;
        oCasn    = 1'b0;
        oWen     = 1'b0;
        oDqm     = 1'b0;
        oBank    = 1'b0;
        oAddr    = op_code;
        rDq      = hi_z;
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
