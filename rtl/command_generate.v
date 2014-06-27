//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.27.2014
//Description : command_generate module is to generate the sram command with read the data from ROB
module command_generate (
sclk,
sresetn,
iROB_Empty,
iROB_Full,
oROB_Rd,
iROB_RdData,
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
reg  [1:0]                 rActiveBankRow[0:3];
//Each bank has a current active Row.
reg  [`ROW_W-1:0]          rActiveRow[0:3];
reg                        rROB_Valid;
reg  [2:0]                 rCmd;
reg  [2:0]                 rPrevCmd;

parameter tPR = 3; //Precharging needs 3 cylces


parameter hi_z=32'bz;


parameter CMD_NOP         = 3'b000;
parameter CMD_PRE_ALLBANK = 3'b100;
parameter CMD_PRE_ONEBANK = 3'b101;

parameter CMDRD_IDLE = 2'b00;
parameter CMDRD_EN   = 2'b01;
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

always @(*) begin 
    nsCmdRd = sCmdRd;
    case (sCmdRd) 
        CMDRD_IDLE : begin 
                         if (~iROB_Empty)
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

always @(*) begin 


end

reg [3:0]  timer;

//Command control logic 
//timer controller
always @(posedge sclk or negedge sresetn) begin 
    if(~sresetn) begin 
       rCmd     <= CMD_PRE_ALLBANK;
       rPrevCmd <= CMD_PRE_ALLBANK;
       timer    <= tPR;
    end else begin 
       if (timer == 4'b0 && rPrevCmd == CMD_PRE_ALLBANK) begin 
           rCmd <= CMD_NOP;
       end  
       timer <= (timer == 4'b0) ? 4'b0 : (timer - 4'b1);
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

        endcase

    end 


end 
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

endmodule 
