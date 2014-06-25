//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun. 16. 2014
//Description : Create the testbench
`timescale 100ps/1ps
module test;
reg         clk;
reg         resetn;
reg         clk333;

event start_sim_evt;
event end_sim_evt;
reg          rMAC_ValidRd;
reg  [31:0]  rMAC_AddrRd;
reg  [3:0]   rMAC_TagRd;
reg  [2:0]   rMAC_IdRd;
reg  [1:0]   rMAC_LenRd;
reg  [3:0]   rMAC_QoSRd;
wire         wMAC_ReadyRd;
wire         wMAC_ValidRsp;
wire [3:0]   wMAC_TagRsp;
wire [31:0]  wMAC_DataRsp;
wire [1:0]   wMAC_StatusRsp;
wire         wMAC_EoD;
reg          rMAC_ReadyRsp;
reg          rMAC_ValidWr;
reg  [31:0]  rMAC_AddrWr;
reg  [3:0]   rMAC_TagWr;
reg  [2:0]   rMAC_IdWr;
reg  [1:0]   rMAC_LenWr;
reg  [3:0]   rMAC_QoSWr;
wire         wMAC_ReadyWr;
reg  [31:0]  rMAC_DataWr;
reg  [3:0]   rMAC_MaskWr;
reg          rMAC_EoD;

mem_access_controller  Mac(
  .clk(clk),
  .resetn(resetn),
  .sclk(clk333),
  .sresetn(resetn),
  //Instruction Fetech (MAC)
  .iMAC_ValidRd(rMAC_ValidRd),
  .iMAC_AddrRd(rMAC_AddrRd),
  .iMAC_TagRd(rMAC_TagRd),
  .iMAC_IdRd(rMAC_IdRd),
  .iMAC_LenRd(rMAC_LenRd),
  .iMAC_QoSRd(rMAC_QoSRd),
  .oMAC_ReadyRd(wMAC_ReadyRd),
  .oMAC_ValidRsp(wMAC_ValidRsp),
  .oMAC_TagRsp(wMAC_TagRsp),
  .oMAC_DataRsp(wMAC_DataRsp),
  .oMAC_StatusRsp(wMAC_StatusRsp),
  .oMAC_EoD(wMAC_EoD),
  .iMAC_ReadyRsp(rMAC_ReadyRsp),
  .iMAC_ValidWr(rMAC_ValidWr),
  .iMAC_AddrWr(rMAC_AddrWr),
  .iMAC_TagWr(rMAC_TagWr),
  .iMAC_IdWr(rMAC_IdWr),
  .iMAC_LenWr(rMAC_LenWr),
  .iMAC_QoSWr(rMAC_QoSWr),
  .oMAC_ReadyWr(wMAC_ReadyWr),
  .iMAC_DataWr(rMAC_DataWr),
  .iMAC_MaskWr(rMAC_MaskWr),
  .iMAC_EoD(rMAC_EoD),
  .ioDq(),
  .oAddr(),
  .oBank(),
  .oCsn(),
  .oRasn(),
  .oWen(),
  .oDqm()
);



initial begin 
    basic;
end 
initial begin 
    $fsdbDumpfile("./out/mac.fsdb");
    $fsdbDumpvars(0, test);
end 

task basic ;
    begin 
        $display("Start MAC IP testing.");
        #1;
        fork
            drive_clock;
            drive_sram_clock;
            reset_unit;
            drive_sim;
            monitor_sim;
        join 
    end 
endtask 
task monitor_sim;
   begin 
   @(end_sim_evt);
   #10;
   $display("Test End");
   $finish;
   end 
endtask
task reset_unit;
    begin 
        #5;
        resetn = 1;
        #10;
        resetn = 0;
//Reset the reg variable 
        rMAC_ValidRd   = 1'b0;;
        rMAC_AddrRd    = 32'b0;
        rMAC_TagRd     = 4'b0;;
        rMAC_IdRd      = 3'b0;
        rMAC_LenRd     = 2'b0;
        rMAC_QoSRd     = 4'b0;
        rMAC_ReadyRsp  = 1'b0;
        rMAC_ValidWr   = 1'b0;
        rMAC_AddrWr    = 32'b0;
        rMAC_TagWr     = 4'b0;
        rMAC_IdWr      = 3'b0;
        rMAC_LenWr     = 2'b0;
        rMAC_QoSWr     = 4'b0;
        rMAC_DataWr    = 32'b0;
        rMAC_MaskWr    = 4'b0;
        rMAC_EoD       = 1'b0;

        #20;
        resetn = 1;
        ->start_sim_evt;
        $display("Reset is done");
        end
endtask 
//Clock frequency is .
task  drive_clock; 
    begin 
        clk = 0;
        forever begin 
        #0.5ns clk = ~clk;
        end 
    end 
endtask
task  drive_sram_clock;
    begin 
        clk333 = 0;
        forever begin 
        #1.5ns clk333 = ~clk333;
        end 
    end 
endtask
task  drive_sim;
    @(start_sim_evt);
   
    @(posedge clk);
    fork 
        begin  : drive_wrreq
            @(posedge clk);
            rMAC_ValidWr   = 1'b1;
            rMAC_AddrWr    = 32'h2345_F220;
            rMAC_TagWr     = 4'b0;
            rMAC_IdWr      = 3'b101;
            rMAC_LenWr     = 2'b10; //64Bit data
            rMAC_QoSWr     = 4'b0110;
            @(posedge clk);
            rMAC_ValidWr   = 1'b0;
            rMAC_AddrWr    = 32'h0;
            rMAC_TagWr     = 4'b0;
            rMAC_IdWr      = 3'b0;
            rMAC_LenWr     = 2'b0; //64Bit data
            rMAC_QoSWr     = 4'b0;
        end 
        begin :drive_wrdata 
	    $display("%0d - Waiting wMAC_ReadyWr", $time);
            @(posedge clk);
            wait ( wMAC_ReadyWr == 1'b1);
            //@(posedge clk);
            @(posedge clk);
            rMAC_DataWr    = 32'hABCD_EF12;
            rMAC_MaskWr    = 4'b1101;
            rMAC_EoD       = 1'b0;
            @(posedge clk);
            rMAC_DataWr    = 32'hCBCD_EF12;
            rMAC_MaskWr    = 4'b1011;
            rMAC_EoD       = 1'b1;
            @(posedge clk);
            rMAC_DataWr    = 32'h0;
            rMAC_MaskWr    = 4'b0;
            rMAC_EoD       = 1'b0;
        end
    join
    repeat (100) @(posedge clk);

    ->end_sim_evt;
endtask 

endmodule 
