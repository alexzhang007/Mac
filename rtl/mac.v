//Author      : Alex Zhang (cgzhangwei@gmail.com)
//Date        : Jun.17.2014
//Description : Implement the memory_access_controller (MAC)

module memory_access_controller(
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
reg          oMAC_ReadyRd;
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
reg          oMAC_ReadyWr;
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




endmodule 
