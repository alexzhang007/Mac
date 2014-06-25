//Author     : Alex Zhang (cgzhangwei@gmail.com)
//Date       : 2013-12-17 
//Description: Codes from Cummings SNUG 2002 San Joe. 
//             Jun.14.2014
//             Update the full and empty output signal, need one cycle only. 
module async_fifo(
wclk, 
wrst_n,
wr,
rclk,
rrst_n,
rd,
wdata,
rdata,
wfull,
rempty
);
parameter DW=8, AW=4;

input wclk; 
input wrst_n;
input wr;
input rd;
input rclk;
input rrst_n;
input wdata;
output rdata;
output wfull;
output rempty;

wire [DW-1:0] rdata;
wire [DW-1:0] wdata;
//Internal variable 
wire [AW-1:0] wptr, rptr;
wire [AW-1:0] waddr, raddr;
wire             aempty_n;
wire             afull_n;

async_cmp #(AW)  
    async_cmp (
    .aempty_n(aempty_n),
    .afull_n(afull_n),
    .wptr(wptr),
    .rptr(rptr),
    .wrst_n(wrst_n)
    );

fifomem #(DW, AW) 
    fifomem (
    .rdata(rdata),
    .wdata(wdata),
    .waddr(wptr),
    .raddr(rptr),
    .wr_en(wr),
    .wclk(wclk)
    );

rptr_empty #(AW) 
    rptr_empty(
    .rempty(rempty),
    .rptr(rptr),
    .aempty_n(aempty_n),
    .rd_en(rd),
    .rclk(rclk),
    .rrst_n(rrst_n)
    );

wptr_full #(AW)
    wptr_full(
    .wfull(wfull),
    .wptr(wptr),
    .afull_n(afull_n),
    .wr_en(wr),
    .wclk(wclk),
    .wrst_n(wrst_n)
    );
endmodule 

module async_cmp(
aempty_n,
afull_n,
wptr,
rptr,
wrst_n
);
parameter AW=4;
parameter N = AW-1;

output aempty_n;
output afull_n;
input [N:0] wptr;
input [N:0] rptr;
input wrst_n;

reg direction;
wire high = 1'b1;
wire dir_set_n;
wire dir_clr_n;

assign dir_set_n = ~((  wptr[N]^rptr[N-1]) & ~(wptr[N-1]^rptr[N]));
assign dir_clr_n = ~((~(wptr[N]^rptr[N-1]) &  (wptr[N-1]^rptr[N]))| ~wrst_n);

always @(posedge high or negedge dir_set_n or negedge dir_clr_n)
    if (!dir_clr_n)      direction <=1'b0;
    else if (!dir_set_n) direction <=1'b1;
    else                 direction <=high;

assign aempty_n = ~( (wptr== rptr) && !direction );
assign afull_n  = ~( (wptr== rptr) &&  direction );

endmodule 

module fifomem (
wclk,
wr_en,
waddr,
wdata,
raddr,
rdata
);
parameter DW=8, AW=4;
parameter DEPTH = 1<< AW;

input wclk;
input wr_en;
input waddr;
input wdata;
input raddr;
output rdata;

wire [DW-1:0] wdata;
wire [DW-1:0] rdata;
wire [AW-1:0] waddr;
wire [AW-1:0] raddr;

`ifdef VENDOR_MEM
    VENDOR_RAM MEM (
    .dout(rdata),
    .din(wdata),
    .waddr(waddr),
    .raddr(raddr),
    .wclken(wr_en),
    .clk(wclk)
    )
`else 
    reg [DW-1:0] MEM[0: DEPTH-1];

    assign rdata = MEM[raddr];
    always @(posedge wclk)
        if (wr_en) MEM[waddr] <= wdata;
`endif
endmodule 
  
module rptr_empty(
rclk,
rrst_n,
rd_en,
aempty_n,
rempty,
rptr
);
input rclk;
input rrst_n;
input rd_en;
input aempty_n;
output rempty;
output rptr;
parameter AW = 4;
reg [AW-1:0] rptr;

//internal variable
reg[AW-1:0] rbin, rgnext, rbnext;
reg rempty, rempty2;
 
//Gray Style2 Pointer 
always @(posedge rclk or negedge rrst_n)
    if (~rrst_n) begin 
        rbin <= 0;
        rptr <= 0;
    end else begin
        rbin <= rbnext;
        rptr <= rgnext;
    end 
//increment the binary count if not empty
assign rbnext = !rempty ? rbin + rd_en : rbin ;
assign rgnext = (rbnext>>1)^rbnext;

always @(posedge rclk or negedge aempty_n)
    if (!aempty_n) {rempty, rempty2} <= 2'b11;
    else           {rempty, rempty2} <= {rempty, ~aempty_n};

endmodule //endmodule rptr_empty  

module wptr_full (
wclk,
wrst_n,
wr_en,
wptr,
afull_n,
wfull
);
input wclk;
input wrst_n;
input wr_en;
output wptr;
input afull_n;
output wfull;

parameter AW =4;
reg [AW-1:0] wptr;

//internal variables
reg [AW-1:0] wbin;
wire [AW-1:0] wgnext, wbnext;
reg wfull, wfull2;

//Gray Style2 Pointer
always @(posedge wclk or negedge wrst_n)
    if (~wrst_n) begin 
        wbin <= 0 ;
        wptr <= 0;
    end  else begin 
        wbin <= wbnext;
        wptr <= wgnext;
    end 
 
//increment the binary count if not full
assign wbnext = !wfull ? wbin +wr_en : wbin;
assign wgnext = (wbnext>>1) ^ wbnext;

always @(posedge wclk or negedge wrst_n or negedge afull_n)
    if (~wrst_n) {wfull, wfull2} <= 2'b00;
    else if (~afull_n) {wfull, wfull2} <= 2'b11;
    else {wfull2, wfull} <= {wfull, ~afull_n}; //We need one cycle delay

endmodule //endmodule wptr_full
