//Author     : Alex Zhang (cgzhangwei@gmail.com)
//Date       : 2014-06-24
//Description: Synchronize fifo
module sync_fifo
( 
clk,                     //clock input
reset_n,                 //reset
flush,                   //flush
wdata,                   //write data
wr,                      //write valid data
rd,                      //read request
rdata,                   //read data
rdata_valid,             //read data valid
rempty,                  //fifo empty flag
fifo_aempty,             //fifo almost empty flag
fifo_full,               //fifo full flag
wfull,                   //fifo almost empty flag  
write_ack                //write acknowlegment
);
parameter AW=4;
parameter DW=16;
parameter depth=1<<AW;
parameter aempty=3;
parameter afull=3;
//INPUTS//
input clk;                           //clock input
input reset_n;                       //reset
input flush;                         //flush
input wdata;                         //write data
input wr;                            //write valid data
input rd;                            //read request
//OUTPUTS//
output rdata;                        //read data
output rdata_valid;                  //read data valid
output rempty;                       //fifo empty flag
output fifo_aempty;                  //fifoalmostempty flag
output fifo_full;                    //fifo full flag
output wfull;                        //fifoalmostempty flag  
output write_ack;                    //write acknowlegment

wire [DW-1:0] wdata;   //write data
wire [DW-1:0] rdata;    //read data
//Internal wires//
wire [AW:0]   read_ptr;       //read pointer
wire [AW:0]   write_ptr;      //write pointer
wire          rd_en;          //read enable
wire          wr_en;          //write enable


write_control_logic #(.AW(AW), .afull(afull)) write_cntl(
  .read_ptr(read_ptr),
  .flush(flush),
  .reset_n(reset_n),
  .clk(clk),
  .wdata_valid(wr),
  .write_ack(write_ack),
  .wr_en(wr_en),
  .write_ptr(write_ptr),
  .fifo_full(fifo_full),
  .fifo_afull(wfull)
);

read_control_logic #(.AW(AW), .aempty(aempty)) read_cntl(
  .write_ptr(write_ptr),
  .clk(clk),
  .reset_n(reset_n),
  .flush(flush),
  .read_req(rd),
  .rd_en(rd_en),
  .rdata_valid(rdata_valid),
  .fifo_empty(rempty),
  .read_ptr(read_ptr),
  .fifo_aempty(fifo_aempty)
);

memory_array #(.AW(AW), .DW(DW)) mem_arr 
(
  .write_addr(write_ptr[AW-1:0]),
  .read_addr(read_ptr[AW-1:0]),
  .wr_en(wr_en),
  .rd_en(rd_en),
  .clk(clk),
  .write_data(wdata),
  .read_data(rdata)
);
endmodule

//write control logic//
module write_control_logic
(
// Inputs//
clk,
reset_n,
flush,
wdata_valid,
read_ptr,

// Outputs//
write_ack,
wr_en,
write_ptr,
fifo_full,
fifo_afull
);
parameter AW=4;
parameter afull=3;
parameter depth=1<<AW;
input clk;
input reset_n;
input flush;
input wdata_valid;
input read_ptr;

// Outputs//
output write_ack;
output wr_en;
output write_ptr;
output fifo_full;
output fifo_afull;


wire [AW:0] read_ptr;
reg  [AW:0] write_ptr;
reg         fifo_afull;
reg         write_ack;
//Internal wires//
wire [AW-1:0] write_addr;
wire [AW-1:0] read_addr;  

//Read and Write addresses from their pointers//
assign read_addr= read_ptr[AW-1:0];
assign write_addr= write_ptr[AW-1:0];

//When fifo is full there will be no write//
assign wr_en= wdata_valid&&(~fifo_full);

//Logic for fifo full status//
//Fifo full is asserted when both pointers are at same address but msb are different//
assign fifo_full=((write_addr==read_addr)&&(write_ptr[AW]^read_ptr[AW]));

//Logic for almost full status//
always @(*) begin
    if(write_ptr[AW]==read_ptr[AW])
        fifo_afull=((write_addr-read_addr)>=(depth- afull));
    else
        fifo_afull=((read_addr-write_addr)<=afull);
end

//Logic for Write pointer//
always @(posedge clk or negedge reset_n) begin
    if (~reset_n)  begin
        write_ptr<={(AW+1){1'b0}};
        write_ack<=1'b0;
    end  else if(flush)  begin
        write_ptr<={(AW+1){1'b0}};
        write_ack<=1'b0;
    end  else if(wr_en)  begin
        write_ptr<=write_ptr+{{AW{1'b0}},1'b1};
        write_ack<=1'b1;
    end  else  begin
        write_ack<=1'b0;
    end
end
endmodule

//Read control logic//
module read_control_logic 
(
//Inputs//
clk,
reset_n,
flush,
read_req,
write_ptr,

//Outputs//
rd_en,
rdata_valid,
fifo_empty,
fifo_aempty,
read_ptr
);

parameter AW=4;
parameter aempty=3;
parameter depth=1<<AW;

input clk;
input reset_n;
input flush;
input read_req;
input write_ptr;

//Outputs//
output rd_en;
output rdata_valid;
output fifo_empty;
output fifo_aempty;
output read_ptr;

wire [AW:0] write_ptr;
reg                 rdata_valid;
reg                 fifo_aempty;
reg [AW:0]  read_ptr;
//Internal wires//
wire [AW-1:0] read_addr; 
wire [AW-1:0] write_addr;


//Read and Write address from their pointers//
assign read_addr=read_ptr[AW-1:0];
assign write_addr=write_ptr[AW-1:0];

//FIFO is empty when read pointer is equal to write pointer//
assign fifo_empty=(read_ptr==write_ptr);

//When FIFO is empty there will be no read//
assign rd_en=read_req&&(~fifo_empty);

// Logic for almost empty flag//
always @(*) begin
    if (read_ptr[AW]==write_ptr[AW])
        fifo_aempty=((write_addr-read_addr)<= aempty);
    else
        fifo_aempty=((read_addr-write_addr)>=(depth - aempty));
    end

//Logic for Read pointer and read valid data//
always @(posedge clk or negedge reset_n) begin
    if (~reset_n)
        read_ptr<={(AW+1){1'b0}};
    else if (flush)
        read_ptr<={(AW+1){1'b0}};
    else if (rd_en) begin
        read_ptr<=read_ptr+{{AW {1'b0}},1'b1};
        rdata_valid<= 1'b1;
    end else
        rdata_valid<= 1'b0;
end
endmodule

//Memory array//
module memory_array(
//Inputs//
write_addr,
read_addr,
wr_en,
rd_en,
clk,
write_data,
//Output//
read_data
);
parameter AW=4;
parameter depth=1<<AW;
parameter DW=16;
input write_addr;
input read_addr;
input wr_en;
input rd_en;
input clk;
input write_data;
//Output//
output read_data;

wire  [AW-1:0] write_addr;
wire  [AW-1:0] read_addr;
reg   [DW-1:0] read_data;
wire  [DW-1:0] write_data;

integer k;
//Register//
reg [DW-1:0] memory [0:depth-1];
initial begin 
    for (k=0; k<depth; k=k+1) begin 
        memory[k]=0;
    end 
end 

//Write into memory(data-in)//
always@(posedge clk) begin
    if(wr_en)
        memory[write_addr]<= write_data;
end

//Read from memory(data out)//
always@(posedge clk) begin
    if(rd_en) begin
        read_data<= memory[read_addr];
    end
end
endmodule




 



