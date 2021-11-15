`timescale 1ns / 1ns
module soc_tb;

reg clk;
reg rst;
wire [9:0] out;
initial begin
clk = 1'b0;
rst = 1'b0;
#10
rst = 1'b1;
end

always #10 clk = ~clk;

tinyriscv_soc_top tinyriscv_soc_top_inst(

    .clk         ( clk           )    ,
    .rst         ( rst           )    ,
    .gpio        ( out           )    // GPIOÒý½Å
    );
endmodule