`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/05/03 16:06:56
// Design Name: 
// Module Name: HEX8
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module HEX8(clk,rst_n,en,disp_data,sel,seg);
 input clk;     
 input rst_n;    
 input en;     
 input [63:0]disp_data; 
 
 output [7:0]sel;   
 output reg[7:0]seg;  
 
 reg [15:0]divider_cnt;
 reg clk_1k;
 reg [7:0]sel_r;


 always@(posedge clk,negedge rst_n)
  if(!rst_n)
   divider_cnt <= 15'd0;
  else if(!en)
   divider_cnt <= 15'd0;
  else if(divider_cnt == 24999)
   divider_cnt <= 15'd0;
  else
   divider_cnt <= divider_cnt + 1'b1;


 always@(posedge clk,negedge rst_n)
  if(!rst_n)
   clk_1k <= 1'b0;
  else if(!en)
   clk_1k <= 1'b0;
  else if(divider_cnt == 24999)
   clk_1k <= ~clk_1k;
  else 
   clk_1k <= clk_1k;


 always@(posedge clk_1k,negedge rst_n)
  if(!rst_n)
   sel_r <= 8'b0000_0001;
  else if(sel_r == 8'b1000_0000)
   sel_r <= 8'b0000_0001;
  else
   sel_r <= sel_r << 1;


 always@(*)
  case(sel_r)
   8'b0000_0001: seg =disp_data[7:0];
   8'b0000_0010: seg =disp_data[15:8];
   8'b0000_0100: seg =disp_data[23:16];
   8'b0000_1000: seg =disp_data[31:24];
   8'b0001_0000: seg =disp_data[39:32];
   8'b0010_0000: seg =disp_data[47:40];
   8'b0100_0000: seg =disp_data[55:48];
   8'b1000_0000: seg = disp_data[63:56];
   default:   seg = 8'b1111_1111;
  endcase
  

 assign sel = en?sel_r:8'b0000_0000;
endmodule
