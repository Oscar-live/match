module top_dht22(   
    input         clk,
    input         rst,
	//温湿度传感信号
    inout         dht22,              //
	output[31:0]  tem_hum,            //温湿度数据输出
	//8位数码管信号
    output    wire    SCLK,                //shcp
    output    wire    RCLK,                //stcp
    output    wire    DIO,
	//串口发送数据
	output		  tx,
	//测试
	output        led  
 );

wire    [31:0]    data_temp;
reg led_reg;
wire led_test;
//test led
wire hdt22end;
wire tx_end;
wire  [63:0] date_led ;

DHT22_drive     DHT22_drive_inst(
    .clk          (clk                   ),   //
    .res          (rst                   ),   //            
    .dht22        (dht22                 ),   //
    .data_out     (data_temp             ),   //
	.finish		  (hdt22end				 )
); 
assign tem_hum = data_temp;
//-----------------测试温湿度传感器原始数据---------------//

assign led = led_reg;
always@(posedge clk or negedge rst) begin
	if(!rst) begin
		led_reg <= 1'd1;
	end
	else if(led_test)begin
		led_reg <= 1'd0;
	end
    else
        led_reg <= 1'd1;
end

assign led_test = (data_temp == 32'h00_00_07_00)? 1'b1:1'b0;

reg [3:0] cnt_1;
reg       star;
reg [7:0] data_uart;

//--------------将采集标志延时1秒，使得采集数据稳定------------------//
//wire hdt22end_1s_flag;
//reg flag;
//reg tx_flag;
//reg [22:0] cnt_1s;
//always@(posedge clk or negedge rst)
//	if(rst == 1'b0)
//		cnt_1s <= 23'd0;
//	else if(cnt_1s == 23'd4999999)
//		cnt_1s <= 23'd0;
//	else
//		cnt_1s <= cnt_1s + 1'b1;
//  
//always@(posedge clk or negedge rst)
//	if(rst == 1'b0) begin
//		flag <= 1'b0;
//		tx_flag <= 1'b0;
//	end
//	else if(flag == 1'b1 && hdt22end_1s_flag == 1'b1) begin
//		flag <= 1'b0;
//		tx_flag <= 1'b1;
//	end
//	else if(hdt22end == 1'b1)begin
//		flag <= 1'b1;
//	end
//	else
//		tx_flag <= 1'b0;
//
//assign hdt22end_1ms_flag = (cnt_1s == 23'd4999999)? 1'b1 : 1'b0;


//------------------串口发送cnt-----------------------//
always@(posedge clk or negedge rst)
    if(rst == 1'b0)
        star <= 1'b0;
    else if(hdt22end == 1'b1)
        star <= 1'b1;
    else if(cnt_1 == 4'd5)
        star <= 1'b0;
		
always@(posedge clk or negedge rst)
	if(rst == 1'b0)
		cnt_1 <= 4'd0;
    else if(hdt22end == 1'b1)
        cnt_1 <= 4'd1;
	else if( tx_end == 1'b1 )
		cnt_1 <= cnt_1 + 1'b1;
	else if(cnt_1 == 4'd5)
		cnt_1 <= 4'd0;


always@(posedge clk or negedge rst)
    if(rst == 1'b0)
        data_uart  <= 8'b0;
    else 
        case(cnt_1)
            4'd1   :data_uart <= data_temp[31:24];
            4'd2   :data_uart <= data_temp[23:16];
            4'd3   :data_uart <= data_temp[15:8 ];
            4'd4   :data_uart <= data_temp[7:0 ];
            default :data_uart <= 8'd0;
        endcase




//-----------------蓝牙串口发送-------------------------//
hc05_top		hc05_top_inst( 

    .clk 		  (clk					 ),
    .rst 		  (rst					 ), 
//---------------------------------------
    .data_i 	  (data_uart 			 ),    //
    .en_flag_i    (star					 ),    //

    .data_o 	  (						 ),    //
    .en_flag_o    (						 ),    //

//---------------------------------
    .rx 		  (						 ),    //
    .tx			  (tx					 ),    //
	.tx_end	      (tx_end				 )     //

 );

//-----------------bcd 2进制转8421码-------------------------//
wire [3:0] one, two,three,four,five,six,seven,eight;
bcd_8421 bcd_8421_inst0
(
    .sys_clk    (clk                ) ,   //
    .sys_rst_n  (rst                ) ,   //
    .data       (data_temp[15:0]    ) ,   //
    .unit       (one                ) ,   //
    .ten        (two                ) ,   //
    .hun        (three              ) ,   //
    .tho        (four               )     //
);

bcd_8421 bcd_8421_inst1
(
    .sys_clk    (clk                ) ,   //
    .sys_rst_n  (rst                ) ,   //
    .data       (data_temp[31:16]   ) ,   //
	
    .unit       (five               ) ,   //
    .ten        (six                ) ,   //
    .hun        (seven              ) ,   //
    .tho        (eight              )     //
);

wire [31:0] data_8421;
assign data_8421 = {eight,seven,six,five,four,three,two,one};

//-----------------8421转码 用于数码管显示-----------------------//
data_conver data_conver_inst(
.data_in            (data_8421),
.dot_disp           (         ),
.data_out           (date_led )
);




wire [7:0] sel,seg;
HEX8    HEX8_inst(
    .clk         (clk                         ),
    .rst_n       (rst                         ),
    .en          (1'b1                        ),
    .disp_data   ({8'b1111_1111,date_led[55:32],8'b1111_1111,date_led[23:0]}),
    .sel         (sel                         ),
    .seg         (seg                         ) 
);

//-----------------数码管驱动-------------------------//
OLED_drive    OLED_drive_inst(
    .Clk          (clk                    ),
    .Rst_n        (rst                    ),
	.Data         ({seg[7:0],sel[7:0]}    ),
    .S_EN         (1'b1                   ),
    .SH_CP        (SCLK                   ),
    .ST_CP        (RCLK                   ),
    .DS           (DIO                     )
    );                     

endmodule