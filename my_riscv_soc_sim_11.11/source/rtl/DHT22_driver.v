module DHT22_drive (
    input                clk   ,   //系统时钟
    input                res     ,   //系统复位                                    
    inout                dht22     ,   //dht22温湿度传感器单总线
    output  reg  [31:0]  data_out ,     //温度高位
    output               finish
);                                                                             
//parameter define 
parameter  POWER_ON_NUM     = 2000_000; //上电延时等待时间,单位us
//状态机各个状态                     
parameter  st_power_on_wait = 3'd0;     //上电延时等待 000
parameter  st_low_500us     = 3'd1;     //主机发送500us低电平 001
parameter  st_high_40us     = 3'd2;     //主机释放总线40us 010
parameter  st_rec_low_83us  = 3'd3;     //接收83us低电平响应 011
parameter  st_rec_high_87us = 3'd4;     //等待87us高电平（准备接收数据）100
parameter  st_rec_data      = 3'd5;     //接收40位数据 101
parameter  st_delay         = 3'd6;     //延时等待,延时完成后重新操作DHT22 110

//reg define
reg    [2:0]   	cur_state   	  ;        //当前状态 3位寄存器
reg    [2:0]   	next_state  	  ;        //下一个状态
								  			
reg    [4:0]   	clk_cnt     	  ;        //分频计数器
reg            	clk_1m      	  ;        //1Mhz时钟
reg    [20:0]  	us_cnt      	  ;        //1微秒计数器
reg            	us_cnt_clr  	  ;        //1微秒计数器清零信号
								  			
reg    [39:0]  	data_temp   	  ;        //缓存接收到的数据
reg            	step        	  ;        //数据采集状态
reg    [5:0]   	data_cnt    	  ;        //接收数据用计数器
								  
reg            	dht22_buffer	  ;        //DHT22输出信号
reg            	dht22_d0    	  ;        //DHT22输入信号寄存器0
reg            	dht22_d1    	  ;        //DHT22输入信号寄存器1
								  
//wire define  	           		       
wire        	dht22_pos      	  ;        //DHT22上升沿
wire        	dht22_neg      	  ;        //DHT22下降沿
reg          test_end          ;
//*****************************************************
//**                    main code
//*****************************************************

assign dht22     = dht22_buffer;
assign dht22_pos = ~dht22_d1 & dht22_d0; //采集上升沿
assign dht22_neg = dht22_d1 & ~dht22_d0; //采集下降沿

//得到1Mhz分频时钟
always @ (posedge clk or negedge res) begin   //posedge上升沿触发或者negedge下降沿触发
    if (!res) begin
        clk_cnt <= 5'd0;
        clk_1m  <= 1'b0;
    end 
    else if (clk_cnt < 5'd24) 
        clk_cnt <= clk_cnt + 1'b1;       
    else begin
        clk_cnt <= 5'd0;
        clk_1m  <= ~ clk_1m;
    end 
end

//对DHT22输入信号连续寄存两次，用于边沿检测
always @ (posedge clk_1m or negedge res) begin
    if (!res) begin
        dht22_d0 <= 1'b1;
        dht22_d1 <= 1'b1;
    end 
    else begin
        dht22_d0 <= dht22;
        dht22_d1 <= dht22_d0;
    end 
end 

//1微秒计数器
always @ (posedge clk_1m or negedge res) begin
    if (!res)
        us_cnt <= 21'd0;
    else if (us_cnt_clr)
        us_cnt <= 21'd0;
    else 
        us_cnt <= us_cnt + 1'b1;
end 

//状态跳转
always @ (posedge clk_1m or negedge res) begin
    if (!res)
        cur_state <= st_power_on_wait;
    else 
        cur_state <= next_state;
end 

//状态机读取DHT11数据
always @ (posedge clk_1m or negedge res) begin
    if(!res) begin
        next_state   <= st_power_on_wait;
        data_temp    <= 40'd0;
        step         <= 1'b0; 
        us_cnt_clr   <= 1'b0;
        data_cnt     <= 6'd0;
        dht22_buffer <= 1'bz;   
    end 
    else begin
        case (cur_state)
                //上电后延时2秒等待DHT22稳定
            st_power_on_wait : begin                
                if(us_cnt < POWER_ON_NUM) begin
                    dht22_buffer <= 1'bz; //空闲状态释放总线
                    us_cnt_clr   <= 1'b0;
					//data_out	 <= 40'h100; ///////////测试标志，上电两秒钟等待
                end
                else begin            
                    next_state   <= st_low_500us;
                    us_cnt_clr   <= 1'b1;
                end
            end
                //FPGA发送起始信号（1000us的低电平）    
            st_low_500us: begin
                if(us_cnt < 1000) begin
                    dht22_buffer <= 1'b0; //起始信号为低电平 
                    us_cnt_clr   <= 1'b0;
					//data_out	 <= 40'h200; ///////////测试标志，发送低电平
                end
                else begin
                    dht22_buffer <= 1'bz; //起始信号结束后释放总线
                    next_state   <= st_high_40us;
                    us_cnt_clr   <= 1'b1;
					//data_out	 <= 40'h300; ///////////测试标志，释放总线了（或许要拉高）
                end    
            end 
                //等待DHT22的响应信号（等待20~40us）
            st_high_40us:begin                      
                if (us_cnt < 40) begin
                    us_cnt_clr   <= 1'b0;
					//data_out	 <= 40'h400; ///////////测试标志，释放等待（或需要拉高）
                    if(dht22_neg) begin   //检测到DHT22响应信号
                        next_state <= st_rec_low_83us;
                        us_cnt_clr <= 1'b1; 
						//data_out	 <= 40'h500; ///////////测试标志，有响应了
                    end
                end
                else                      //超过40us未响应
                    next_state <= st_delay;
					//data_out	 <= 40'h9900; ///////////测试标志，没响应，被发配西伯利亚了- -
            end 
                //等待DHT22的83us低电平响应信号结束
            st_rec_low_83us: begin                  
                if(dht22_pos)         	//等待上升沿          
                    next_state <= st_rec_high_87us; 
					//data_out	 <= 40'h600; ///////////测试标志，等待上升沿					
            end 
                //DHT22拉高87us通知FPGA准备接收数据
            st_rec_high_87us: begin
                if(dht22_neg) begin       //准备时间结束    ，等待下降沿
                    next_state <= st_rec_data; 
                    us_cnt_clr <= 1'b1;
					//data_out	 <= 40'h700; ///////////测试标志，等待下降沿
                end
                else begin                //高电平准备接收数据
                    data_cnt  <= 6'd0;
                    data_temp <= 40'd0;
                    step  <= 1'b0;
                end
            end 
                //连续接收40位数据 
            st_rec_data: begin                                
                case(step)
                    0: begin              //接收数据低电平，等待上升沿
                        if(dht22_pos) begin 
                            step   <= 1'b1;
                            us_cnt_clr <= 1'b1;
                        end            
                        else              //等待数据低电平结束
                            us_cnt_clr <= 1'b0;
                    end
                    1: begin              //接收数据高电平
                        if(dht22_neg) begin 
                            data_cnt <= data_cnt + 1'b1;
                                          //判断接收数据为0/1
                            if(us_cnt < 60)
                                data_temp <= {data_temp[38:0],1'b0};
                            else                
                                data_temp <= {data_temp[38:0],1'b1};
                            step <= 1'b0;
                            us_cnt_clr <= 1'b1;
                        end 
                        else              //等待数据高电平结束
                            us_cnt_clr <= 1'b0;
                    end
                endcase
                
                if(data_cnt == 40) begin  //数据传输结束，验证校验位
                    next_state <= st_delay;
                    if(data_temp[7:0] == data_temp[39:32] + data_temp[31:24] 
                                         + data_temp[23:16] + data_temp[15:8])begin
                        data_out <= data_temp[39:8];   
                        test_end <= 1'b1;
                    end
                end
            end 
                //完成一次数据采集后延时2s
            st_delay:begin
                if(us_cnt < 2000_000)
                    us_cnt_clr <= 1'b0;
                else begin                //延时结束后重新发送起始信号
                    next_state <= st_low_500us;      
                    us_cnt_clr <= 1'b1;
                    test_end <= 1'b0;
                end
            end
            default : ;
        endcase
    end 
end

//-----------------上升沿检测模块，检测到采集结束时发送一个时钟的高电平终止信号---------------------------
reg end1,end2;
always@(posedge clk or negedge res) begin
	if(res == 1'b0) begin
	end1<=1'b0;
	end2<=1'b0;
	end
	else begin
	end1<=test_end;
	end2<=end1;
	end
end
assign finish = (~end1)&&(end2);
endmodule   