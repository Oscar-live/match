 /*                                                                      
	Created by wangsijia 2021/10/14                                        
 */
 
//`include "../core/defines.v"
 `define CpuResetAddr 32'h0

`define RstEnable 1'b0
`define RstDisable 1'b1
`define ZeroWord 32'h0
`define ZeroReg 5'h0
`define WriteEnable 1'b1
`define WriteDisable 1'b0
`define ReadEnable 1'b1
`define ReadDisable 1'b0
`define True 1'b1
`define False 1'b0
`define ChipEnable 1'b1
`define ChipDisable 1'b0
`define JumpEnable 1'b1
`define JumpDisable 1'b0
`define DivResultNotReady 1'b0
`define DivResultReady 1'b1
`define DivStart 1'b1
`define DivStop 1'b0
`define HoldEnable 1'b1
`define HoldDisable 1'b0
`define Stop 1'b1
`define NoStop 1'b0
`define RIB_ACK 1'b1
`define RIB_NACK 1'b0
`define RIB_REQ 1'b1
`define RIB_NREQ 1'b0
`define INT_ASSERT 1'b1
`define INT_DEASSERT 1'b0

`define INT_BUS 7:0
`define INT_NONE 8'h0
`define INT_RET 8'hff
`define INT_TIMER0 8'b00000001
`define INT_TIMER0_ENTRY_ADDR 32'h4

`define Hold_Flag_Bus   2:0
`define Hold_None 3'b000
`define Hold_Pc   3'b001
`define Hold_If   3'b010
`define Hold_Id   3'b011

// I type inst
`define INST_TYPE_I 7'b0010011
`define INST_ADDI   3'b000
`define INST_SLTI   3'b010
`define INST_SLTIU  3'b011
`define INST_XORI   3'b100
`define INST_ORI    3'b110
`define INST_ANDI   3'b111
`define INST_SLLI   3'b001
`define INST_SRI    3'b101

// L type inst
`define INST_TYPE_L 7'b0000011
`define INST_LB     3'b000
`define INST_LH     3'b001
`define INST_LW     3'b010
`define INST_LBU    3'b100
`define INST_LHU    3'b101

// S type inst
`define INST_TYPE_S 7'b0100011
`define INST_SB     3'b000
`define INST_SH     3'b001
`define INST_SW     3'b010

// R and M type inst
`define INST_TYPE_R_M 7'b0110011
// R type inst
`define INST_ADD_SUB 3'b000
`define INST_SLL    3'b001
`define INST_SLT    3'b010
`define INST_SLTU   3'b011
`define INST_XOR    3'b100
`define INST_SR     3'b101
`define INST_OR     3'b110
`define INST_AND    3'b111
// M type inst
`define INST_MUL    3'b000
`define INST_MULH   3'b001
`define INST_MULHSU 3'b010
`define INST_MULHU  3'b011
`define INST_DIV    3'b100
`define INST_DIVU   3'b101
`define INST_REM    3'b110
`define INST_REMU   3'b111

// J type inst
`define INST_JAL    7'b1101111
`define INST_JALR   7'b1100111

`define INST_LUI    7'b0110111
`define INST_AUIPC  7'b0010111
`define INST_NOP    32'h00000001
`define INST_NOP_OP 7'b0000001
`define INST_MRET   32'h30200073
`define INST_RET    32'h00008067

`define INST_FENCE  7'b0001111
`define INST_ECALL  32'h73
`define INST_EBREAK 32'h00100073

// J type inst
`define INST_TYPE_B 7'b1100011
`define INST_BEQ    3'b000
`define INST_BNE    3'b001
`define INST_BLT    3'b100
`define INST_BGE    3'b101
`define INST_BLTU   3'b110
`define INST_BGEU   3'b111

// CSR inst
`define INST_CSR    7'b1110011
`define INST_CSRRW  3'b001
`define INST_CSRRS  3'b010
`define INST_CSRRC  3'b011
`define INST_CSRRWI 3'b101
`define INST_CSRRSI 3'b110
`define INST_CSRRCI 3'b111

// CSR reg addr
`define CSR_CYCLE   12'hc00
`define CSR_CYCLEH  12'hc80
`define CSR_MTVEC   12'h305
`define CSR_MCAUSE  12'h342
`define CSR_MEPC    12'h341
`define CSR_MIE     12'h304
`define CSR_MSTATUS 12'h300
`define CSR_MSCRATCH 12'h340

`define RomNum 4096  // rom depth(how many words)

`define MemNum 4096  // memory depth(how many words)
`define MemBus 31:0
`define MemAddrBus 31:0

`define InstBus 31:0
`define InstAddrBus 31:0

// common regs
`define RegAddrBus 4:0
`define RegBus 31:0
`define DoubleRegBus 63:0
`define RegWidth 32
`define RegNum 32        // reg num
`define RegNumLog2 5
module pwm
    #(
    	parameter CNT_1US_MAX = 6'd49   ,
    	parameter CNT_1MS_MAX = 10'd999 
    )
    (
	input wire clk,
	input wire rst,
	
	input wire[31:0]	data_duty_i,//pwm 占空比 
	input wire[31:0]	addr_i     ,
	input wire			we_i	   ,
	//温湿度传感器信号
    input wire[31:0]    tem_hum_i  ,//温湿度传感器数据
	output wire         hum_flag_o ,//LED信号测试输出
	//光电传感器
	input wire			light_sense,//光电传感
	
	
	output reg[31:0]    data_o,     //相关寄存器数据输出
	output reg       	pwm_out_1, 	//pwm reg_1输出
	output reg       	pwm_out_2, 	//pwm reg_2输出
	output reg       	pwm_out_3, 	//pwm reg_3输出
	output wire			led_out_io  //pwm wire输出
	);
	
	//寄存器地址
	localparam REG_DUTY_1 = 4'h0;
	localparam REG_DUTY_2 = 4'h4;
	localparam REG_DUTY_3 = 4'h8;
	localparam REG_HUM_FLAG = 4'hc;
	//寄存器定义		
	reg [5:0]   cnt_1us     ;
	reg [9:0]   cnt_1ms     ;
	reg [9:0]   duty_r_1    ; //占空比寄存器
	reg [9:0]   duty_r_2    ; 
	reg [9:0]   duty_r_3    ; 
	reg [9:0]	hum_flag	; //加湿器工作的标志寄存器
	
	//写寄存器
	always@(posedge clk)
	begin
		if(rst == `RstEnable)
		begin
			duty_r_1   <= 10'd0;
			duty_r_2   <= 10'd0;
			duty_r_3   <= 10'd0;
		end else begin
			if(we_i == `WriteEnable) begin
                case (addr_i[3:0])
                    REG_DUTY_1: begin
                        duty_r_1 <= data_duty_i;
                    end
					REG_DUTY_2: begin
                        duty_r_2 <= data_duty_i;
                    end
					REG_DUTY_3: begin
                        duty_r_3 <= data_duty_i;
                    end
                endcase
			end
		end
	end
	
	//***************************** Main Code ****************************//
	//cnt_1us:1us计数器
	always@(posedge clk)
		if(rst == 1'b0)
			cnt_1us <= 6'b0;
		else    if(cnt_1us == CNT_1US_MAX)
			cnt_1us <= 6'b0;
		else
			cnt_1us <= cnt_1us + 1'b1;
	
	//cnt_1ms:1ms计数器
	always@(posedge clk)
		if(rst == 1'b0)
			cnt_1ms <= 10'b0;
		else    if(cnt_1ms == CNT_1MS_MAX && cnt_1us == CNT_1US_MAX)
			cnt_1ms <= 10'b0;
		else    if(cnt_1us == CNT_1US_MAX)
			cnt_1ms <= cnt_1ms + 1'b1;
	
	//always@(posedge clk or negedge rst)
	//	if(rst == 1'b0)
	//		duty_r <= 10'd0;
	//	else 
	//		duty_r <= 10'd666;//duty;

	//pwm_out_1:输出信号连接到外部的led灯
	always@(posedge clk)
		if((rst == 1'b0) || (light_sense == 1'b0))
			pwm_out_1 <= 1'b0;
		else    if( cnt_1ms <= duty_r_1 )
       // else    if( cnt_1ms <= 'd450)
			pwm_out_1 <= 1'b1;
		else
			pwm_out_1 <= 1'b0;
	assign led_out_io = pwm_out_1;
	
	//pwm_out_2:输出信号连接到外部的led灯
	always@(posedge clk)
		if((rst == 1'b0) || (light_sense == 1'b0))
			pwm_out_2 <= 1'b0;
	else    if( cnt_1ms <= duty_r_2 )
        //else    if( cnt_1ms <= 'd440 )
			pwm_out_2 <= 1'b1;
		else
			pwm_out_2 <= 1'b0;
			
	//pwm_out_3:输出信号连接到外部的led灯
	always@(posedge clk)
		if(rst == 1'b0)
			pwm_out_3 <= 1'b0;
		else    if( cnt_1ms <= duty_r_3 )
			pwm_out_3 <= 1'b1;
		else
			pwm_out_3<= 1'b0;
			
	// 读寄存器
    always @ (*) begin
        if (rst == `RstEnable) begin
            data_o = `ZeroWord;
        end else begin
            case (addr_i[3:0])
				//占空比寄存器输出
                REG_DUTY_1: begin
                    data_o = duty_r_1;
                end
                REG_DUTY_2: begin
                    data_o = duty_r_2;
                end
                REG_DUTY_3: begin
                    data_o = duty_r_3;
                end
                REG_HUM_FLAG: begin
                    data_o = hum_flag;
                end
                default: begin
                    data_o = `ZeroWord;
                end
            endcase
        end
    end
	
	//温湿度数据处理
	assign hum_flag_o = hum_flag[0]; 
    always@(posedge clk) begin
        if (rst == `RstEnable) begin
            hum_flag <= 10'd0;	
        end
		else if(tem_hum_i[31:16] < 16'd700)
			hum_flag <= 10'd1;
		else
			hum_flag <= 10'd0;
	end
	
endmodule






