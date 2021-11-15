 /*                                                                      
                                        
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
// tinyriscv soc����ģ��
module tinyriscv_soc_top(

    input wire clk,
    input wire rst,
    input wire stop,         //����ȫ�ֿ���

    output wire uart_tx_pin, // UART��������
    input wire uart_rx_pin,  // UART��������
    inout wire[9:0] gpio,    // GPIO����

    input wire jtag_TCK,     // JTAG TCK����
    input wire jtag_TMS,     // JTAG TMS����
    input wire jtag_TDI,     // JTAG TDI����
    output wire jtag_TDO,    // JTAG TDO����

    input wire spi_miso,     // SPI MISO����
    output wire spi_mosi,    // SPI MOSI����
    output wire spi_ss,      // SPI SS����
    output wire spi_clk,     // SPI CLK����
	
    //PWM�ź�
	 output wire pwm_out_1,	 	//pwm�����
	 output wire pwm_out_2,	 	//pwm�����
	 output wire pwm_out_3,	 	//pwm�����
	 output wire led_out_io,     //pwm���������

	//��ʪ�ȴ����ź�
    inout         dht22,        //��ʪ�ȴ�������  
	 output		  hum_flag_led,   //��ʪ��������־�Ĳ������
    output wire       hum_flag_o,
	//��紫��
	input  wire	  light_sense,
	//8λ������ź�
    output wire   SCLK,                //shcp
    output wire   RCLK,                //stcp
    output wire   DIO,
	//���ڷ�������
	 output wire	  tx,
    output wire   tx_uart
    );
    assign tx_uart = tx;

    // master 0 interface
    wire[`MemAddrBus] m0_addr_i;
    wire[`MemBus] m0_data_i;
    wire[`MemBus] m0_data_o;
    wire m0_req_i;
    wire m0_we_i;

    // master 1 interface
    wire[`MemAddrBus] m1_addr_i;
    wire[`MemBus] m1_data_i;
    wire[`MemBus] m1_data_o;
    wire m1_req_i;
    wire m1_we_i;

    // master 2 interface
    wire[`MemAddrBus] m2_addr_i;
    wire[`MemBus] m2_data_i;
    wire[`MemBus] m2_data_o;
    wire m2_req_i;
    wire m2_we_i;

    // master 3 interface
    wire[`MemAddrBus] m3_addr_i;
    wire[`MemBus] m3_data_i;
    wire[`MemBus] m3_data_o;
    wire m3_req_i;
    wire m3_we_i;

    // slave 0 interface
    wire[`MemAddrBus] s0_addr_o;
    wire[`MemBus] s0_data_o;
    wire[`MemBus] s0_data_i;
    wire s0_we_o;

    // slave 1 interface
    wire[`MemAddrBus] s1_addr_o;
    wire[`MemBus] s1_data_o;
    wire[`MemBus] s1_data_i;
    wire s1_we_o;

    // slave 2 interface
    wire[`MemAddrBus] s2_addr_o;
    wire[`MemBus] s2_data_o;
    wire[`MemBus] s2_data_i;
    wire s2_we_o;

    // slave 3 interface
    wire[`MemAddrBus] s3_addr_o;
    wire[`MemBus] s3_data_o;
    wire[`MemBus] s3_data_i;
    wire s3_we_o;

    // slave 4 interface
    wire[`MemAddrBus] s4_addr_o;
    wire[`MemBus] s4_data_o;
    wire[`MemBus] s4_data_i;
    wire s4_we_o;

    // slave 5 interface
    wire[`MemAddrBus] s5_addr_o;
    wire[`MemBus] s5_data_o;
    wire[`MemBus] s5_data_i;
    wire s5_we_o;
	
	// slave pwm interface
    wire[`MemAddrBus] pwm_addr_o;
    wire[`MemBus] pwm_data_o;
    wire[`MemBus] pwm_data_i;
    wire pwm_we_o;
	
	
	

    // rib
    wire rib_hold_flag_o;

    // jtag
    wire jtag_halt_req_o;
    wire jtag_reset_req_o;
    wire[`RegAddrBus] jtag_reg_addr_o;
    wire[`RegBus] jtag_reg_data_o;
    wire jtag_reg_we_o;
    wire[`RegBus] jtag_reg_data_i;

    // tinyriscv
    wire[`INT_BUS] int_flag;

    // timer0
    wire timer0_int;

    // gpio
    wire[9:0] io_in;
    wire[31:0] gpio_ctrl;
    wire[31:0] gpio_data;

    assign int_flag = {7'h0, timer0_int};

    // �͵�ƽ����LED
    // �͵�ƽ��ʾ�Ѿ�haltסCPU
    assign halted_ind = ~jtag_halt_req_o;
    reg  over,succ;
    wire[`RegBus] regs_26;  // �����Ƿ�����ź�
    wire[`RegBus] regs_27;  // �����Ƿ�ɹ��ź�
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            over <= 1'b1;
            succ <= 1'b1;
        end else begin
            over <= ~regs_26;    // when = 1, run over                
            succ <= ~regs_27;    // when = 1, run succ, otherwise fail
        end
    end


    //------------��������������ȫ��ʱ���ź�--------------------//
    //wire stop_flag;
    //wire clk;
    //reg  clk_lock;
    
    //key_filter key_filter_inst(
    //    .sys_clk     (clk_sys          ),   //ϵͳʱ��50Mhz
    //    .sys_rst_n   (rst              ),           //ȫ�ָ�λ
    //    .key_in      (stop             ),   //���������ź�
    //    .key_flag    (stop_flag        )    //key_flagΪ1ʱ��ʾ�������⵽����������
    //                                      
    //);
    
    //always@(posedge clk_sys)
    //    if(rst == 1'b0)
    //        clk_lock <= 1'b1;
    //    else if(stop_flag)
    //        clk_lock <= ~clk_lock;
    //    else
    //        clk_lock <= clk_lock;
    
    //assign clk =( 1'b1 )? clk_sys : 1'b0;
	wire [31:0]		tem_hum; //��ʪ������
    top_dht22 top_dht22_inst(   
        .clk  (clk),
        .rst  (rst),
    	//��ʪ�ȴ����ź�
        .dht22(dht22),              //   
    	.tem_hum(tem_hum),
    	//8λ������ź�       
        .SCLK (SCLK),                //shcp
        .RCLK (RCLK),                //stcp
        .DIO  (DIO),
    	//���ڷ�������
    	.tx   (tx),
    	//����
    	.led  ()
     );

    // tinyriscv��������ģ������
    tinyriscv u_tinyriscv(
        .clk(clk),
        .rst(rst),
        .rib_ex_addr_o(m0_addr_i),
        .rib_ex_data_i(m0_data_o),
        .rib_ex_data_o(m0_data_i),
        .rib_ex_req_o(m0_req_i),
        .rib_ex_we_o(m0_we_i),

        .rib_pc_addr_o(m1_addr_i),
        .rib_pc_data_i(m1_data_o),

        .jtag_reg_addr_i(jtag_reg_addr_o),
        .jtag_reg_data_i(jtag_reg_data_o),
        .jtag_reg_we_i(jtag_reg_we_o),
        .jtag_reg_data_o(jtag_reg_data_i),

        .rib_hold_flag_i(rib_hold_flag_o),
        .jtag_halt_flag_i(jtag_halt_req_o),
        .jtag_reset_flag_i(jtag_reset_req_o),

        .int_i(int_flag),
		.regs_26(regs_26),
		.regs_27(regs_27)
    );

    // romģ������
    rom u_rom(
        .clk(clk),
        .rst(rst),
        .we_i(s0_we_o),
        .addr_i(s0_addr_o),
        .data_i(s0_data_o),
        .data_o(s0_data_i)
    );

    // ramģ������
    ram u_ram(
        .clk(clk),
        .rst(rst),
        .we_i(s1_we_o),
        .addr_i(s1_addr_o),
        .data_i(s1_data_o),
        .data_o(s1_data_i)
    );

    // timerģ������
    timer timer_0(
        .clk(clk),
        .rst(rst),
        .data_i(s2_data_o),
        .addr_i(s2_addr_o),
        .we_i(s2_we_o),
        .data_o(s2_data_i),
        .int_sig_o(timer0_int)
    );

    // uartģ������
    uart uart_0(
        .clk(clk),
        .rst(rst),
        .we_i(s3_we_o),
        .addr_i(s3_addr_o),
        .data_i(s3_data_o),
        .data_o(s3_data_i),
        .tx_pin(uart_tx_pin),
        .rx_pin(uart_rx_pin)
    );

	//GPIO����
     //assign gpio[0] = 1'b0;
     //assign gpio[1] = 1'b1;
     //assign gpio[2] = 1'b0;
     //assign gpio[3] = 1'b1;
     //assign gpio[4] = 1'b1;//stby������п���

    // io0 // 0: ���裬1�������2������
    assign gpio[0] = (gpio_ctrl[1:0] == 2'b01)? gpio_data[0]: 1'bz;
    assign io_in[0] = gpio[0];
    // io1 // 0: ���裬1�������2������
    assign gpio[1] = (gpio_ctrl[3:2] == 2'b01)? gpio_data[1]: 1'bz;
    assign io_in[1] = gpio[1];
    // io2 // 0: ���裬1�������2������
    assign gpio[2] = (gpio_ctrl[5:4] == 2'b01)? gpio_data[2]: 1'bz;
    assign io_in[2] = gpio[2];
    // io3 // 0: ���裬1�������2������
    assign gpio[3] = (gpio_ctrl[7:6] == 2'b01)? gpio_data[3]: 1'bz;
    assign io_in[3] = gpio[3];
    // io4 // 0: ���裬1�������2������
    assign gpio[4] = (gpio_ctrl[9:8] == 2'b01)? gpio_data[4]: 1'bz;
    assign io_in[4] = gpio[4];
    // io5 // 0: ���裬1�������2������
    assign gpio[5] = (gpio_ctrl[11:10] == 2'b01)? gpio_data[5]: 1'bz;
    assign io_in[5] = gpio[5];
    // io6 // 0: ���裬1�������2������
    assign gpio[6] = (gpio_ctrl[13:12] == 2'b01)? gpio_data[6]: 1'bz;
    assign io_in[6] = gpio[6];
    // io7 // 0: ���裬1�������2������
    assign gpio[7] = (gpio_ctrl[15:14] == 2'b01)? gpio_data[7]: 1'bz;
    assign io_in[7] = gpio[7];
    // io8 // 0: ���裬1�������2������
    assign gpio[8] = (gpio_ctrl[17:16] == 2'b01)? gpio_data[8]: 1'bz;
    assign io_in[8] = gpio[8];
    // io9 // 0: ���裬1�������2������
    assign gpio[9] = (gpio_ctrl[19:18] == 2'b01)? gpio_data[9]: 1'bz;
    assign io_in[9] = gpio[9];

    // gpioģ������
    gpio gpio_0(
        .clk(clk),			
        .rst(rst),
		
        .we_i(s4_we_o),			
        .addr_i(s4_addr_o),		//����ӻ���GPIO��ؼĴ�����ַ
        .data_i(s4_data_o),     //���ģʽ��д��ӻ���GPIO�����ƽ
		                        
        .data_o(s4_data_i),     //���ӻ�GPIO�Ĵ����������
		                        
        .io_pin_i(io_in),       //����ģʽ�£�GPIO����������
        .reg_ctrl(gpio_ctrl),   //���д���GPIO���ƼĴ���
        .reg_data(gpio_data)    //����ģʽ�£�ģ���������д���GPIO�����ƽ
    );

    // spiģ������
    //spi spi_0(
    //    .clk(clk),
    //    .rst(rst),
    //    .data_i(s5_data_o),
    //    .addr_i(s5_addr_o),
    //    .we_i(s5_we_o),
    //    .data_o(s5_data_i),
    //    .spi_mosi(spi_mosi),
    //    .spi_miso(spi_miso),
    //    .spi_ss(spi_ss),
    //    .spi_clk(spi_clk)
    //);
	
	//pwm ģ������

   // wire   hum_flag_o;
    assign hum_flag_led = hum_flag_o;
	pwm u_pwm(
	    .clk(clk),
	    .rst(rst),
	    
	    .data_duty_i (pwm_data_o),		//pwm ���� �� ռ�ձ�
	    .addr_i 			(pwm_addr_o),
	    .we_i				(pwm_we_o),
	    //��ʪ��
	    .tem_hum_i			(tem_hum),
	    .hum_flag_o			(hum_flag_o),//��ʪ��������־�Ĳ������
	    //��紫��
	    .light_sense		(light_sense),
	    
	    
	    .data_o				(pwm_data_i),
	    .pwm_out_1 	        (pwm_out_1),         //pwm reg���
	    .pwm_out_2 	        (pwm_out_2),         //pwm reg���
	    .pwm_out_3 	        (pwm_out_3),         //pwm reg���
	    .led_out_io			(led_out_io)	   //pwm wire���
	);

    // ribģ������
    rib u_rib(
        .clk(clk),
        .rst(rst),

        // master 0 interface
        .m0_addr_i(m0_addr_i),
        .m0_data_i(m0_data_i),
        .m0_data_o(m0_data_o),
        .m0_req_i(m0_req_i),
        .m0_we_i(m0_we_i),

        // master 1 interface
        .m1_addr_i(m1_addr_i),
        .m1_data_i(`ZeroWord),
        .m1_data_o(m1_data_o),
        .m1_req_i(`RIB_REQ),
        .m1_we_i(`WriteDisable),

        // master 2 interface
        .m2_addr_i(m2_addr_i),
        .m2_data_i(m2_data_i),
        .m2_data_o(m2_data_o),
        .m2_req_i(m2_req_i),
        .m2_we_i(m2_we_i),

        // master 3 interface
        .m3_addr_i(m3_addr_i),
        .m3_data_i(m3_data_i),
        .m3_data_o(m3_data_o),
        .m3_req_i(m3_req_i),
        .m3_we_i(m3_we_i),

        // slave 0 interface
        .s0_addr_o(s0_addr_o),
        .s0_data_o(s0_data_o),
        .s0_data_i(s0_data_i),
        .s0_we_o(s0_we_o),

        // slave 1 interface
        .s1_addr_o(s1_addr_o),
        .s1_data_o(s1_data_o),
        .s1_data_i(s1_data_i),
        .s1_we_o(s1_we_o),

        // slave 2 interface
        .s2_addr_o(s2_addr_o),
        .s2_data_o(s2_data_o),
        .s2_data_i(s2_data_i),
        .s2_we_o(s2_we_o),

        // slave 3 interface
        .s3_addr_o(s3_addr_o),
        .s3_data_o(s3_data_o),
        .s3_data_i(s3_data_i),
        .s3_we_o(s3_we_o),

        // slave 4 interface
        .s4_addr_o(s4_addr_o),
        .s4_data_o(s4_data_o),
        .s4_data_i(s4_data_i),
        .s4_we_o(s4_we_o),
        
        // slave 5 interface
        .s5_addr_o(s5_addr_o),
        .s5_data_o(s5_data_o),
        .s5_data_i(s5_data_i),
        .s5_we_o(s5_we_o),
		
		// pwm interface
		.pwm_addr_o(pwm_addr_o), // ������ӻ��� �ӻ��Ĵ�����ַ
		.pwm_data_o(pwm_data_o), // ����д��ӻ�������
		.pwm_data_i(pwm_data_i), // �ӻ����͵�����������
		.pwm_we_o  (pwm_we_o  ), // �������뵽�ӻ���д��־

        .hold_flag_o(rib_hold_flag_o)
    );

   // jtagģ������
   jtag_top #(
       .DMI_ADDR_BITS(6),
       .DMI_DATA_BITS(32),
       .DMI_OP_BITS(2)
   ) u_jtag_top(
       .clk(clk),
       .jtag_rst_n(rst),
       .jtag_pin_TCK(jtag_TCK),
       .jtag_pin_TMS(jtag_TMS),
       .jtag_pin_TDI(jtag_TDI),
       .jtag_pin_TDO(jtag_TDO),
       .reg_we_o(jtag_reg_we_o),
       .reg_addr_o(jtag_reg_addr_o),
       .reg_wdata_o(jtag_reg_data_o),
       .reg_rdata_i(jtag_reg_data_i),
       .mem_we_o(m2_we_i),
       .mem_addr_o(m2_addr_i),
       .mem_wdata_o(m2_data_i),
       .mem_rdata_i(m2_data_o),
       .op_req_o(m2_req_i),
       .halt_req_o(jtag_halt_req_o),
       .reset_req_o(jtag_reset_req_o)
   );

endmodule
