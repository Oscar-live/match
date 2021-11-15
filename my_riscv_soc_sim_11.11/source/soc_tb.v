`timescale 1ns / 1ns
module soc_tb;

reg  clk;
reg  rst;
wire out;
wire [31:0] pc,inst_addr,inst_data;
wire [31:0] pwm_duty_i,pwm_duty1,pwm_duty2,pwm_duty3,rib_pwm_addr,pwm_addr;
wire [9:0]  pwm_cnt;
wire       pwm_out1,pwm_out2,pwm_out3;
initial begin
clk = 1'b0;
rst = 1'b0;
#20
rst = 1'b1;
end

always #10 clk = ~clk;


assign pc        = tinyriscv_soc_top_inst.u_tinyriscv.pc_pc_o;
assign inst_addr = tinyriscv_soc_top_inst.u_tinyriscv.rib_pc_addr_o;
assign inst_data = tinyriscv_soc_top_inst.u_tinyriscv.rib_pc_data_i;

assign pwm_duty1 = tinyriscv_soc_top_inst.u_pwm.duty_r_1;
assign pwm_duty2 = tinyriscv_soc_top_inst.u_pwm.duty_r_2;
assign pwm_duty3 = tinyriscv_soc_top_inst.u_pwm.duty_r_3;

assign pwm_cnt = tinyriscv_soc_top_inst.u_pwm.cnt_1ms;     
assign rib_pwm_addr = tinyriscv_soc_top_inst.m0_addr_i;  
assign pwm_addr = tinyriscv_soc_top_inst.pwm_addr_o;

assign pwm_duty_i = tinyriscv_soc_top_inst.u_pwm.data_duty_i;

assign pwm_out1 = tinyriscv_soc_top_inst.pwm_out_1;
assign pwm_out2 = tinyriscv_soc_top_inst.pwm_out_2;
assign pwm_out3 = tinyriscv_soc_top_inst.pwm_out_3;

tinyriscv_soc_top tinyriscv_soc_top_inst(

    .clk         ( clk           )    ,
    .rst         ( rst           )    ,
    .led_out_io  ( out           )    // GPIOÒý½Å
    );
endmodule