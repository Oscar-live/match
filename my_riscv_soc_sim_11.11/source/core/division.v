 /*                                                                      
 Copyright 2019 Blue Liang, liangkangnan@163.com
                                                                         
 Licensed under the Apache License, Version 2.0 (the "License");         
 you may not use this file except in compliance with the License.        
 You may obtain a copy of the License at                                 
                                                                         
     http://www.apache.org/licenses/LICENSE-2.0                          
                                                                         
 Unless required by applicable law or agreed to in writing, software    
 distributed under the License is distributed on an "AS IS" BASIS,       
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and     
 limitations under the License.                                          
 */
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

//`include "defines.v"

// 除法模块
// 试商法实现32位整数除法
// 每次除法运算至少需要33个时钟周期才能完成
module div(

    input wire clk,
    input wire rst,

    // from ex
    input wire[`RegBus] dividend_i,      // 被除数
    input wire[`RegBus] divisor_i,       // 除数
    input wire start_i,                  // 开始信号，运算期间这个信号需要一直保持有效
    input wire[2:0] op_i,                // 具体是哪一条指令
    input wire[`RegAddrBus] reg_waddr_i, // 运算结束后需要写的寄存器

    // to ex
    output reg[`RegBus] result_o,        // 除法结果，高32位是余数，低32位是商
    output reg ready_o,                  // 运算结束信号
    output reg busy_o,                  // 正在运算信号
    output reg[`RegAddrBus] reg_waddr_o  // 运算结束后需要写的寄存器

    );

    // 状态定义
    localparam STATE_IDLE  = 4'b0001;
    localparam STATE_START = 4'b0010;
    localparam STATE_CALC  = 4'b0100;
    localparam STATE_END   = 4'b1000;

    reg[`RegBus] dividend_r;
    reg[`RegBus] divisor_r;
    reg[2:0] op_r;
    reg[3:0] state;
    reg[31:0] count;
    reg[`RegBus] div_result;
    reg[`RegBus] div_remain;
    reg[`RegBus] minuend;
    reg invert_result;

    wire op_div = (op_r == `INST_DIV);
    wire op_divu = (op_r == `INST_DIVU);
    wire op_rem = (op_r == `INST_REM);
    wire op_remu = (op_r == `INST_REMU);

    wire[31:0] dividend_invert = (-dividend_r);
    wire[31:0] divisor_invert = (-divisor_r);
    wire minuend_ge_divisor = minuend >= divisor_r;
    wire[31:0] minuend_sub_res = minuend - divisor_r;
    wire[31:0] div_result_tmp = minuend_ge_divisor? ({div_result[30:0], 1'b1}): ({div_result[30:0], 1'b0});
    wire[31:0] minuend_tmp = minuend_ge_divisor? minuend_sub_res[30:0]: minuend[30:0];

    // 状态机实现
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            state <= STATE_IDLE;
            ready_o <= `DivResultNotReady;
            result_o <= `ZeroWord;
            div_result <= `ZeroWord;
            div_remain <= `ZeroWord;
            op_r <= 3'h0;
            reg_waddr_o <= `ZeroWord;
            dividend_r <= `ZeroWord;
            divisor_r <= `ZeroWord;
            minuend <= `ZeroWord;
            invert_result <= 1'b0;
            busy_o <= `False;
            count <= `ZeroWord;
        end else begin
            case (state)
                STATE_IDLE: begin
                    if (start_i == `DivStart) begin
                        op_r <= op_i;
                        dividend_r <= dividend_i;
                        divisor_r <= divisor_i;
                        reg_waddr_o <= reg_waddr_i;
                        state <= STATE_START;
                        busy_o <= `True;
                    end else begin
                        op_r <= 3'h0;
                        reg_waddr_o <= `ZeroWord;
                        dividend_r <= `ZeroWord;
                        divisor_r <= `ZeroWord;
                        ready_o <= `DivResultNotReady;
                        result_o <= `ZeroWord;
                        busy_o <= `False;
                    end
                end

                STATE_START: begin
                    if (start_i == `DivStart) begin
                        // 除数为0
                        if (divisor_r == `ZeroWord) begin
                            if (op_div | op_divu) begin
                                result_o <= 32'hffffffff;
                            end else begin
                                result_o <= dividend_r;
                            end
                            ready_o <= `DivResultReady;
                            state <= STATE_IDLE;
                            busy_o <= `False;
                        // 除数不为0
                        end else begin
                            busy_o <= `True;
                            count <= 32'h40000000;
                            state <= STATE_CALC;
                            div_result <= `ZeroWord;
                            div_remain <= `ZeroWord;

                            // DIV和REM这两条指令是有符号数运算指令
                            if (op_div | op_rem) begin
                                // 被除数求补码
                                if (dividend_r[31] == 1'b1) begin
                                    dividend_r <= dividend_invert;
                                    minuend <= dividend_invert[31];
                                end else begin
                                    minuend <= dividend_r[31];
                                end
                                // 除数求补码
                                if (divisor_r[31] == 1'b1) begin
                                    divisor_r <= divisor_invert;
                                end
                            end else begin
                                minuend <= dividend_r[31];
                            end

                            // 运算结束后是否要对结果取补码
                            if ((op_div && (dividend_r[31] ^ divisor_r[31] == 1'b1))
                                || (op_rem && (dividend_r[31] == 1'b1))) begin
                                invert_result <= 1'b1;
                            end else begin
                                invert_result <= 1'b0;
                            end
                        end
                    end else begin
                        state <= STATE_IDLE;
                        result_o <= `ZeroWord;
                        ready_o <= `DivResultNotReady;
                        busy_o <= `False;
                    end
                end

                STATE_CALC: begin
                    if (start_i == `DivStart) begin
                        dividend_r <= {dividend_r[30:0], 1'b0};
                        div_result <= div_result_tmp;
                        count <= {1'b0, count[31:1]};
                        if (|count) begin
                            minuend <= {minuend_tmp[30:0], dividend_r[30]};
                        end else begin
                            state <= STATE_END;
                            if (minuend_ge_divisor) begin
                                div_remain <= minuend_sub_res;
                            end else begin
                                div_remain <= minuend;
                            end
                        end
                    end else begin
                        state <= STATE_IDLE;
                        result_o <= `ZeroWord;
                        ready_o <= `DivResultNotReady;
                        busy_o <= `False;
                    end
                end

                STATE_END: begin
                    if (start_i == `DivStart) begin
                        ready_o <= `DivResultReady;
                        state <= STATE_IDLE;
                        busy_o <= `False;
                        if (op_div | op_divu) begin
                            if (invert_result) begin
                                result_o <= (-div_result);
                            end else begin
                                result_o <= div_result;
                            end
                        end else begin
                            if (invert_result) begin
                                result_o <= (-div_remain);
                            end else begin
                                result_o <= div_remain;
                            end
                        end
                    end else begin
                        state <= STATE_IDLE;
                        result_o <= `ZeroWord;
                        ready_o <= `DivResultNotReady;
                        busy_o <= `False;
                    end
                end

            endcase
        end
    end

endmodule
