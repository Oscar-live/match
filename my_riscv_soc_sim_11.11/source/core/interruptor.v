
//`include "defines.v"

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

// core local interruptor module
// 核心中断管理、仲裁模块
module clint(

    input wire clk,
    input wire rst,

    // from core
    input wire[`INT_BUS] int_flag_i,         // 中断输入信号

    // from id
    input wire[`InstBus] inst_i,             // 指令内容
    input wire[`InstAddrBus] inst_addr_i,    // 指令地址

    // from ex
    input wire jump_flag_i,
    input wire[`InstAddrBus] jump_addr_i,
    input wire div_started_i,

    // from ctrl
    input wire[`Hold_Flag_Bus] hold_flag_i,  // 流水线暂停标志

    // from csr_reg
    input wire[`RegBus] data_i,              // CSR寄存器输入数据
    input wire[`RegBus] csr_mtvec,           // mtvec寄存器
    input wire[`RegBus] csr_mepc,            // mepc寄存器
    input wire[`RegBus] csr_mstatus,         // mstatus寄存器

    input wire global_int_en_i,              // 全局中断使能标志

    // to ctrl
    output wire hold_flag_o,                 // 流水线暂停标志

    // to csr_reg
    output reg we_o,                         // 写CSR寄存器标志
    output reg[`MemAddrBus] waddr_o,         // 写CSR寄存器地址
    output reg[`MemAddrBus] raddr_o,         // 读CSR寄存器地址
    output reg[`RegBus] data_o,              // 写CSR寄存器数据

    // to ex
    output reg[`InstAddrBus] int_addr_o,     // 中断入口地址
    output reg int_assert_o                  // 中断标志

    );


    // 中断状态定义
    localparam S_INT_IDLE            = 4'b0001;
    localparam S_INT_SYNC_ASSERT     = 4'b0010;
    localparam S_INT_ASYNC_ASSERT    = 4'b0100;
    localparam S_INT_MRET            = 4'b1000;

    // 写CSR寄存器状态定义
    localparam S_CSR_IDLE            = 5'b00001;
    localparam S_CSR_MSTATUS         = 5'b00010;
    localparam S_CSR_MEPC            = 5'b00100;
    localparam S_CSR_MSTATUS_MRET    = 5'b01000;
    localparam S_CSR_MCAUSE          = 5'b10000;

    reg[3:0] int_state;
    reg[4:0] csr_state;
    reg[`InstAddrBus] inst_addr;
    reg[31:0] cause;


    assign hold_flag_o = ((int_state != S_INT_IDLE) | (csr_state != S_CSR_IDLE))? `HoldEnable: `HoldDisable;


    // 中断仲裁逻辑
    always @ (*) begin
        if (rst == `RstEnable) begin
            int_state = S_INT_IDLE;
        end else begin
            if (inst_i == `INST_ECALL || inst_i == `INST_EBREAK) begin
                // 如果执行阶段的指令为除法指令，则先不处理同步中断，等除法指令执行完再处理
                if (div_started_i == `DivStop) begin
                    int_state = S_INT_SYNC_ASSERT;
                end else begin
                    int_state = S_INT_IDLE;
                end
            end else if (int_flag_i != `INT_NONE && global_int_en_i == `True) begin
                int_state = S_INT_ASYNC_ASSERT;
            end else if (inst_i == `INST_MRET) begin
                int_state = S_INT_MRET;
            end else begin
                int_state = S_INT_IDLE;
            end
        end
    end

    // 写CSR寄存器状态切换
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            csr_state <= S_CSR_IDLE;
            cause <= `ZeroWord;
            inst_addr <= `ZeroWord;
        end else begin
            case (csr_state)
                S_CSR_IDLE: begin
                    // 同步中断
                    if (int_state == S_INT_SYNC_ASSERT) begin
                        csr_state <= S_CSR_MEPC;
                        // 在中断处理函数里会将中断返回地址加4
                        if (jump_flag_i == `JumpEnable) begin
                            inst_addr <= jump_addr_i - 4'h4;
                        end else begin
                            inst_addr <= inst_addr_i;
                        end
                        case (inst_i)
                            `INST_ECALL: begin
                                cause <= 32'd11;
                            end
                            `INST_EBREAK: begin
                                cause <= 32'd3;
                            end
                            default: begin
                                cause <= 32'd10;
                            end
                        endcase
                    // 异步中断
                    end else if (int_state == S_INT_ASYNC_ASSERT) begin
                        // 定时器中断
                        cause <= 32'h80000004;
                        csr_state <= S_CSR_MEPC;
                        if (jump_flag_i == `JumpEnable) begin
                            inst_addr <= jump_addr_i;
                        // 异步中断可以中断除法指令的执行，中断处理完再重新执行除法指令
                        end else if (div_started_i == `DivStart) begin
                            inst_addr <= inst_addr_i - 4'h4;
                        end else begin
                            inst_addr <= inst_addr_i;
                        end
                    // 中断返回
                    end else if (int_state == S_INT_MRET) begin
                        csr_state <= S_CSR_MSTATUS_MRET;
                    end
                end
                S_CSR_MEPC: begin
                    csr_state <= S_CSR_MSTATUS;
                end
                S_CSR_MSTATUS: begin
                    csr_state <= S_CSR_MCAUSE;
                end
                S_CSR_MCAUSE: begin
                    csr_state <= S_CSR_IDLE;
                end
                S_CSR_MSTATUS_MRET: begin
                    csr_state <= S_CSR_IDLE;
                end
                default: begin
                    csr_state <= S_CSR_IDLE;
                end
            endcase
        end
    end

    // 发出中断信号前，先写几个CSR寄存器
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            we_o <= `WriteDisable;
            waddr_o <= `ZeroWord;
            data_o <= `ZeroWord;
        end else begin
            case (csr_state)
                // 将mepc寄存器的值设为当前指令地址
                S_CSR_MEPC: begin
                    we_o <= `WriteEnable;
                    waddr_o <= {20'h0, `CSR_MEPC};
                    data_o <= inst_addr;
                end
                // 写中断产生的原因
                S_CSR_MCAUSE: begin
                    we_o <= `WriteEnable;
                    waddr_o <= {20'h0, `CSR_MCAUSE};
                    data_o <= cause;
                end
                // 关闭全局中断
                S_CSR_MSTATUS: begin
                    we_o <= `WriteEnable;
                    waddr_o <= {20'h0, `CSR_MSTATUS};
                    data_o <= {csr_mstatus[31:4], 1'b0, csr_mstatus[2:0]};
                end
                // 中断返回
                S_CSR_MSTATUS_MRET: begin
                    we_o <= `WriteEnable;
                    waddr_o <= {20'h0, `CSR_MSTATUS};
                    data_o <= {csr_mstatus[31:4], csr_mstatus[7], csr_mstatus[2:0]};
                end
                default: begin
                    we_o <= `WriteDisable;
                    waddr_o <= `ZeroWord;
                    data_o <= `ZeroWord;
                end
            endcase
        end
    end

    // 发出中断信号给ex模块
    always @ (posedge clk) begin
        if (rst == `RstEnable) begin
            int_assert_o <= `INT_DEASSERT;
            int_addr_o <= `ZeroWord;
        end else begin
            case (csr_state)
                // 发出中断进入信号.写完mcause寄存器才能发
                S_CSR_MCAUSE: begin
                    int_assert_o <= `INT_ASSERT;
                    int_addr_o <= csr_mtvec;
                end
                // 发出中断返回信号
                S_CSR_MSTATUS_MRET: begin
                    int_assert_o <= `INT_ASSERT;
                    int_addr_o <= csr_mepc;
                end
                default: begin
                    int_assert_o <= `INT_DEASSERT;
                    int_addr_o <= `ZeroWord;
                end
            endcase
        end
    end

endmodule
