// ==================== 两级流水线对角线 PE 模块 ====================
// 流水线级：Stage1(乘法) -> Stage2(加法+累加+输出)
module pe_diag #(
    parameter DATA_WIDTH = 16,
    parameter ACC_WIDTH = 40,
    parameter SAMPLES_BITS=4
)(
    input                           clk, rst_n,
    input                           finish_flag_in,
    input  signed [DATA_WIDTH-1:0]  data_q, data_i,
    output reg signed [DATA_WIDTH-1:0] a_data_out_q, a_data_out_i,
    output reg                      finish_flag_out,
    output reg signed [ACC_WIDTH-1:0]  result_q
);

    // ==================== 流水线级1: 乘法运算 ====================
    reg signed [ACC_WIDTH-1:0] power_q_stage1;
    reg signed [ACC_WIDTH-1:0] power_i_stage1;
    reg                           finish_flag_stage1;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            power_q_stage1 <= {ACC_WIDTH{1'b0}};
            power_i_stage1 <= {ACC_WIDTH{1'b0}};
            finish_flag_stage1 <= 1'b0;
        end else begin
            power_q_stage1 <= data_q * data_q;  // 乘法运算
            power_i_stage1 <= data_i * data_i;  // 乘法运算
            a_data_out_q <= data_q;
            a_data_out_i <= data_i;
            finish_flag_stage1 <= finish_flag_in;
        end
    end

    // ==================== 流水线级2: 加法+累加+输出 ====================
    reg signed [ACC_WIDTH-1:0] accumulator;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accumulator <= {ACC_WIDTH{1'b0}};
            result_q <= {ACC_WIDTH{1'b0}};
            a_data_out_q <= {DATA_WIDTH{1'b0}};
            a_data_out_i <= {DATA_WIDTH{1'b0}};
            finish_flag_out <= 1'b0;
        end else begin
            finish_flag_out <= finish_flag_stage1;

            if (finish_flag_stage1) begin
                // 加法+累加+输出同时进行
                result_q <= (accumulator + power_q_stage1 + power_i_stage1) >>> SAMPLES_BITS;
                accumulator <= {ACC_WIDTH{1'b0}};  // 清零
            end else begin
                // 加法+累加同时进行
                accumulator <= accumulator + power_q_stage1 + power_i_stage1;
            end
        end
    end
    
endmodule