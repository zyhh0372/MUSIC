// ==================== 两级流水线非对角线 PE 模块 ====================
// 流水线级：Stage1(乘法) -> Stage2(加减法+累加+输出)
module pe_offdiag #(
    parameter DATA_WIDTH = 16,
    parameter ACC_WIDTH = 40,
    parameter SAMPLES_BITS=4
)(
    input                           clk, rst_n,
    input                           finish_flag_in,
    input  signed [DATA_WIDTH-1:0]  a_data_q, a_data_i,      // 来自左边
    input  signed [DATA_WIDTH-1:0]  b_data_q, b_data_i,      // 来自上边
    output reg signed [DATA_WIDTH-1:0] a_data_out_q, a_data_out_i, // 向右传递
    output reg signed [DATA_WIDTH-1:0] b_data_out_q, b_data_out_i, // 向下传递
    output reg                      finish_flag_out,
    output reg signed [ACC_WIDTH-1:0]  result_q, result_i
);

    // ==================== 流水线级1: 复数乘法的4个乘法运算 ====================
    reg signed [ACC_WIDTH-1:0] mult_ac_stage1;
    reg signed [ACC_WIDTH-1:0] mult_bd_stage1;
    reg signed [ACC_WIDTH-1:0] mult_bc_stage1;
    reg signed [ACC_WIDTH-1:0] mult_ad_stage1;
    reg                           finish_flag_stage1;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_ac_stage1 <= {ACC_WIDTH{1'b0}};
            mult_bd_stage1 <= {ACC_WIDTH{1'b0}};
            mult_bc_stage1 <= {ACC_WIDTH{1'b0}};
            mult_ad_stage1 <= {ACC_WIDTH{1'b0}};
            finish_flag_stage1 <= 1'b0;
        end else begin
            // 复数乘法：(a+jb) * (c-jd) = (ac+bd) + j(bc-ad)
            mult_ac_stage1 <= a_data_q * b_data_q;  // 乘法运算
            mult_bd_stage1 <= a_data_i * b_data_i;  // 乘法运算
            mult_bc_stage1 <= a_data_i * b_data_q;  // 乘法运算
            mult_ad_stage1 <= a_data_q * b_data_i;  // 乘法运算

            finish_flag_stage1 <= finish_flag_in;
        end
    end

    // ==================== 流水线级2: 加减法+累加+输出 ====================
    reg signed [ACC_WIDTH-1:0] acc_q, acc_i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_q <= {ACC_WIDTH{1'b0}};
            acc_i <= {ACC_WIDTH{1'b0}};
            result_q <= {ACC_WIDTH{1'b0}};
            result_i <= {ACC_WIDTH{1'b0}};
            finish_flag_out <= 1'b0;
        end else begin
            finish_flag_out <= finish_flag_stage1;
            
            if (finish_flag_stage1) begin
                // 加减法+累加+输出同时进行
                result_q <= (acc_q + mult_ac_stage1 + mult_bd_stage1) >>> SAMPLES_BITS;
                result_i <= (acc_i + mult_bc_stage1 - mult_ad_stage1) >>> SAMPLES_BITS;
                acc_q <= {ACC_WIDTH{1'b0}};  // 清零
                acc_i <= {ACC_WIDTH{1'b0}};
            end else begin
                // 加减法+累加同时进行
                acc_q <= acc_q + mult_ac_stage1 + mult_bd_stage1;
                acc_i <= acc_i +  mult_ad_stage1 -mult_bc_stage1 ;
            end
        end
    end
    
    // ==================== 数据传递（增加延迟以匹配流水线） ====================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_data_out_q <= {DATA_WIDTH{1'b0}};
            a_data_out_i <= {DATA_WIDTH{1'b0}};
            b_data_out_q <= {DATA_WIDTH{1'b0}};
            b_data_out_i <= {DATA_WIDTH{1'b0}};
        end else begin
            a_data_out_q <= a_data_q;
            a_data_out_i <= a_data_i;
            b_data_out_q <= b_data_q;
            b_data_out_i <= b_data_i;
        end
    end

endmodule