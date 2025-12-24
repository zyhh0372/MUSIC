module Right_Rotation #(
    parameter ACC_WIDTH = 20,
    parameter N = 4
) (
    input                               clk,
    input                               rst_n,
    input                               start,               // 开始信号
    input signed [ACC_WIDTH-1:0]        matrix_col_p [0:N-1], // 第p列数据
    input signed [ACC_WIDTH-1:0]        matrix_col_q [0:N-1], // 第q列数据
    input signed [15:0]                 sin_theta,           // sin(θ)
    input signed [15:0]                 cos_theta,           // cos(θ)
    
    output reg                          done,                // 完成信号
    output reg signed [ACC_WIDTH-1:0]   matrix_col_p_new [0:N-1], // 更新后的第p列
    output reg signed [ACC_WIDTH-1:0]   matrix_col_q_new [0:N-1]  // 更新后的第q列
);

    integer i;

    // ========== 元素索引计数器（串行控制） ==========
    reg [$clog2(N)-1:0] elem_idx;        // 当前处理的元素索引
    reg processing;                      // 处理中标志

    // ========== 输入数据缓存 ==========
    reg signed [ACC_WIDTH-1:0] matrix_p_buf [0:N-1];
    reg signed [ACC_WIDTH-1:0] matrix_q_buf [0:N-1];
    reg signed [15:0] sin_buf, cos_buf;

    // ========== 流水线Stage 1: 乘法运算（单个元素） ==========
    reg valid_s1, valid_s2;  // 只需2级有效位
    reg [$clog2(N)-1:0] idx_s1, idx_s2;  // 流水线中的索引

    reg signed [ACC_WIDTH-1:0] mult_cos_p_s1;   // cos * p[i]
    reg signed [ACC_WIDTH-1:0] mult_sin_q_s1;   // sin * q[i]
    reg signed [ACC_WIDTH-1:0] mult_cos_q_s1;   // cos * q[i]
    reg signed [ACC_WIDTH-1:0] mult_sin_p_s1;   // sin * p[i]

    // ========== 流水线Stage 2: 加减法 + 右移（合并） ==========
    reg signed [ACC_WIDTH-1:0] result_p_s2;
    reg signed [ACC_WIDTH-1:0] result_q_s2;

    // ========== 串行控制逻辑 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            processing <= 1'b0;
            elem_idx <= 0;
            done <= 1'b0;
            for (i = 0; i < N; i = i + 1) begin
                matrix_p_buf[i] <= 0;
                matrix_q_buf[i] <= 0;
            end
            sin_buf <= 0;
            cos_buf <= 0;
        end else begin
            done <= 1'b0;  // 默认为0
            
            if (start && !processing) begin
                // 开始处理：缓存输入数据
                processing <= 1'b1;
                elem_idx <= 0;
                for (i = 0; i < N; i = i + 1) begin
                    matrix_p_buf[i] <= matrix_col_p[i];
                    matrix_q_buf[i] <= matrix_col_q[i];
                end
                sin_buf <= sin_theta;
                cos_buf <= cos_theta;
            end else if (processing) begin
                if (elem_idx < N-1) begin
                    elem_idx <= elem_idx + 1;
                end else begin
                    // 等待最后2个周期（而非3个）
                    if (valid_s2 && idx_s2 == N-1) begin
                        processing <= 1'b0;
                        done <= 1'b1;
                    end
                end
            end
        end
    end

    // ========== 流水线有效位和索引传递 ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
            valid_s2 <= 1'b0;
            idx_s1 <= 0;
            idx_s2 <= 0;
        end else begin
            // Stage 1 有效：当正在处理且有元素待处理
            valid_s1 <= processing && (elem_idx < N || elem_idx == 0);
            idx_s1 <= elem_idx;
            
            // Stage 2 有效
            valid_s2 <= valid_s1;
            idx_s2 <= idx_s1;
        end
    end

    // ========== Stage 1: 乘法运算（单个元素） ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_cos_p_s1 <= 0;
            mult_sin_q_s1 <= 0;
            mult_cos_q_s1 <= 0;
            mult_sin_p_s1 <= 0;
        end else if (processing && elem_idx < N) begin
            mult_cos_p_s1 <= cos_buf * matrix_p_buf[elem_idx];
            mult_sin_q_s1 <= sin_buf * matrix_q_buf[elem_idx];
            mult_cos_q_s1 <= cos_buf * matrix_q_buf[elem_idx];
            mult_sin_p_s1 <= sin_buf * matrix_p_buf[elem_idx];
        end
    end

    // ========== Stage 2: 加减法 + 右移（合并） ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_p_s2 <= 0;
            result_q_s2 <= 0;
        end else if (valid_s1) begin
            result_p_s2 <= (mult_cos_p_s1 + mult_sin_q_s1);
            result_q_s2 <= (mult_cos_q_s1 - mult_sin_p_s1);
        end
    end

    // ========== 输出赋值（串行写入） ==========
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N; i = i + 1) begin
                matrix_col_p_new[i] <= 0;
                matrix_col_q_new[i] <= 0;
            end
        end else if (valid_s2) begin
            matrix_col_p_new[idx_s2] <= result_p_s2;
            matrix_col_q_new[idx_s2] <= result_q_s2;
        end
    end

endmodule