module Calu_Angle #(
    parameter int N = 4,
    parameter int JACOBI_WIDTH      = 32,
    parameter int DOASEARCH_WIDTH =48
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 calu_angle_start,  
    input  wire [9:0]           azimuth_angle,     

    input  wire signed [JACOBI_WIDTH-1:0]   noise_vector [0:(2*N)*3-1],

    output reg                  calu_angle_done,
    output reg signed [DOASEARCH_WIDTH-1:0]    calu_angle_value
);
    localparam signed [15:0] PI_Q13       = 16'sh6488;  // π (Q3.13)
    localparam signed [15:0] DEG2RAD_Q13  = 16'sd143;   // 1° → Q3.13 rad
    localparam int NUM_VECTORS = 3;  // 噪声向量个数

    // ================== 状态机定义 ==================
    typedef enum logic [2:0] {
        IDLE,
        AZIMUTH_SIN,
        ELEMENT_PHASE,
        COMPLEX_ACCUM,
        NEXT_VECTOR,
        COMPLETE
    } state_t;
    state_t cur_state, next_state;

    // ================== 信号声明 ==================
    // CORDIC1 相关信号 (产生 azimuth 的 sin)
    reg  cordic1_busy;
    reg  signed [15:0]          cordic1_phase;   // 16-bit phase (与CORDIC接口格式一致)
    reg                         cordic1_valid;
    wire                        cordic1_done;
    wire [31:0]                 cordic1_result;  // TODO: 确认 IP 输出位宽/格式
    reg  signed [15:0]          azimuth_sin_val; // Q1.15 假定

    // 阵元CORDIC相关信号
    reg [N-1:0]     element_phase_valid;
    reg signed [17:0] raw_phase[0:N-1];
    reg signed [17:0] norm_phase[0:N-1];
    reg  signed [15:0]          cordic_elem_phase [0:N-1];
    reg                         cordic_elem_valid [0:N-1];
    wire [31:0]                 cordic_elem_result [0:N-1]; // TODO: IP 输出格式确认
    wire signed [15:0]         sin_val[0:N-1];
    wire signed [15:0]         cos_val[0:N-1];

    // 计数器和状态（修正位宽）
    reg [$clog2(N)-1:0]         elem_cnt;         // 0..N-1
    reg [$clog2(NUM_VECTORS)-1:0] vector_cnt;     // 0..NUM_VECTORS-1
    reg [2:0]                   complex_state;    // 扩展到3位以支持更多流水状态
    reg                         complex_accum_done;

    // 数据处理信号
    reg signed [JACOBI_WIDTH -1:0]           nv_real, nv_imag;       
    reg signed [DOASEARCH_WIDTH-1:0]           real_part, imag_part;    // 扩宽，避免乘法溢出
    reg signed [DOASEARCH_WIDTH-1:0]           vector_power;            // 更宽累加
    reg signed [DOASEARCH_WIDTH-1:0]          sum_power;               // 更宽累加总和

    // ===== 新增：流水线中间寄存器 =====
    // real_part 和 imag_part 计算的流水线寄存器
    reg signed [DOASEARCH_WIDTH-1:0]           real_mult1, real_mult2;  // 第一级：乘法结果
    reg signed [DOASEARCH_WIDTH-1:0]           imag_mult1, imag_mult2;  // 第一级：乘法结果
    
    // vector_power 计算的流水线寄存器
    reg signed [DOASEARCH_WIDTH-1:0]           real_square;             // 第一级：real_part^2
    reg signed [DOASEARCH_WIDTH-1:0]           imag_square;             // 第一级：imag_part^2

    // 除法器相关信号
    reg                         div_valid;
    wire                        div_ready;      // m_axis_dout_tvalid
    wire signed [DOASEARCH_WIDTH+16-1:0]          div_result;     // TODO: 用实际 IP 宽度替换
    reg  signed [DOASEARCH_WIDTH-1:0]          div_dividend;
    reg  signed [DOASEARCH_WIDTH-1:0]          div_divisor;

    integer i;

    // ================== 状态机控制 ==================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_state <= IDLE;
        end else begin
            cur_state <= next_state;
        end
    end

    always @(*) begin
        next_state = cur_state;
        case (cur_state)
            IDLE:       if (calu_angle_start) next_state = AZIMUTH_SIN;
            AZIMUTH_SIN: if (cordic1_done) next_state = ELEMENT_PHASE;
            ELEMENT_PHASE:if(&element_phase_valid) next_state = COMPLEX_ACCUM;
            COMPLEX_ACCUM: if (elem_cnt == N-1 && complex_state==3'd5) next_state = NEXT_VECTOR;
            NEXT_VECTOR: if (vector_cnt == NUM_VECTORS-1) next_state = COMPLETE;
                         else next_state = ELEMENT_PHASE; // 回到 ELEMENT_PHASE 重新计算阵元相位并处理下一个向量
            COMPLETE:   next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    // ================== 主流程寄存器更新 ==================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // reset 所有状态
            cordic1_phase <= 16'sd0;
            cordic1_valid <= 1'b0;
            elem_cnt <= 0;
            vector_cnt <= 0;
            complex_state <= 0;
            complex_accum_done <= 1'b0;
            real_part <=0;
            imag_part <=0;
            sum_power <= 0;
            vector_power <= 0;
            calu_angle_value <= 0;
            calu_angle_done <= 0;
            div_valid <= 1'b0;
            div_dividend <= 0;
            div_divisor <= 0;
            cordic1_busy<=0;
            
            // 流水线寄存器初始化
            real_mult1 <= 0;
            real_mult2 <= 0;
            imag_mult1 <= 0;
            imag_mult2 <= 0;
            real_square <= 0;
            imag_square <= 0;
            
            // 初始化数组
            for (i=0; i<N; i=i+1) begin
                cordic_elem_phase[i] <= 16'sd0;
                cordic_elem_valid[i] <= 1'b0;
            end
        end else begin
            // 默认清除本周期的握手信号
            cordic1_valid <= 1'b0;
            for (i=0; i<N; i=i+1) cordic_elem_valid[i] <= 1'b0;
            div_valid <= 1'b0;

            case (cur_state)
                IDLE: begin
                    calu_angle_done <= 1'b0;
                    real_part <=0;
                    imag_part <=0;
                    sum_power <= 0;
                    vector_power <= 0;
                    vector_cnt <= 0;
                    elem_cnt <= 0;
                    complex_state <= 0;
                    complex_accum_done <= 1'b0;
                end

                AZIMUTH_SIN: begin
                    if(!cordic1_busy)begin
                        cordic1_phase <= $signed(azimuth_angle) * DEG2RAD_Q13; // 度 → Q3.13 rad (需要确认相位格式)
                        cordic1_valid <= 1'b1;
                        cordic1_busy<=1;
                    end
                    if (cordic1_done) begin
                        cordic1_busy<=0;
                    end
                end

                ELEMENT_PHASE: begin
 
                    for (i=0; i<N; i=i+1) begin
                        // i * PI_Q13 * sin(azimuth) 
                        raw_phase[i] = ($signed(i) * PI_Q13 * azimuth_sin_val)>>>14;

                        // 相位归一化到 (-π, π)
                        if (raw_phase[i]  > PI_Q13)
                            norm_phase[i]  = raw_phase[i]  - (2*PI_Q13);
                        else if (raw_phase[i]  < -PI_Q13)
                            norm_phase[i]  = raw_phase[i]  + (2*PI_Q13);
                        else
                            norm_phase[i]  = raw_phase[i] ;

                        cordic_elem_phase[i] <= norm_phase[i][15:0] ;  // Q13 -> Q16.14 之类
                        cordic_elem_valid[i] <= 1'b1;
                    end
                    // 进入复数累加前初始化
                    elem_cnt <= 0;
                    complex_state <= 0;
                    vector_power <= 0;
                    complex_accum_done <= 1'b0;
                end

                COMPLEX_ACCUM: begin
                    case (complex_state)
                        3'd0: begin
                            // 读取第 elem_cnt 个复数（实部/虚部分开）
                            nv_real <= noise_vector[vector_cnt * 2 * N + elem_cnt];
                            nv_imag <= noise_vector[vector_cnt * 2 * N + N + elem_cnt];
                            complex_state <= 3'd1;
                        end
                        
                        // ===== real_part 和 imag_part 两级流水 =====
                        3'd1: begin
                            // 第一级流水：执行乘法
                            real_mult1 <= $signed(nv_real) * $signed(cos_val[elem_cnt]);
                            real_mult2 <= $signed(nv_imag) * $signed(sin_val[elem_cnt]);
                            imag_mult1 <= $signed(nv_real) * $signed(sin_val[elem_cnt]);
                            imag_mult2 <= $signed(nv_imag) * $signed(cos_val[elem_cnt]);
                            complex_state <= 3'd2;
                        end
                        
                        3'd2: begin
                            // 第二级流水：执行加减和累加
                            real_part <= real_part + ((real_mult1 - real_mult2) >>> 14);
                            imag_part <= imag_part + ((imag_mult1 + imag_mult2) >>> 14);
                            complex_state <= 3'd3;
                        end
                        
                        3'd3: begin
                            // 判断是否处理完所有阵元
                            if (elem_cnt == N-1) begin
                                complex_accum_done <= 1'b1;
                                complex_state <= 3'd4;  // 进入 vector_power 计算
                            end else begin
                                elem_cnt <= elem_cnt + 1;
                                complex_state <= 3'd0;  // 继续处理下一个阵元
                            end
                        end
                        
                        // ===== vector_power 两级流水 =====
                        3'd4: begin
                            // 第一级流水：执行平方乘法
                            real_square <= real_part * real_part;
                            imag_square <= imag_part * imag_part;
                            complex_state <= 3'd5;
                        end
                        
                        3'd5: begin
                            // 第二级流水：执行加法
                            vector_power <= real_square[DOASEARCH_WIDTH-1:0] + imag_square[DOASEARCH_WIDTH-1:0];
                            complex_state <= 3'd0;  // 完成，准备跳转到 NEXT_VECTOR
                        end
                    endcase
                end

                NEXT_VECTOR: begin
                    // 将当前向量功率累加到总功率并准备处理下一个向量
                    sum_power <= sum_power + vector_power;
                    vector_cnt <= vector_cnt + 1;
                    elem_cnt <= 0;
                    real_part<=0;
                    imag_part<=0;
                    vector_power <= 0;
                    complex_accum_done <= 1'b0;
                    complex_state <= 3'd0;
                end

                COMPLETE: begin
                        calu_angle_value <= sum_power; // 简单截取示例
                        calu_angle_done <= 1'b1;   
                end


            endcase
        end
    end

    // ================ CORDIC / DIV IP 实例化 ================
    // azimuth CORDIC
    cordic_sincos cordic1 (
        .aclk(clk),
        .s_axis_phase_tvalid(cordic1_valid),
        .s_axis_phase_tdata(cordic1_phase),
        .m_axis_dout_tvalid(cordic1_done),
        .m_axis_dout_tdata(cordic1_result)
    );

    // 将 cordic1_result 切分为 sin/cos（请根据 IP 文档确认哪半字表示 sin）
    // 假设 cordic1_result[31:16] = sin (Q1.15)

    assign  azimuth_sin_val = cordic1_result[31:16];

    genvar gi;
    generate
        for (gi = 0; gi < N; gi=gi+1) begin : GEN_CORDIC_ELEM
            cordic_sincos cordic_elem (
                .aclk(clk),
                .s_axis_phase_tvalid(cordic_elem_valid[gi]),
                .s_axis_phase_tdata(cordic_elem_phase[gi]),
                .m_axis_dout_tvalid(element_phase_valid[gi]), // 如果需要 ready 信号，可连线
                .m_axis_dout_tdata(cordic_elem_result[gi])
            );

            // 假设输出高 16 位为 sin，低 16 位为 cos（Q1.15）
            assign sin_val[gi] = cordic_elem_result[gi][31:16];
            assign cos_val[gi] = cordic_elem_result[gi][15:0];
        end
    endgenerate

endmodule