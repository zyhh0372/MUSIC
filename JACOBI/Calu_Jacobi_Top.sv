module Calu_Jacobi_Top #(
    parameter int N           = 4,     // 阵元数
    parameter int ACC_WIDTH   = 20,    // 累加器位宽（内部计算位宽）
    parameter int JACOBI_WIDTH = 32,  // 输出位宽
    parameter int MAX_ITER   = 14      // 最大迭代次数
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // 
    input  wire                       Calu_Jacobi_Start,        
    input  wire signed [ACC_WIDTH-1:0] result_diag   [0:N-1],                 
    input  wire signed [ACC_WIDTH-1:0] result_upper_q[0:(N*(N-1)/2)-1],      
    input  wire signed [ACC_WIDTH-1:0] result_upper_i[0:(N*(N-1)/2)-1],       

    // 
    output reg signed [JACOBI_WIDTH-1:0] eigen_value  [0:N-1],              
    output reg signed [JACOBI_WIDTH-1:0] eigen_vector [0:2*N*N-1],         
    output reg                        jacobi_done,
    output reg                        jacobi_valid
);

    // 
    localparam int FRAC = 15;                       // 小数位数 Q14
    localparam int CORDIC_IN_W = 16;                // cordic 输入宽度 (signed 16-bit Q14)
    localparam int OUTW = CORDIC_IN_W;              // 输出到 cordic 的宽度（16）
    localparam int DIV_W = JACOBI_WIDTH + FRAC;        // 假设除法 IP 输出宽度 = ACC_WIDTH 的整数位 + FRAC 小数位

    // 状态定义
    typedef enum logic [4:0] {
        IDLE         = 5'd0,
        DATA_CACHE   = 5'd1,
        JACOBI_ORDER = 5'd2,
        CORDIC_CALC  = 5'd3,  // 并行计算4个pair的CORDIC
        LEFT_ROT     = 5'd4,  // 并行左旋4个pair
        UPDATE_LEFT  = 5'd5,  // 更新所有4个pair的矩阵行
        RIGHT_ROT    = 5'd6,  // 并行右旋4个pair
        UPDATE_RIGHT = 5'd7,  // 更新所有4个pair的矩阵列
        COMPLETE     = 5'd8   // 完成状态
    } state_t;
    
    state_t current_state, next_state;
    
    // 内部信号定义 - 使用ACC_WIDTH进行内部计算
    reg signed [JACOBI_WIDTH-1:0] matrix [0:2*N-1][0:2*N-1];              
    reg signed [JACOBI_WIDTH-1:0] eigen_vectors [0:2*N-1][0:2*N-1];       
    reg cache_done;
    integer i, j, upper_index;
    
    // 迭代控制
    reg [3:0] step_count;           // 当前步数计数
    
    // 并行配对索引 - 固定4个pair
    reg [4:0] p_idx [0:N-1];
    reg [4:0] q_idx [0:N-1];
    
    // Jacobi配对调度表 - N=4的完整调度
    reg [4:0] jacobi_pair_schedule [0:MAX_ITER-1][0:N-1][0:1]; // [step][pair][p/q]
    
    // 状态计数器
    reg [3:0] jacobi_order_counter;
    reg [3:0] update_left_counter;
    reg [3:0] update_right_counter;
    
    // 并行CORDIC相关信号 - N套并行
    reg signed [JACOBI_WIDTH-1:0] x_int [0:N-1];
    reg signed [JACOBI_WIDTH-1:0] y_int [0:N-1];
    reg signed [31:0] arct_s_axis_cartesian_tdata [0:N-1];
    reg arct_s_axis_cartesian_tvalid [0:N-1];
    wire arct_m_axis_dout_tvalid [0:N-1];
    wire signed [15:0] arct_m_axis_dout_tdata [0:N-1];

    // Division signals - per-instance
    reg signed [JACOBI_WIDTH-1:0] div_dividend[0:N-1];
    reg signed [JACOBI_WIDTH-1:0] div_divisor[0:N-1];

    reg signed [DIV_W-1:0] div_result[0:N-1];
    reg div_ready[0:N-1];       
    reg  div_valid[0:N-1];        
    wire signed [OUTW-1:0] div_q14 [0:N-1];

    reg phase_s_axis_phase_tvalid [0:N-1];
    wire phase_m_axis_dout_tvalid [0:N-1];
    reg signed [15:0] phase_s_axis_phase_tdata [0:N-1];
    wire signed [31:0] phase_m_axis_dout_tdata [0:N-1];
    wire signed [15:0] sin_q [0:N-1];
    wire signed [15:0] cos_q [0:N-1];
    
    // CORDIC计算阶段控制和完成标志
    reg [2:0] cordic_stage [0:N-1];
    reg [N-1:0] cordic_done; 
    wire all_cordic_done;
    
    // 并行旋转相关信号 - N套并行(矩阵A的旋转)
    reg start_left [0:N-1];
    reg start_right [0:N-1];
    wire [N-1:0] done_left ;
    wire [N-1:0] done_right;
    wire all_left_done, all_right_done;
    
    // 特征向量矩阵的旋转信号 - 改为左旋(行旋转)
    reg start_eigen_left [0:N-1];
    wire [N-1:0] done_eigen_left;
    wire all_eigen_left_done;
    
    // 添加:左右旋转完成后的更新完成标志
    reg [N-1:0] left_update_done;
    reg [N-1:0] right_update_done;
    wire all_left_update_done, all_right_update_done;
    
    reg signed [JACOBI_WIDTH-1:0] row_p     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] row_q     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] row_p_new [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] row_q_new [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] col_p     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] col_q     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] col_p_new [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] col_q_new [0:N-1][0:2*N-1];
    
    // --- 修改：用于行存储的特征向量左旋数据结构（替代原来的 eigen_col_*） ---
    reg signed [JACOBI_WIDTH-1:0] eigen_row_p     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] eigen_row_q     [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] eigen_row_p_new [0:N-1][0:2*N-1];
    reg signed [JACOBI_WIDTH-1:0] eigen_row_q_new [0:N-1][0:2*N-1];
    
    // 并行完成信号
    assign all_cordic_done = &cordic_done;
    assign all_left_done = &done_left;
    assign all_right_done = &done_right;
    assign all_eigen_left_done = &done_eigen_left;
    
    // 添加:并行更新完成信号
    assign all_left_update_done = &left_update_done;
    assign all_right_update_done = &right_update_done;
    
    // 初始化Jacobi配对调度表 - N=4的完整配对调度
    initial begin
        // Step 0: (0,1) (2,3) (4,5) (6,7)
        jacobi_pair_schedule[0][0][0] = 0; jacobi_pair_schedule[0][0][1] = 1;
        jacobi_pair_schedule[0][1][0] = 2; jacobi_pair_schedule[0][1][1] = 3;
        jacobi_pair_schedule[0][2][0] = 4; jacobi_pair_schedule[0][2][1] = 5;
        jacobi_pair_schedule[0][3][0] = 6; jacobi_pair_schedule[0][3][1] = 7;

        // Step 1: (0,2) (1,3) (4,6) (5,7)
        jacobi_pair_schedule[1][0][0] = 0; jacobi_pair_schedule[1][0][1] = 2;
        jacobi_pair_schedule[1][1][0] = 1; jacobi_pair_schedule[1][1][1] = 3;
        jacobi_pair_schedule[1][2][0] = 4; jacobi_pair_schedule[1][2][1] = 6;
        jacobi_pair_schedule[1][3][0] = 5; jacobi_pair_schedule[1][3][1] = 7;

        // Step 2: (0,3) (1,2) (4,7) (5,6)
        jacobi_pair_schedule[2][0][0] = 0; jacobi_pair_schedule[2][0][1] = 3;
        jacobi_pair_schedule[2][1][0] = 1; jacobi_pair_schedule[2][1][1] = 2;
        jacobi_pair_schedule[2][2][0] = 4; jacobi_pair_schedule[2][2][1] = 7;
        jacobi_pair_schedule[2][3][0] = 5; jacobi_pair_schedule[2][3][1] = 6;

        // Step 3: (0,4) (1,5) (2,6) (3,7)
        jacobi_pair_schedule[3][0][0] = 0; jacobi_pair_schedule[3][0][1] = 4;
        jacobi_pair_schedule[3][1][0] = 1; jacobi_pair_schedule[3][1][1] = 5;
        jacobi_pair_schedule[3][2][0] = 2; jacobi_pair_schedule[3][2][1] = 6;
        jacobi_pair_schedule[3][3][0] = 3; jacobi_pair_schedule[3][3][1] = 7;

        // Step 4: (0,5) (1,4) (2,7) (3,6)
        jacobi_pair_schedule[4][0][0] = 0; jacobi_pair_schedule[4][0][1] = 5;
        jacobi_pair_schedule[4][1][0] = 1; jacobi_pair_schedule[4][1][1] = 4;
        jacobi_pair_schedule[4][2][0] = 2; jacobi_pair_schedule[4][2][1] = 7;
        jacobi_pair_schedule[4][3][0] = 3; jacobi_pair_schedule[4][3][1] = 6;

        // Step 5: (0,6) (1,7) (2,4) (3,5)
        jacobi_pair_schedule[5][0][0] = 0; jacobi_pair_schedule[5][0][1] = 6;
        jacobi_pair_schedule[5][1][0] = 1; jacobi_pair_schedule[5][1][1] = 7;
        jacobi_pair_schedule[5][2][0] = 2; jacobi_pair_schedule[5][2][1] = 4;
        jacobi_pair_schedule[5][3][0] = 3; jacobi_pair_schedule[5][3][1] = 5;

        // Step 6: (0,7) (1,6) (2,5) (3,4)
        jacobi_pair_schedule[6][0][0] = 0; jacobi_pair_schedule[6][0][1] = 7;
        jacobi_pair_schedule[6][1][0] = 1; jacobi_pair_schedule[6][1][1] = 6;
        jacobi_pair_schedule[6][2][0] = 2; jacobi_pair_schedule[6][2][1] = 5;
        jacobi_pair_schedule[6][3][0] = 3; jacobi_pair_schedule[6][3][1] = 4;

        // Step 7~13:重复 0~6
        jacobi_pair_schedule[7]  = jacobi_pair_schedule[0];
        jacobi_pair_schedule[8]  = jacobi_pair_schedule[1];
        jacobi_pair_schedule[9]  = jacobi_pair_schedule[2];
        jacobi_pair_schedule[10] = jacobi_pair_schedule[3];
        jacobi_pair_schedule[11] = jacobi_pair_schedule[4];
        jacobi_pair_schedule[12] = jacobi_pair_schedule[5];
        jacobi_pair_schedule[13] = jacobi_pair_schedule[6];
    end


    // 状态机主逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    // 状态转移逻辑 (保持不变)
    always @(*) begin
        next_state=current_state;
        case (current_state)
            IDLE:        if (Calu_Jacobi_Start) next_state = DATA_CACHE;
            DATA_CACHE:  if (cache_done) next_state = JACOBI_ORDER;

            JACOBI_ORDER: if (jacobi_order_counter >= 2) next_state = CORDIC_CALC;

            CORDIC_CALC: if (all_cordic_done) next_state = LEFT_ROT;  // 等待所有CORDIC完成

            LEFT_ROT:    if (all_left_done&& all_eigen_left_done) next_state = UPDATE_LEFT; // 等待所有左旋完成


            UPDATE_LEFT: if (all_left_update_done ) next_state = RIGHT_ROT;


            RIGHT_ROT:   if (all_right_done ) next_state = UPDATE_RIGHT;


            UPDATE_RIGHT: if ( all_right_update_done) begin
                if (step_count >= MAX_ITER) 
                    next_state = COMPLETE;
                else 
                    next_state = JACOBI_ORDER;
            end
            COMPLETE:    next_state = IDLE;
        endcase
    end

    // 状态机控制逻辑（大体保持不变，内部加入 div_valid 初始化等）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cache_done <= 0;
            jacobi_done <= 0;
            jacobi_valid <= 0;
            step_count <= 0;
            
            // 初始化并行配对索引和控制信号
            for (i = 0; i < N; i = i + 1) begin
                p_idx[i] <= 0;
                q_idx[i] <= 0;
                cordic_stage[i] <= 0;
                cordic_done[i] <= 0;
                arct_s_axis_cartesian_tvalid[i] <= 0;
                phase_s_axis_phase_tvalid[i] <= 0;
                start_left[i] <= 0;
                start_right[i] <= 0;
                start_eigen_left[i] <= 0;
                // 添加:初始化更新完成标志
                left_update_done[i] <= 0;
                right_update_done[i] <= 0;
                // 初始化除法握手
                div_valid[i] <= 0;
            end
            
            // 状态计数器初始化
            jacobi_order_counter <= 0;
            update_left_counter <= 0;
            update_right_counter <= 0;
            
            // 初始化矩阵
            for (i = 0; i < 2*N; i = i + 1) begin
                for (j = 0; j < 2*N; j = j + 1) begin
                    matrix[i][j] <= 0;
                    eigen_vectors[i][j] <= (i==j) ? 1000  : 0;
                end
            end
        end else begin
            cache_done <= 0;
            // 默认清零脉冲信号
            for (i = 0; i < N; i = i + 1) begin
                arct_s_axis_cartesian_tvalid[i] <= 0;
                phase_s_axis_phase_tvalid[i] <= 0;
                start_left[i] <= 0;
                start_right[i] <= 0;
                start_eigen_left[i] <= 0;
                cordic_done[i] <= 0;  
                left_update_done[i] <= 0;
                right_update_done[i] <= 0;
                div_valid[i] <= 0;
            end

            case (current_state)
                IDLE: begin
                    jacobi_done <= 0;
                    jacobi_valid <= 0;
                    step_count <= 0;
                    jacobi_order_counter <= 0;
                    update_left_counter <= 0;
                    update_right_counter <= 0;
                    cache_done <= 0;
                    for (i = 0; i < N; i = i + 1) begin
                        cordic_stage[i] <= 0;
                        cordic_done[i] <= 0;
                        left_update_done[i] <= 0;
                        right_update_done[i] <= 0;
                        div_valid[i] <= 0;
                    end
                    for (i = 0; i < 2*N; i = i + 1) begin
                        for (j = 0; j < 2*N; j = j + 1) begin
                            eigen_vectors[i][j] <= (i==j) ? 1000  : 0;
                        end
                    end
                end
                
                DATA_CACHE: begin
                    upper_index = 0;
                    // 全存储矩阵 - 存储完整的复数Hermitian矩阵
                    for (i = 0; i < N; i = i + 1) begin
                        // 对角元素(实数)
                        matrix[i][i]       <= result_diag[i];
                        matrix[i+N][i+N]   <= result_diag[i];
                        matrix[i][i+N]     <= 0;
                        matrix[i+N][i]     <= 0;
                        
                        for (j = i + 1; j < N; j = j + 1) begin
                            // 上三角元素
                            matrix[i][j]     <= result_upper_q[upper_index];      // 实部
                            matrix[i+N][j+N] <= result_upper_q[upper_index];      // 实部
                            matrix[i][j+N]   <= -result_upper_i[upper_index];     // -虚部
                            matrix[i+N][j]   <= result_upper_i[upper_index];      // 虚部
                            
                            // 下三角元素(Hermitian对称)
                            matrix[j][i]     <= result_upper_q[upper_index];      // 共轭实部
                            matrix[j+N][i+N] <= result_upper_q[upper_index];      // 共轭实部
                            matrix[j][i+N]   <= result_upper_i[upper_index];      // 共轭虚部
                            matrix[j+N][i]   <= -result_upper_i[upper_index];     // -共轭虚部
                            
                            upper_index = upper_index + 1;
                        end
                    end
                    cache_done <= 1;
                end
                
                JACOBI_ORDER: begin
                    if (jacobi_order_counter == 0) begin
                        // 一次性获取N个并行配对
                        for (i = 0; i < N; i = i + 1) begin
                            p_idx[i] <= jacobi_pair_schedule[step_count][i][0];
                            q_idx[i] <= jacobi_pair_schedule[step_count][i][1];
                        end
                    end
                    
                    if (jacobi_order_counter == 2) begin
                        // 更新步数计数
                        step_count <= step_count + 1;
                        jacobi_order_counter<=0;
                    end else begin
                        jacobi_order_counter <= jacobi_order_counter + 1;
                    end
                end
                
                 CORDIC_CALC: begin
                    // 在进入 CORDIC_CALC 时先清除 cordic_done 和阶段
                    // （这样可以确保上次迭代不影响本次）
                    for (i = 0; i < N; i = i + 1) begin
                        cordic_done[i] <= 0;
                    end

                    // 并行处理 N 个 pair 的 CORDIC 计算
                    for (i = 0; i < N; i = i + 1) begin
                        if (!cordic_done[i]) begin
                            case (cordic_stage[i])
                                3'd0: begin  // 第一阶段：计算基本参数
                                    x_int[i] <= matrix[p_idx[i]][p_idx[i]] - matrix[q_idx[i]][q_idx[i]];
                                    y_int[i] <= matrix[p_idx[i]][q_idx[i]] <<< 1; // 2* a_pq
                                    cordic_stage[i] <= 3'd1;
                                end
                                
                               3'd1: begin  // 第二阶段：缩放计算和格式转换 (发起除法请求)
                                    // 使用自定义函数 abs_val 来计算绝对值
                                    if ( abs_val(x_int[i]) >= abs_val(y_int[i]) ) begin
                                        // x 较大 -> 把较小者作为 dividend，较大者作为 divisor
                                        div_dividend[i] <= y_int[i];       // 注意：这里用原始值，保留符号
                                        div_divisor[i]  <= abs_val(x_int[i]); // 除数使用绝对值
                                        div_valid[i] <= 1'b1; // 请求除法
                                        cordic_stage[i] <= 3'd2;
                                    end else begin
                                        // y 较大
                                        div_dividend[i] <= x_int[i];       // 注意：这里用原始值，保留符号
                                        div_divisor[i]  <= abs_val(y_int[i]); // 除数使用绝对值
                                        div_valid[i] <= 1'b1; // 请求除法
                                        cordic_stage[i] <= 3'd2;
                                    end
                                end
                                3'd2: begin
                                    if (div_ready[i]) begin
                                        // 根据 x、y 符号确定 ±1（Q14 格式）
                                        logic signed [15:0] sign_x_q14, sign_y_q14;
                                        sign_x_q14 = x_int[i][JACOBI_WIDTH-1] ? -($signed({{(JACOBI_WIDTH-16){1'b0}}, 16'sd16384})) 
                                                                            :  $signed({{(JACOBI_WIDTH-16){1'b0}}, 16'sd16384});

                                        sign_y_q14 = y_int[i][JACOBI_WIDTH-1] ? -($signed({{(JACOBI_WIDTH-16){1'b0}}, 16'sd16384})) 
                                                                            :  $signed({{(JACOBI_WIDTH-16){1'b0}}, 16'sd16384});


                                        // 比较绝对值，决定哪一轴作为 1.0
                                        if ( (x_int[i][JACOBI_WIDTH-1] ? -x_int[i] : x_int[i]) >= 
                                            (y_int[i][JACOBI_WIDTH-1] ? -y_int[i] : y_int[i]) ) begin
                                            // x 绝对值更大
                                            arct_s_axis_cartesian_tdata[i]  <= { div_q14[i],sign_x_q14};
                                            arct_s_axis_cartesian_tvalid[i] <= 1'b1;  
                                        end else begin
                                            // y 绝对值更大
                                            arct_s_axis_cartesian_tdata[i]  <= {sign_y_q14,div_q14[i] };
                                            arct_s_axis_cartesian_tvalid[i] <= 1'b1;               
                                        end

                                        div_valid[i]   <= 1'b0; // 清除请求
                                        cordic_stage[i] <= 3'd3;

                                    end else begin
                                        // 等待 div_ready 拉高
                                        div_valid[i]   <= 1'b1;
                                        cordic_stage[i] <= 3'd2;
                                    end
                                end

                                
                                3'd3: begin  // 第三阶段：等待CORDIC结果 (arctan)
                                    if (arct_m_axis_dout_tvalid[i]) begin
                                        // 把 arctan 输出 (16-bit) 左移 1 位供 sincos 使用（与原逻辑保持） 
                                        phase_s_axis_phase_tdata[i] <= (arct_m_axis_dout_tdata[i] >>> 1);
                                        phase_s_axis_phase_tvalid[i] <= 1;
                                        cordic_stage[i] <= 3'd4;
                                    end
                                end
                                
                                3'd4: begin  // 第四阶段：等待相位 sincos 计算完成
                                    if (phase_m_axis_dout_tvalid[i]) begin
                                        cordic_done[i] <= 1;  // 标记该pair完成
                                        cordic_stage[i] <= 0; // 重置计算阶段
                                    end
                                end
                            endcase
                        end
                    end
                end
                
                LEFT_ROT: begin
                    // 并行启动N个pair的左旋转 (矩阵行旋转)
                    for (i = 0; i < N; i = i + 1) begin
                        if (!start_left[i] && !done_left[i]) begin
                            for (j = 0; j < 2*N; j = j + 1) begin
                                row_p[i][j] <= matrix[p_idx[i]][j];
                                row_q[i][j] <= matrix[q_idx[i]][j];
                            end
                            start_left[i] <= 1;
                        end
                        if (!start_eigen_left[i] && !done_eigen_left[i]) begin
                            for (j = 0; j < 2*N; j = j + 1) begin
                                eigen_row_p[i][j] <= eigen_vectors[p_idx[i]][j];
                                eigen_row_q[i][j] <= eigen_vectors[q_idx[i]][j];
                            end
                            start_eigen_left[i] <= 1;
                        end
                    end
                end
                
                UPDATE_LEFT: begin
                    if(step_count==1) begin
                        for (i = 0; i < N; i = i + 1) begin
                            // 更新矩阵A
                            for (j = 0; j < 2*N; j = j + 1) begin
                                matrix[p_idx[i]][j] <= row_p_new[i][j];
                                matrix[q_idx[i]][j] <= row_q_new[i][j];
                                // 更新特征向量矩阵 (行更新)
                                eigen_vectors[p_idx[i]][j] <= eigen_row_p_new[i][j];
                                eigen_vectors[q_idx[i]][j] <= eigen_row_q_new[i][j];
                                if (update_left_counter >= 1) begin  // 确保有足够的时钟周期进行更新
                                    left_update_done[i] <= 1; // 标记该pair的更新完成
                                end
                            end    
                        end
                    end else begin
                        for (i = 0; i < N; i = i + 1) begin
                            // 更新矩阵A
                            for (j = 0; j < 2*N; j = j + 1) begin
                                matrix[p_idx[i]][j] <= row_p_new[i][j]>>>14;
                                matrix[q_idx[i]][j] <= row_q_new[i][j]>>>14;
                                // 更新特征向量矩阵 (行更新)
                                eigen_vectors[p_idx[i]][j] <= eigen_row_p_new[i][j]>>>14;
                                eigen_vectors[q_idx[i]][j] <= eigen_row_q_new[i][j]>>>14;
                                if (update_left_counter >= 1) begin  // 确保有足够的时钟周期进行更新
                                    left_update_done[i] <= 1; // 标记该pair的更新完成
                                end
                            end 
                            
                         end
                    end
                    update_left_counter <= update_left_counter + 1;
                    
                end
                
                RIGHT_ROT: begin
                    // 并行启动N个pair的右旋转(矩阵A) - 列旋转
                    for (i = 0; i < N; i = i + 1) begin
                        if (!start_right[i] && !done_right[i]) begin
                            // 准备列旋转数据
                            for (j = 0; j < 2*N; j = j + 1) begin
                                col_p[i][j] <= matrix[j][p_idx[i]];
                                col_q[i][j] <= matrix[j][q_idx[i]];
                            end
                            start_right[i] <= 1;
                        end
                        end
                    end
                
                UPDATE_RIGHT: begin
                    for (i = 0; i < N; i = i + 1) begin
                        // 更新矩阵A
                        for (j = 0; j < 2*N; j = j + 1) begin
                            matrix[j][p_idx[i]] <= col_p_new[i][j]>>>14;
                            matrix[j][q_idx[i]] <= col_q_new[i][j]>>>14;
                        end    
                        if (update_right_counter >= 1) begin  // 确保有足够的时钟周期进行更新
                            right_update_done[i] <= 1; // 标记该pair的更新完成
                        end
                    end                     
                    update_right_counter <= update_right_counter + 1;
                    if (update_right_counter == 1) begin
                        // 重置计数器,准备下一轮迭代
                        jacobi_order_counter <= 0;
                        update_left_counter <= 0;
                        update_right_counter <= 0;
                    end 

                end
                
                COMPLETE: begin
                    // 修改：输出时进行位宽转换
                    // 只输出偶数索引的特征值(索引0,2,4,6对应原始复矩阵的特征值)
                    for (i = 0; i < N; i = i + 1) begin
                        eigen_value[i] <= matrix[2*i][2*i];
                    end
                    
                    // 修改：输出时进行位宽转换
                    // 只输出偶数列的特征向量(每列2N维，共N列)
                    for (i = 0; i < N; i = i + 1) begin
                        for (j = 0; j < N; j = j + 1) begin
                            eigen_vector[i*N*2+j] <= eigen_vectors[2*i][j] ;
                            eigen_vector[i*N*2+N+j] <= eigen_vectors[2*i][N+j] ;
                        end
                    end
                    
                    jacobi_done <= 1;
                    jacobi_valid <= 1;
                end
            endcase
        end
    end

    // 并行CORDIC IP核信号连接和实例化
    genvar k;
    generate
        for (k = 0; k < N; k = k + 1) begin : cordic_gen
            assign sin_q[k] = phase_m_axis_dout_tdata[k][31:16];
            assign cos_q[k] = phase_m_axis_dout_tdata[k][15:0];

            cordic_arctan cordic_cartesian_inst (
                .aclk(clk),
                .s_axis_cartesian_tvalid(arct_s_axis_cartesian_tvalid[k]),
                .s_axis_cartesian_tdata(arct_s_axis_cartesian_tdata[k]),
                .m_axis_dout_tvalid(arct_m_axis_dout_tvalid[k]),
                .m_axis_dout_tdata(arct_m_axis_dout_tdata[k])
            );

            cordic_sincos cordic_phase_inst (
                .aclk(clk),
                .s_axis_phase_tvalid(phase_s_axis_phase_tvalid[k]),
                .s_axis_phase_tdata(phase_s_axis_phase_tdata[k]),
                .m_axis_dout_tvalid(phase_m_axis_dout_tvalid[k]),
                .m_axis_dout_tdata(phase_m_axis_dout_tdata[k])
            );
            div_gen_Jacobi divider_inst_0 (
                .aclk(clk),
                .s_axis_dividend_tvalid(div_valid[k]), 
                .s_axis_dividend_tdata(div_dividend[k]),
                .s_axis_divisor_tvalid(div_valid[k]),
                .s_axis_divisor_tdata(div_divisor[k]),
                .m_axis_dout_tvalid(div_ready[k]),
                .m_axis_dout_tdata(div_result[k])
            );
            assign div_q14[k] = {div_result[k][14], div_result[k][14:0]};  // 16-bit 有符号 Q14

            // 并行旋转模块实例化 - 矩阵A的左旋转
            Left_Rotation #(.ACC_WIDTH(JACOBI_WIDTH), .N(2*N)) u_left (
                .clk(clk), .rst_n(rst_n),
                .start(start_left[k]),
                .matrix_row_p(row_p[k]), .matrix_row_q(row_q[k]),
                .sin_theta(sin_q[k]), .cos_theta(cos_q[k]),
                .done(done_left[k]),
                .matrix_row_p_new(row_p_new[k]), .matrix_row_q_new(row_q_new[k])
            );

            // 并行旋转模块实例化 - 矩阵A的右旋转
            Right_Rotation #(.ACC_WIDTH(JACOBI_WIDTH), .N(2*N)) u_right (
                .clk(clk), .rst_n(rst_n),
                .start(start_right[k]),
                .matrix_col_p(col_p[k]), .matrix_col_q(col_q[k]),
                .sin_theta(sin_q[k]), .cos_theta(cos_q[k]),
                .done(done_right[k]),
                .matrix_col_p_new(col_p_new[k]), .matrix_col_q_new(col_q_new[k])
            );
            
            Left_Rotation_Vector #(.ACC_WIDTH(JACOBI_WIDTH), .N(2*N)) u_eigen_left (
                .clk(clk), .rst_n(rst_n),
                .start(start_eigen_left[k]),
                .matrix_row_p(eigen_row_p[k]), .matrix_row_q(eigen_row_q[k]),
                .sin_theta(sin_q[k]), .cos_theta(cos_q[k]),
                .done(done_eigen_left[k]),
                .matrix_row_p_new(eigen_row_p_new[k]), .matrix_row_q_new(eigen_row_q_new[k])
            );
        end
    endgenerate

    // Function to calculate absolute value of a signed number
    function automatic logic signed [JACOBI_WIDTH-1:0] abs_val;
        input logic signed [JACOBI_WIDTH-1:0] data_in;
        begin
            // 如果输入是负数（符号位为1），则返回其相反数
            // 否则，直接返回原数
            abs_val = (data_in[JACOBI_WIDTH-1] == 1'b1) ? (~data_in + 1'b1) : data_in;
        end
    endfunction


endmodule
