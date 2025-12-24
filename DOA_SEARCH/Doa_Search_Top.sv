module DOA_JUDGE_Top #(
    parameter int JACOBI_WIDTH = 32,
    parameter int DOASEARCH_WIDTH = 48,
    parameter int N = 4,
    parameter int COARSE_SEARCH_STEP = 20,
    parameter int FINE_SEARCH_STEP = 2,
    parameter int LOCAL_MIN_DEPTH = 16
)(
    input  wire                          iclk,
    input  wire                          irst_n,
    input  wire                          istart_doa_search,

    input  wire signed [JACOBI_WIDTH-1:0]   ieigen_vector [0:2*N*N-1],
    input  wire signed [JACOBI_WIDTH-1:0]   ieigen_value  [0:N-1],

    output reg  [9:0]                   oazimuth_angle,
    output reg                           odoa_search_done
);

    localparam  IDLE            = 3'd0,
                FIND_NOISE      = 3'd1,
                COARSE_SEARCH   = 3'd2,
                DO_FINE_LOOP    = 3'd3,
                DOA_JUDGE       = 3'd4,
                DOA_SEARCH_DONE = 3'd5;  // 修正：原代码为 "o = 3'd5"

    // ======== 状态寄存器 ========
    reg [2:0] current_state, next_state;

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    // ======== 状态跳转逻辑 ========
    always @(*) begin
        case (current_state)
            IDLE:           next_state = istart_doa_search ? FIND_NOISE : IDLE;  // 修正：原代码为 "i"
            FIND_NOISE:     next_state = find_noise_done ? COARSE_SEARCH : FIND_NOISE;
            COARSE_SEARCH:  next_state = coarse_search_done ? DO_FINE_LOOP : COARSE_SEARCH;
            DO_FINE_LOOP:   next_state = (fine_idx >= coarse_local_min_count) ? DOA_JUDGE : DO_FINE_LOOP;
            DOA_JUDGE:      next_state = judge_done ? DOA_SEARCH_DONE : DOA_JUDGE;
            DOA_SEARCH_DONE: next_state = IDLE;
            default:        next_state = IDLE;
        endcase
    end

    integer i, j, k;

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            min_val   <= {1'b0, {(DOASEARCH_WIDTH-1){1'b1}}};  // 重置为最大正数
            min_angle <= 16'd0;
            judge_done <= 1'b0;
        end else if (current_state == FIND_NOISE) begin
            min_val   <= {1'b0, {(DOASEARCH_WIDTH-1){1'b1}}};  // 重置为最大正数
            min_angle <= 16'd0;
            judge_done <= 1'b0;
        end
    end


    // ================== 寻找噪声子空间 ==================
    reg find_noise_done;
    reg signed [JACOBI_WIDTH-1:0] noise_vector [0:2*N*(N-1)-1];
    reg [$clog2(N)-1:0] max_eigen_idx;
    reg signed [JACOBI_WIDTH-1:0] max_eigen_val;

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            find_noise_done <= 1'b0;
            max_eigen_idx <= 0;
            max_eigen_val <= 0;
            for (i = 0; i < 2*N*(N-1); i = i + 1)
                noise_vector[i] <= '0;
        end else if (current_state == FIND_NOISE) begin
            // 找最大特征值（用于排除信号子空间）
            max_eigen_val = $signed({1'b1, {(JACOBI_WIDTH-1){1'b0}}});  // 最小负数
            max_eigen_idx = 0;
            for (i = 0; i < N; i = i + 1) begin
                if (ieigen_value[i] > max_eigen_val) begin
                    max_eigen_val = ieigen_value[i];
                    max_eigen_idx = i;
                end
            end

            // 提取噪声子空间向量（排除最大特征值对应的向量）
            k = 0;
            for (j = 0; j < N; j = j + 1) begin
                if (j != max_eigen_idx) begin
                    for (i = 0; i < 2*N; i = i + 1) begin
                        noise_vector[k*2*N + i] <= ieigen_vector[j*2*N + i];  // 修正：原代码为 "i[j*2*N + i]"
                    end
                    k = k + 1;
                end
            end

            find_noise_done <= 1'b1;
        end else begin
            find_noise_done <= 1'b0;
        end
    end


    // ================== 粗搜索模块 ==================
    reg coarse_search_start;
    reg coarse_search_busy;
    wire coarse_search_done;
    wire [9:0] coarse_local_min_angle [0:LOCAL_MIN_DEPTH-1];
    wire [$clog2(LOCAL_MIN_DEPTH)-1:0] coarse_local_min_count;

    Doa_Coarse_Search #(
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .DOASEARCH_WIDTH(DOASEARCH_WIDTH),
        .N(N),
        .LOCAL_MIN_DEPTH(LOCAL_MIN_DEPTH),
        .STEP(COARSE_SEARCH_STEP)
    ) coarse_search (
        .clk(iclk),
        .rst_n(irst_n),
        .start_search(coarse_search_start),
        .noise_vector(noise_vector),
        .azimuth_angle_max(10'd90),
        .azimuth_angle_min(10'd0),
        .local_min_angle(coarse_local_min_angle),
        .local_min_count(coarse_local_min_count),
        .search_done(coarse_search_done)
    );

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            coarse_search_start <= 1'b0;
            coarse_search_busy  <= 1'b0;
        end else begin
            coarse_search_start <= 1'b0;
            coarse_search_busy <= 1'b0;
            if (current_state == COARSE_SEARCH) begin
                if (!coarse_search_busy) begin
                    coarse_search_start <= 1'b1;
                    coarse_search_busy  <= 1'b1;
                end
            end else if (coarse_search_done) begin
                coarse_search_busy <= 1'b0;
            end
        end
    end


    // ================== 精搜索模块 ==================
    reg fine_search_start;
    wire fine_search_done;
    reg [9:0] fine_azimuth_min, fine_azimuth_max;
    wire signed [DOASEARCH_WIDTH-1:0] fine_local_min [0:LOCAL_MIN_DEPTH-1];
    wire [9:0] fine_local_min_angle [0:LOCAL_MIN_DEPTH-1];
    wire [$clog2(LOCAL_MIN_DEPTH)-1:0] fine_local_min_count;

    reg signed [DOASEARCH_WIDTH-1:0] fine_all_min [0:LOCAL_MIN_DEPTH-1];
    reg [9:0] fine_all_angle [0:LOCAL_MIN_DEPTH-1];
    reg [$clog2(LOCAL_MIN_DEPTH)-1:0] fine_all_count;
    reg [$clog2(LOCAL_MIN_DEPTH)-1:0] fine_idx;
    reg fine_search_busy;

    Doa_Fine_Search #(
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .DOASEARCH_WIDTH(DOASEARCH_WIDTH),
        .N(N),
        .LOCAL_MIN_DEPTH(LOCAL_MIN_DEPTH),
        .STEP(FINE_SEARCH_STEP)
    ) fine_search (
        .clk(iclk),
        .rst_n(irst_n),
        .start_search(fine_search_start),
        .noise_vector(noise_vector),
        .azimuth_angle_max(fine_azimuth_max),
        .azimuth_angle_min(fine_azimuth_min),
        .local_min(fine_local_min),
        .local_min_angle(fine_local_min_angle),
        .local_min_count(fine_local_min_count),
        .search_done(fine_search_done)
    );

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            fine_search_start <= 1'b0;
            fine_search_busy  <= 1'b0;
        end else begin
            fine_search_start <= 1'b0;
            fine_search_busy  <= 1'b0;
            if (current_state == DO_FINE_LOOP) begin
                if (!fine_search_busy) begin
                    fine_search_start <= 1'b1;
                    fine_search_busy  <= 1'b1;
                end else if (fine_search_done) begin
                    fine_search_busy <= 1'b0;
                end
            end
        end
    end


    // ================== 精搜索控制逻辑 ==================
    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            fine_idx         <= 0;
            fine_all_count   <= 0;
            fine_azimuth_min <= 16'd0;
            fine_azimuth_max <= 16'd0;
            for (i = 0; i < LOCAL_MIN_DEPTH; i = i + 1) begin
                fine_all_min[i]   <= '0;
                fine_all_angle[i] <= 16'd0;
            end
        end else if (current_state == COARSE_SEARCH) begin
            // 在粗搜索阶段初始化精搜索相关变量
            fine_idx       <= 0;
            fine_all_count <= 0;
        end else if (current_state == DO_FINE_LOOP) begin
            // 设置精搜索范围
            if (fine_idx < coarse_local_min_count && !fine_search_busy && !fine_search_start) begin
                if (coarse_local_min_angle[fine_idx] >= COARSE_SEARCH_STEP)
                    fine_azimuth_min <= coarse_local_min_angle[fine_idx] - COARSE_SEARCH_STEP;
                else
                    fine_azimuth_min <= 16'd0;

                if (coarse_local_min_angle[fine_idx] + COARSE_SEARCH_STEP <= 16'd90)
                    fine_azimuth_max <= coarse_local_min_angle[fine_idx] + COARSE_SEARCH_STEP;
                else
                    fine_azimuth_max <= 16'd90;
            end

            // 收集精搜索结果
            if (fine_search_done) begin
                for (j = 0; j < fine_local_min_count && (fine_all_count + j) < LOCAL_MIN_DEPTH; j = j + 1) begin
                    fine_all_min[fine_all_count + j]   <= fine_local_min[j];
                    fine_all_angle[fine_all_count + j] <= fine_local_min_angle[j];
                end
                fine_all_count <= fine_all_count + fine_local_min_count;
                fine_idx <= fine_idx + 1;
            end
        end
    end


    // ================== DOA_JUDGE: 找最小值（MUSIC伪谱的最小值对应DOA角度） ==================
    reg signed [DOASEARCH_WIDTH-1:0] min_val;
    reg [9:0] min_angle;
    reg judge_done;

    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            min_val    <= {1'b0, {(DOASEARCH_WIDTH-1){1'b1}}};  // 初始化为最大正数
            min_angle  <= 16'd0;
            judge_done <= 1'b0;
        end else if (current_state == DOA_JUDGE) begin
            for (i = 0; i < fine_all_count; i = i + 1) begin
                if (fine_all_min[i] < min_val) begin
                    min_val   <= fine_all_min[i];
                    min_angle <= fine_all_angle[i];
                end
            end
            judge_done <= 1'b1;
        end else begin
            judge_done <= 1'b0;
        end
    end


    // ================== 输出阶段 ==================
    always @(posedge iclk or negedge irst_n) begin  // 修正：原代码为 "clk" 和 "rst_n"
        if (!irst_n) begin
            oazimuth_angle   <= 16'd0;
            odoa_search_done <= 1'b0;
        end else if (current_state == DOA_SEARCH_DONE) begin
            oazimuth_angle   <= min_angle;
            odoa_search_done <= 1'b1;
        end else begin
            odoa_search_done <= 1'b0;
        end
    end

endmodule