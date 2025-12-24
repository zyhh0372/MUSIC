// ====================== Doa_Fine_Search 精搜索模块 ======================
module Doa_Fine_Search #(
    parameter int JACOBI_WIDTH      = 32,
    parameter int DOASEARCH_WIDTH   = 48,
    parameter int N                 = 4,
    parameter int LOCAL_MIN_DEPTH   = 16,
    parameter int STEP              = 5
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             start_search,
    input  wire signed [JACOBI_WIDTH-1:0]   noise_vector [0:(2*N)*3-1],
    input  wire [9:0]                      azimuth_angle_max,
    input  wire [9:0]                      azimuth_angle_min,
    output reg signed [DOASEARCH_WIDTH-1:0] local_min [0:LOCAL_MIN_DEPTH-1],
    output reg [9:0]                       local_min_angle [0:LOCAL_MIN_DEPTH-1],
    output reg [$clog2(LOCAL_MIN_DEPTH)-1:0] local_min_count,
    output reg                              search_done
);

    typedef enum logic [2:0] {
        IDLE           = 3'd0,
        INIT_WINDOW    = 3'd1,
        FIND_LOCAL_MIN = 3'd2,
        SLIDE_WINDOW   = 3'd3,
        UPDATE_PTR     = 3'd4,
        DONE           = 3'd5
    } state_t;

    state_t current_state, next_state;

    // pointers & buffers
    reg [9:0] azimuth_ptr;
    reg signed [DOASEARCH_WIDTH-1:0] window_cache [0:2];
    reg [1:0] init_counter;
    reg waiting;
    reg calu_start;
    wire calu_valid;
    wire signed [DOASEARCH_WIDTH-1:0] calced_angle;
    reg [9:0] calu_azimuth;

    integer i;

    // instantiate calu
    Calu_Angle #(
        .N(N),
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .DOASEARCH_WIDTH(DOASEARCH_WIDTH)
    ) calu_inst (
        .clk(clk),
        .rst_n(rst_n),
        .calu_angle_start(calu_start),
        .azimuth_angle(calu_azimuth),
        .noise_vector(noise_vector),
        .calu_angle_value(calced_angle),
        .calu_angle_done(calu_valid)
    );

    // next state logic
    always @(*) begin
        case (current_state)
            IDLE:           next_state = start_search ? INIT_WINDOW : IDLE;
            INIT_WINDOW:    next_state = (init_counter == 2'd3 && !waiting) ? FIND_LOCAL_MIN : INIT_WINDOW;
            FIND_LOCAL_MIN: next_state = UPDATE_PTR;
            SLIDE_WINDOW:   next_state = calu_valid ? FIND_LOCAL_MIN : SLIDE_WINDOW;
            UPDATE_PTR:     next_state = (azimuth_ptr >= (azimuth_angle_max - 2*STEP)) ? DONE : SLIDE_WINDOW;
            DONE:           next_state = IDLE;
            default:        next_state = IDLE;
        endcase
    end

    // state register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // main behavior
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            azimuth_ptr <= 16'd0;
            init_counter <= 2'd0;
            waiting <= 1'b0;
            calu_start <= 1'b0;
            calu_azimuth <= 9'd0;

            local_min_count <= '0;
            search_done <= 1'b0;
            for (i = 0; i < 3; i = i + 1) window_cache[i] <= '0;
            for (i = 0; i < LOCAL_MIN_DEPTH; i = i + 1) begin
                local_min[i] <= '0;
                local_min_angle[i] <= 16'd0;
            end
        end else begin
            // defaults
            calu_start <= 1'b0;
            search_done <= 1'b0;

            case (current_state)
                IDLE: begin
                    if (start_search) begin
                        azimuth_ptr <= azimuth_angle_min;
                        init_counter <= 2'd0;
                        local_min_count <= '0;
                        waiting <= 1'b0;
                    end
                end

                // INIT_WINDOW: request 3 points
                INIT_WINDOW: begin
                    if (!waiting && init_counter <= 2) begin
                        calu_azimuth <= azimuth_ptr + init_counter * STEP;
                        calu_start <= 1'b1;
                        waiting <= 1'b1;
                    end else if (calu_valid) begin
                        window_cache[init_counter] <= calced_angle;
                        waiting <= 1'b0;
                        init_counter <= init_counter + 1;
                    end
                end

                // FIND_LOCAL_MIN: compare center (找局部最小值)
                FIND_LOCAL_MIN: begin
                    // 中心点比两边都小则为局部最小值
                    if ((window_cache[0] >= window_cache[1]) && (window_cache[2] >= window_cache[1])) begin
                        if (local_min_count < LOCAL_MIN_DEPTH) begin
                            local_min[local_min_count] <= window_cache[1];
                            local_min_angle[local_min_count] <= azimuth_ptr + STEP;
                            local_min_count <= local_min_count + 1;
                        end
                    end
                end

                // SLIDE_WINDOW: request new rightmost point
                SLIDE_WINDOW: begin
                    if (!waiting) begin
                        calu_azimuth <= azimuth_ptr + 3 * STEP;
                        calu_start <= 1'b1;
                        waiting <= 1'b1;
                    end else if (calu_valid) begin
                        window_cache[0] <= window_cache[1];
                        window_cache[1] <= window_cache[2];
                        window_cache[2] <= calced_angle;
                        waiting <= 1'b0;
                    end
                end

                // UPDATE_PTR
                UPDATE_PTR: begin
                    azimuth_ptr <= azimuth_ptr + STEP;
                end

                DONE: begin
                    search_done <= 1'b1;
                end

                default: ;
            endcase
        end
    end

endmodule