module Calu_Covariance_Top #(
    parameter N           = 4,    // number of array elements
    parameter DATA_WIDTH  = 16,   // input bit width
    parameter ACC_WIDTH   = 20,   // accumulator bit width
    parameter SAMPLES_NUM = 4096, // number of samples
    parameter DONE_WAIT_CYCLES = 10 // wait cycles in DONE state
)(
    input  wire                       iclk,
    input  wire                       irst_n,
    input  wire                       istart,        // start calculation signal
    input  wire                       idata_valid,

    // input data (I/Q components)
    input  wire signed [DATA_WIDTH-1:0] idata_i [0:N-1],
    input  wire signed [DATA_WIDTH-1:0] idata_q [0:N-1],

    // covariance output
    output wire signed [ACC_WIDTH-1:0] oresult_diag   [0:N-1],                 // Rii
    output wire signed [ACC_WIDTH-1:0] oresult_upper_q[0:(N*(N-1)/2)-1],       // Rij real part
    output wire signed [ACC_WIDTH-1:0] oresult_upper_i[0:(N*(N-1)/2)-1],       // Rij imag part
    output reg                         oresult_valid,  // result valid flag
    output reg                         ocalu_done      // calculation done flag
);

    // ==================== State Machine Parameters ====================
    localparam [1:0] IDLE = 2'b00;
    localparam [1:0] CALU = 2'b01;
    localparam [1:0] WAIT = 2'b10;
    localparam [1:0] DONE = 2'b11;

    reg [1:0] current_state, next_state;
    
    // ==================== State Machine Control ====================
    localparam DONE_CNT_BITS = $clog2(DONE_WAIT_CYCLES);
    reg [DONE_CNT_BITS:0] done_wait_cnt;  // extra bit to handle exact count
    
    // Internal signals to control the original logic
    reg internal_rst_n;
    wire internal_data_valid;
    wire internal_samples_complete;

    // State register
    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    // Next state logic
    always @(*) begin
        case (current_state)
            IDLE: begin
                if (istart)
                    next_state = CALU;
                else
                    next_state = IDLE;
            end
            
            CALU: begin
                if (internal_samples_complete)
                    next_state = WAIT;
                else
                    next_state = CALU;
            end

            WAIT:begin
                if (finish_flag[N-1][N-1])
                    next_state = DONE;
                else
                    next_state = WAIT;
            end
            
            DONE: begin
                if (done_wait_cnt >= DONE_WAIT_CYCLES)
                    next_state = IDLE;
                else
                    next_state = DONE;
            end
            
            default: next_state = IDLE;
        endcase
    end

    // State machine output logic
    always @(posedge iclk or negedge irst_n) begin
        if (!irst_n) begin
            done_wait_cnt <= {(DONE_CNT_BITS+1){1'b0}};
            internal_rst_n <= 1'b0;
            oresult_valid <= 1'b0;
            ocalu_done <= 1'b0;
        end else begin
            case (current_state)
                IDLE: begin
                    done_wait_cnt <= {(DONE_CNT_BITS+1){1'b0}};
                    internal_rst_n <= 1'b0;  // Reset internal logic
                    oresult_valid <= 1'b0;
                    ocalu_done <= 1'b0;
                end
                
                CALU: begin
                    internal_rst_n <= 1'b1;  // Enable internal logic
                    oresult_valid <= 1'b0;
                    ocalu_done <= 1'b0;
                end

                WAIT:begin

                end
                
                DONE: begin
                    internal_rst_n <= 1'b1;  // Keep internal logic enabled
                    oresult_valid <= 1'b1;   // Result is valid
                    ocalu_done <= 1'b1;      // Calculation is done
                    
                    // Wait counter
                    if (done_wait_cnt < DONE_WAIT_CYCLES)
                        done_wait_cnt <= done_wait_cnt + 1'b1;
                    else
                        done_wait_cnt <= {(DONE_CNT_BITS+1){1'b0}};
                end
                
                default: begin
                    done_wait_cnt <= {(DONE_CNT_BITS+1){1'b0}};
                    internal_rst_n <= 1'b0;
                    oresult_valid <= 1'b0;
                    ocalu_done <= 1'b0;
                end
            endcase
        end
    end

    // Control internal data valid - only allow when in CALU state
    assign internal_data_valid = (current_state == CALU) ? idata_valid : 1'b0;

    // ==================== 原始逻辑保持完全不变 ====================
    // ==================== 1. Sample counter ====================
    localparam SAMPLES_BITS = $clog2(SAMPLES_NUM);

    reg [SAMPLES_BITS-1:0] global_sample_cnt;
    reg samples_complete;

    always @(posedge iclk or negedge internal_rst_n) begin
        if (!internal_rst_n) begin
            global_sample_cnt <= {SAMPLES_BITS{1'b0}};
            samples_complete <= 1'b0;
        end else begin
            if (internal_data_valid) begin
                if (global_sample_cnt==(SAMPLES_NUM-1)) begin
                    global_sample_cnt <= {SAMPLES_BITS{1'b0}};
                    samples_complete <= 1'b1;
                end else begin
                    global_sample_cnt <= global_sample_cnt + 1'b1;
                    samples_complete <= 1'b0;
                end
            end else begin
                global_sample_cnt <= global_sample_cnt ;
                samples_complete <= 1'b0;
            end
        end
    end

    assign internal_samples_complete = samples_complete;

    // ==================== 2. Input delay lines (time alignment) ====================
    reg signed [DATA_WIDTH-1:0] delayed_data_q [0:N-1][0:N-2];
    reg signed [DATA_WIDTH-1:0] delayed_data_i [0:N-1][0:N-2];

    genvar col;
    integer s;
    generate
        for (col = 1; col < N; col = col + 1) begin : gen_delay_lines
            always @(posedge iclk or negedge internal_rst_n) begin
                if (!internal_rst_n) begin
                    for (s = 0; s < col; s = s + 1) begin
                        delayed_data_q[col][s] <= {DATA_WIDTH{1'b0}};
                        delayed_data_i[col][s] <= {DATA_WIDTH{1'b0}};
                    end
                end else begin 
                    if(internal_data_valid) begin
                        // first stage take input sample
                        delayed_data_q[col][0] <= idata_q[col];
                        delayed_data_i[col][0] <= idata_i[col];

                        // shift register for alignment
                        for (s = 1; s < col; s = s + 1) begin
                            delayed_data_q[col][s] <= delayed_data_q[col][s-1];
                            delayed_data_i[col][s] <= delayed_data_i[col][s-1];
                        end
                    end else begin                    
                        delayed_data_q[col][0] <= 0;
                        delayed_data_i[col][0] <= 0;
                        for (s = 1; s < col; s = s + 1) begin
                            delayed_data_q[col][s] <= delayed_data_q[col][s-1];
                            delayed_data_i[col][s] <= delayed_data_i[col][s-1];
                        end
                    end
                end
            end
        end
    endgenerate

    // aligned input
    wire signed [DATA_WIDTH-1:0] aligned_data_q [0:N-1];
    wire signed [DATA_WIDTH-1:0] aligned_data_i [0:N-1];

    assign aligned_data_q[0] = idata_q[0];
    assign aligned_data_i[0] = idata_i[0];

    generate
        for (col = 1; col < N; col = col + 1) begin : gen_aligned_signals
            assign aligned_data_q[col] = delayed_data_q[col][col-1];
            assign aligned_data_i[col] = delayed_data_i[col][col-1];
        end
    endgenerate

    // ==================== 3. Finish flag chain ====================
    wire finish_flag [0:N-1][0:N-1];

    // ==================== 4. PE internal interconnection ====================
    wire signed [DATA_WIDTH-1:0] a_data_q [0:N-1][0:N-1];
    wire signed [DATA_WIDTH-1:0] a_data_i [0:N-1][0:N-1];

    wire signed [DATA_WIDTH-1:0] b_data_q [0:N-1][0:N-1];
    wire signed [DATA_WIDTH-1:0] b_data_i [0:N-1][0:N-1];

    wire signed [ACC_WIDTH-1:0] pe_result_q [0:N-1][0:N-1];
    wire signed [ACC_WIDTH-1:0] pe_result_i [0:N-1][0:N-1];
    wire pe_result_valid [0:N-1][0:N-1];

    // ==================== 5. PE array generation ====================
    genvar row, col_gen;
    generate
        for (row = 0; row < N; row = row + 1) begin : gen_pe_rows
            for (col_gen = 0; col_gen < N; col_gen = col_gen + 1) begin : gen_pe_cols
                
                // diagonal PE
                if (row == col_gen) begin : pe_diagonal
                    if (row == 0) begin : pe_diag_first_row
                        pe_diag #(
                            .DATA_WIDTH(DATA_WIDTH),
                            .ACC_WIDTH(ACC_WIDTH),
                            .SAMPLES_BITS(SAMPLES_BITS)
                        ) pe_inst (
                            .clk(iclk),
                            .rst_n(internal_rst_n),
                            .finish_flag_in(samples_complete),
                            .data_q(aligned_data_q[col_gen]),
                            .data_i(aligned_data_i[col_gen]),
                            .a_data_out_q(a_data_q[row][col_gen]),
                            .a_data_out_i(a_data_i[row][col_gen]),
                            .finish_flag_out(finish_flag[row][col_gen]),
                            .result_q(pe_result_q[row][col_gen])
                        );

                        assign pe_result_i[row][col_gen] = {ACC_WIDTH{1'b0}};

                        assign b_data_q[row][col_gen] = aligned_data_q[col_gen];
                        assign b_data_i[row][col_gen] = aligned_data_i[col_gen];

                    end else begin : pe_diag_other_rows
                        pe_diag #(
                            .DATA_WIDTH(DATA_WIDTH),
                            .ACC_WIDTH(ACC_WIDTH),
                            .SAMPLES_BITS(SAMPLES_BITS)
                        ) pe_inst (
                            .clk(iclk),
                            .rst_n(internal_rst_n),
                            .finish_flag_in(finish_flag[row-1][col_gen]),
                            .data_q(b_data_q[row-1][col_gen]),
                            .data_i(b_data_i[row-1][col_gen]),
                            .a_data_out_q(a_data_q[row][col_gen]),
                            .a_data_out_i(a_data_i[row][col_gen]),
                            .finish_flag_out(finish_flag[row][col_gen]),
                            .result_q(pe_result_q[row][col_gen])
                        );

                        assign pe_result_i[row][col_gen] = {ACC_WIDTH{1'b0}};
                        assign b_data_q[row][col_gen] = b_data_q[row-1][col_gen];
                        assign b_data_i[row][col_gen] = b_data_i[row-1][col_gen];
                    end
                    
                end else if (col_gen > row) begin : pe_upper
                    // off-diagonal PE
                    if (row == 0) begin : pe_upper_first_row
                        pe_offdiag #(
                            .DATA_WIDTH(DATA_WIDTH),
                            .ACC_WIDTH(ACC_WIDTH),
                            .SAMPLES_BITS(SAMPLES_BITS)
                        ) pe_inst (
                            .clk(iclk),
                            .rst_n(internal_rst_n),
                            .finish_flag_in(samples_complete),
                            .a_data_q(a_data_q[row][col_gen-1]),
                            .a_data_i(a_data_i[row][col_gen-1]),
                            .b_data_q(aligned_data_q[col_gen]),
                            .b_data_i(aligned_data_i[col_gen]),
                            .a_data_out_q(a_data_q[row][col_gen]),
                            .a_data_out_i(a_data_i[row][col_gen]),
                            .b_data_out_q(b_data_q[row][col_gen]),
                            .b_data_out_i(b_data_i[row][col_gen]),
                            .finish_flag_out(finish_flag[row][col_gen]),
                            .result_q(pe_result_q[row][col_gen]),
                            .result_i(pe_result_i[row][col_gen])
                        );
                        
                    end else begin : pe_upper_other_rows
                        pe_offdiag #(
                            .DATA_WIDTH(DATA_WIDTH),
                            .ACC_WIDTH(ACC_WIDTH),
                            .SAMPLES_BITS(SAMPLES_BITS)
                        ) pe_inst (
                            .clk(iclk),
                            .rst_n(internal_rst_n),
                            .finish_flag_in(finish_flag[row-1][col_gen]),
                            .a_data_q(a_data_q[row][col_gen-1]),
                            .a_data_i(a_data_i[row][col_gen-1]),
                            .b_data_q(b_data_q[row-1][col_gen]),
                            .b_data_i(b_data_i[row-1][col_gen]),
                            .a_data_out_q(a_data_q[row][col_gen]),
                            .a_data_out_i(a_data_i[row][col_gen]),
                            .b_data_out_q(b_data_q[row][col_gen]),
                            .b_data_out_i(b_data_i[row][col_gen]),
                            .finish_flag_out(finish_flag[row][col_gen]),
                            .result_q(pe_result_q[row][col_gen]),
                            .result_i(pe_result_i[row][col_gen])
                        );
                    end
                end
            end
        end
    endgenerate

    // ==================== 6. Output mapping ====================

    // diagonal elements
    generate
        for (col = 0; col < N; col = col + 1) begin : gen_diag_output
            assign oresult_diag[col] = pe_result_q[col][col];
        end
    endgenerate
    
    // upper triangular elements
    integer idx;
    generate
        for (row = 0; row < N; row = row + 1) begin : gen_upper_output
            for (col_gen = row + 1; col_gen < N; col_gen = col_gen + 1) begin : gen_col
                localparam OUT_IDX = (row * (2*N - row - 1)) / 2 + (col_gen - row - 1);
                assign oresult_upper_q[OUT_IDX] = pe_result_q[row][col_gen];
                assign oresult_upper_i[OUT_IDX] = pe_result_i[row][col_gen];
            end
        end
    endgenerate

endmodule