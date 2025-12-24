
module Music_Top #(
    parameter N               = 4,      
    parameter DATA_WIDTH      = 16,     
    parameter ACC_WIDTH       = 32,     
    parameter JACOBI_WIDTH    = 48, 
    parameter DOASEARCH_WIDTH = 64,
    parameter SAMPLES_NUM     = 128
)(
    input  wire                       iclk,
    input  wire                       irst_n,
    input  wire                       idata_valid,    

    input  wire signed [DATA_WIDTH-1:0] idata_i0,
    input  wire signed [DATA_WIDTH-1:0] idata_q0,
    input  wire signed [DATA_WIDTH-1:0] idata_i1,
    input  wire signed [DATA_WIDTH-1:0] idata_q1,
    input  wire signed [DATA_WIDTH-1:0] idata_i2,
    input  wire signed [DATA_WIDTH-1:0] idata_q2,
    input  wire signed [DATA_WIDTH-1:0] idata_i3,
    input  wire signed [DATA_WIDTH-1:0] idata_q3,

    output reg  [9:0] oazimuth_angle,
    output reg        odoa_search_done
);

    localparam [2:0] IDLE        = 3'b000;
    localparam [2:0] COV_CALC    = 3'b001;
    localparam [2:0] JACOBI      = 3'b010;
    localparam [2:0] DOA_SEARCH  = 3'b011;
    localparam [2:0] DONE        = 3'b100;

    reg [2:0] current_state;
    reg [2:0] next_state;

    reg [9:0] idle_counter;
    localparam [9:0] IDLE_COUNT_MAX = 10'd9;

    reg start_jacobi;
    reg start_doa_search;

    // One-pulse busy flags
    reg jacobi_busy;
    reg doa_busy;

    reg Calu_cov_start;
    reg calu_cov_busy;

    wire signed [DATA_WIDTH-1:0] data_i [0:N-1];
    wire signed [DATA_WIDTH-1:0] data_q [0:N-1];

    assign data_i[0] = idata_i0;
    assign data_q[0] = idata_q0;
    assign data_i[1] = idata_i1;
    assign data_q[1] = idata_q1;
    assign data_i[2] = idata_i2;
    assign data_q[2] = idata_q2;
    assign data_i[3] = idata_i3;
    assign data_q[3] = idata_q3;

    wire signed [ACC_WIDTH-1:0] result_diag   [0:N-1];
    wire signed [ACC_WIDTH-1:0] result_upper_q[0:(N*(N-1)/2)-1];
    wire signed [ACC_WIDTH-1:0] result_upper_i[0:(N*(N-1)/2)-1];
    wire cov_calu_done;

    Calu_Covariance_Top #(
        .N(N),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .SAMPLES_NUM(SAMPLES_NUM)
    ) u0_calu_cov (
        .iclk(iclk),
        .irst_n(irst_n),
        .idata_valid(idata_valid),
        .istart(Calu_cov_start),
        .idata_i(data_i),
        .idata_q(data_q),
        .oresult_diag(result_diag),
        .oresult_upper_q(result_upper_q),
        .oresult_upper_i(result_upper_i),
        .ocalu_done(cov_calu_done)
    );

    wire signed [JACOBI_WIDTH-1:0] eigen_value  [0:N-1];
    wire signed [JACOBI_WIDTH-1:0] eigen_vector [0:2*N*N-1];
    wire jacobi_done;
    wire jacobi_valid;

    Calu_Jacobi_Top #(
        .N(N),
        .ACC_WIDTH(ACC_WIDTH),
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .MAX_ITER(14)
    ) u1_jacobi (
        .clk(iclk),
        .rst_n(irst_n),
        .Calu_Jacobi_Start(start_jacobi),
        .result_diag(result_diag),
        .result_upper_q(result_upper_q),
        .result_upper_i(result_upper_i),
        .eigen_value(eigen_value),
        .eigen_vector(eigen_vector),
        .jacobi_done(jacobi_done),
        .jacobi_valid(jacobi_valid)
    );

    wire [9:0] doa_azimuth_angle;
    wire doa_search_done;

    DOA_JUDGE_Top #(
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .DOASEARCH_WIDTH(DOASEARCH_WIDTH),
        .N(N),
        .COARSE_SEARCH_STEP(10),
        .FINE_SEARCH_STEP(1),
        .LOCAL_MIN_DEPTH(16)
    ) u2_doa (
        .iclk(iclk),
        .irst_n(irst_n),
        .istart_doa_search(start_doa_search),
        .ieigen_vector(eigen_vector),
        .ieigen_value(eigen_value),
        .oazimuth_angle(doa_azimuth_angle),
        .odoa_search_done(doa_search_done)
    );

    always @(posedge iclk or negedge irst_n) begin
        if(!irst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;
        case(current_state)
            IDLE:       if(idle_counter >= IDLE_COUNT_MAX) next_state = COV_CALC;
            COV_CALC:   if(cov_calu_done)                 next_state = JACOBI;
            JACOBI:     if(jacobi_done && jacobi_valid)    next_state = DOA_SEARCH;
            DOA_SEARCH: if(doa_search_done)                next_state = DONE;
            DONE:       next_state = IDLE;
        endcase
    end

    // ===============================
    // Output control with 1-cycle start pulses
    // ===============================
    always @(posedge iclk or negedge irst_n) begin
        if(!irst_n) begin
            start_jacobi     <= 0;
            jacobi_busy      <= 0;
            start_doa_search <= 0;
            doa_busy         <= 0;
            Calu_cov_start   <= 0;
            calu_cov_busy    <= 0;
            idle_counter     <= 0;
        end else begin
            start_jacobi     <= 0;
            start_doa_search <= 0;
            Calu_cov_start   <= 0;

            case(current_state)
                IDLE: begin
                    if(idle_counter < IDLE_COUNT_MAX)
                        idle_counter <= idle_counter + 1;
                    else
                        idle_counter <= 0;
                end

                COV_CALC: begin
                    if(!calu_cov_busy) begin
                        Calu_cov_start <= 1;
                        calu_cov_busy  <= 1;
                    end else if(cov_calu_done)
                        calu_cov_busy <= 0;
                end

                JACOBI: begin
                    if(!jacobi_busy) begin
                        start_jacobi <= 1;
                        jacobi_busy  <= 1;
                    end else if(jacobi_done)
                        jacobi_busy <= 0;
                end

                DOA_SEARCH: begin
                    if(!doa_busy) begin
                        start_doa_search <= 1;
                        doa_busy         <= 1;
                    end else if(doa_search_done)
                        doa_busy <= 0;
                end
            endcase
        end
    end

    always @(posedge iclk or negedge irst_n) begin
        if(!irst_n) begin
            oazimuth_angle   <= 0;
            odoa_search_done <= 0;
        end else begin
            if(current_state == DONE) begin
                oazimuth_angle   <= doa_azimuth_angle;
                odoa_search_done <= 1;
            end else
                odoa_search_done <= 0;
        end
    end

endmodule
