`timescale 1ns/1ps

module tb_Music_Top();

    // =====================================================
    // Parameter Definitions
    // =====================================================
    parameter N               = 4;
    parameter DATA_WIDTH      = 16;
    parameter ACC_WIDTH       = 32;
    parameter JACOBI_WIDTH    = 48;
    parameter DOASEARCH_WIDTH = 64;
    parameter SAMPLES_NUM     = 4096;

    // =====================================================
    // Signal Declarations
    // =====================================================
    reg                             clk;
    reg                             rst_n;
    reg                             data_valid;
    
    reg signed [DATA_WIDTH-1:0]     data_i0, data_q0;
    reg signed [DATA_WIDTH-1:0]     data_i1, data_q1;
    reg signed [DATA_WIDTH-1:0]     data_i2, data_q2;
    reg signed [DATA_WIDTH-1:0]     data_i3, data_q3;
    
    wire [9:0]                      azimuth_angle;
    wire                            doa_search_done;

    // =====================================================
    // DUT Instantiation
    // =====================================================
    Music_Top #(
        .N(N),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .JACOBI_WIDTH(JACOBI_WIDTH),
        .DOASEARCH_WIDTH(DOASEARCH_WIDTH),
        .SAMPLES_NUM(512)                  // DUT 内部参数保持原样
    ) dut (
        .iclk(clk),
        .irst_n(rst_n),
        .idata_valid(data_valid),
        
        .idata_i0(data_i0), .idata_q0(data_q0),
        .idata_i1(data_i1), .idata_q1(data_q1),
        .idata_i2(data_i2), .idata_q2(data_q2),
        .idata_i3(data_i3), .idata_q3(data_q3),
        
        .oazimuth_angle(azimuth_angle),
        .odoa_search_done(doa_search_done)
    );

    // =====================================================
    // Clock Generation (100 MHz)
    // =====================================================
    initial clk = 0;
    always #5 clk = ~clk;

    // =====================================================
    // File I/O Variables
    // =====================================================
    integer file, r, c, scan_result;
    integer iq_data [0:4095][0:2*N-1];
    
    integer samples_fed;
    integer round_count = 0;  // 新增：记录发送轮数

    // =====================================================
    // Data File Reading Task
    // =====================================================
    task read_data_file;
        begin
            file = $fopen("D:/FPGA/MUISC/Data/dan-1224-30--50dbm.txt", "r");
            if (file == 0) begin
                $display("❌ ERROR: Failed to open data file!");
                $display("    Using simulated data instead...");
                generate_test_data();
            end else begin
                $display("✅ Reading data file...");
                for (r = 0; r < 4096; r = r + 1) begin
                    for (c = 0; c < 2*N; c = c + 1) begin
                        scan_result = $fscanf(file, "%d", iq_data[r][c]);
                        if (scan_result != 1) begin
                            $display("❌ ERROR: Failed to read file at row=%0d, col=%0d", r, c);
                            $fclose(file);
                            $finish;
                        end
                    end
                end
                $fclose(file);
                $display("✅ File reading completed, total %0d samples loaded.", SAMPLES_NUM);
            end
        end
    endtask

    // =====================================================
    // Generate Test Data (fallback)
    // =====================================================
    task generate_test_data;
        integer i, j;
        begin
            $display("🔧 Generating simulated test data...");
            for (i = 0; i < SAMPLES_NUM; i = i + 1) begin
                for (j = 0; j < 2*N; j = j + 1) begin
                    iq_data[i][j] = $rtoi(1000 * $sin(2 * 3.14159 * i * (j+1) / 32.0));
                end
            end
            $display("✅ Test data generation completed.");
        end
    endtask

    // =====================================================
    // Monitor State Changes
    // =====================================================
    always @(posedge clk) begin
        if (rst_n) begin
            case (dut.current_state)
                3'b000: if (dut.next_state != 3'b000) $display("🔄 State: IDLE → %s", 
                    (dut.next_state == 3'b001) ? "COV_CALC" : "OTHER");
                3'b001: if (dut.next_state != 3'b001) $display("🔄 State: COV_CALC → %s",
                    (dut.next_state == 3'b010) ? "JACOBI" : "OTHER");
                3'b010: if (dut.next_state != 3'b010) $display("🔄 State: JACOBI → %s",
                    (dut.next_state == 3'b011) ? "DOA_SEARCH" : "OTHER");
                3'b011: if (dut.next_state != 3'b011) $display("🔄 State: DOA_SEARCH → %s",
                    (dut.next_state == 3'b100) ? "DONE" : "OTHER");
                3'b100: if (dut.next_state != 3'b100) $display("🔄 State: DONE → IDLE");
            endcase
        end
    end

    // =====================================================
    // Initialization and Reset Sequence
    // =====================================================
    initial begin
        $display("🚀 Starting Music_Top Testbench (Loop Mode)");
        $display("📋 Parameters: N=%0d, DATA_WIDTH=%0d, SAMPLES=%0d", N, DATA_WIDTH, SAMPLES_NUM);
        
        rst_n = 0;
        data_valid = 0;
        data_i0 = 0; data_q0 = 0;
        data_i1 = 0; data_q1 = 0;
        data_i2 = 0; data_q2 = 0;
        data_i3 = 0; data_q3 = 0;
        samples_fed = 0;
        
        read_data_file();
        
        $display("🔄 Applying reset...");
        #100;
        rst_n = 1;
        $display("✅ Reset released");
        #150;        

        // 启动无限循环喂数据
        feed_data_loop();

        // 下面的代码不会执行到（除非你在 feed_data_loop 中加 $finish）
        #1000;
        $display("🏁 Simulation completed");
        $finish;
    end

    // =====================================================
    // 【修改】无限循环喂数据任务
    // =====================================================
    task feed_data_loop;
        integer r;
        begin
            $display("📤 Starting infinite loop data feeding...");
            
            while (1) begin  // 无限循环
                round_count = round_count + 1;
                $display("🔁 Starting Round %0d of data feeding", round_count);
                
                for (r = 0; r < SAMPLES_NUM; r = r + 1) begin
                    @(posedge clk);
                    
                    data_valid <= 1'b1;
                    data_q0 <= iq_data[r][0];
                    data_i0 <= iq_data[r][4];
                    data_q1 <= iq_data[r][1];
                    data_i1 <= iq_data[r][5];
                    data_q2 <= iq_data[r][2];
                    data_i2 <= iq_data[r][6];
                    data_q3 <= iq_data[r][3];
                    data_i3 <= iq_data[r][7];
                    
                    samples_fed = samples_fed + 1;
                    
                    if (r % 512 == 0 || r == SAMPLES_NUM-1) begin
                        $display("   Fed sample %0d/%0d in this round (total fed: %0d)", 
                                 r+1, SAMPLES_NUM, samples_fed);
                    end
                end
                
                // 一轮数据发送完毕后，短暂拉低 data_valid，给 DUT 一个明确的帧结束标志
                repeat(1000) @(posedge clk);
                data_valid <= 1'b0;
                repeat(1000) @(posedge clk);
                
                $display("✅ Round %0d completed (%0d samples sent this round)", round_count, SAMPLES_NUM);
            end
        end
    endtask

    // =====================================================
    // Wait for Completion Task（保持原样，可继续观察每轮结果）
    // =====================================================
    task automatic wait_for_completion;
        begin
            $display("⏳ Waiting for DOA processing to complete in current round...");
            
            fork
                begin
                    wait(dut.current_state == 3'b001);
                    $display("📐 Covariance calculation in progress...");
                    wait(dut.current_state != 3'b001);
                    $display("✅ Covariance calculation completed");
                end
                begin
                    wait(dut.current_state == 3'b010);
                    $display("🧮 Jacobi eigendecomposition in progress...");
                    wait(dut.current_state != 3'b010);
                    $display("✅ Jacobi eigendecomposition completed");
                end
                begin
                    wait(dut.current_state == 3'b011);
                    $display("🔍 DOA search in progress...");
                    wait(dut.current_state != 3'b011);
                    $display("✅ DOA search completed");
                end
            join
            
            wait(doa_search_done == 1'b1);
            $display("🎯 DOA calculation completed! (Round %0d)", round_count);
            $display("    Azimuth Angle: %0d degrees", azimuth_angle);
            $display("    Search Done:   %b\n", doa_search_done);
        end
    endtask



endmodule