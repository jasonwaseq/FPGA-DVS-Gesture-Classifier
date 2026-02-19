`timescale 1ns/1ps

// =============================================================================
// Module: spatio_temporal_classifier
// =============================================================================
// Spatio-Temporal DVS Gesture Classifier — 32×32 grid edition
//
// Drop-in replacement for the old feature_extractor interface.
//
// Processing Pipeline:
//   1. Frame Pulse   — Every FRAME_PERIOD_MS milliseconds
//   2. Flatten       — flatten_buffer scans 1024 time-surface cells,
//                      producing a 1-D exponentially-decayed feature vector
//   3. Feature Feed  — Feeds 1 cell/cycle to systolic_array
//   4. Systolic MAC  — systolic_array reads 4 weight_rom instances in parallel,
//                      completes dot-products in NUM_CELLS+1 cycles (~1025)
//   5. Argmax        — Best class index selected
//   6. Threshold     — gesture_valid gated on minimum surface energy
//
// Weight ROM instances:
//   u_wrom0 (CLASS_IDX=0) — UP
//   u_wrom1 (CLASS_IDX=1) — DOWN
//   u_wrom2 (CLASS_IDX=2) — LEFT
//   u_wrom3 (CLASS_IDX=3) — RIGHT
// =============================================================================

module spatio_temporal_classifier #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 10,
    parameter GRID_SIZE       = 32,
    parameter VALUE_BITS      = 8,
    parameter MOMENT_BITS     = 24,
    parameter WEIGHT_BITS     = 8,
    parameter SCORE_BITS      = 24,
    parameter NUM_CLASSES     = 4,
    parameter MIN_MASS_THRESH = 100
)(
    input  logic                    clk,
    input  logic                    rst,

    // Time-Surface Read Interface (driven to time_surface_encoder)
    output logic [9:0]              ts_read_addr,   // 10-bit for 1024 cells
    output logic                    ts_read_enable,
    input  logic [VALUE_BITS-1:0]   ts_read_value,

    // Classification Output
    output logic [1:0]              gesture_class,
    output logic                    gesture_valid,
    output logic [7:0]              gesture_confidence,

    // Debug Outputs
    output logic [MOMENT_BITS-1:0]  debug_m00,   // Total surface energy
    output logic [MOMENT_BITS-1:0]  debug_m10,   // Score[0] (UP)
    output logic [MOMENT_BITS-1:0]  debug_m01,   // Score[1] (DOWN)
    output logic [2:0]              debug_state
);

    localparam NUM_CELLS  = GRID_SIZE * GRID_SIZE;        // 1024
    localparam ADDR_BITS  = $clog2(NUM_CELLS);            // 10

    // =========================================================================
    // Frame Period Counter
    // =========================================================================
    localparam FRAME_CYCLES   = (CLK_FREQ_HZ / 1000) * FRAME_PERIOD_MS;
    localparam FRAME_CNT_BITS = $clog2(FRAME_CYCLES + 1);

    logic [FRAME_CNT_BITS-1:0] frame_counter;
    logic frame_pulse;

    always_ff @(posedge clk) begin
        if (rst) begin
            frame_counter <= '0;
            frame_pulse   <= 1'b0;
        end else begin
            if (frame_counter >= FRAME_CNT_BITS'(FRAME_CYCLES - 1)) begin
                frame_counter <= '0;
                frame_pulse   <= 1'b1;
            end else begin
                frame_counter <= frame_counter + 1'b1;
                frame_pulse   <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Flatten Buffer
    // =========================================================================
    logic                    flat_valid;
    logic [VALUE_BITS-1:0]   flat_data [0:NUM_CELLS-1];
    logic [ADDR_BITS-1:0]    flat_ts_addr;
    logic                    flat_ts_en;

    flatten_buffer #(
        .GRID_SIZE (GRID_SIZE),
        .VALUE_BITS(VALUE_BITS),
        .NUM_CELLS (NUM_CELLS),
        .PIPE_DEPTH(2)
    ) u_flatten (
        .clk        (clk),
        .rst        (rst),
        .start      (frame_pulse),
        .ts_addr    (flat_ts_addr),
        .ts_en      (flat_ts_en),
        .ts_val     (ts_read_value),
        .flat_valid (flat_valid),
        .flat_data  (flat_data)
    );

    assign ts_read_addr   = 10'(flat_ts_addr);
    assign ts_read_enable = flat_ts_en;

    // =========================================================================
    // Surface Energy Accumulator (for threshold gate)
    // Tracks the sum of all scanned cell values across each frame.
    // =========================================================================
    logic [MOMENT_BITS-1:0] energy_acc;
    logic [1:0] scan_active_sr;   // 2-deep SR to track when ts_val is valid

    always_ff @(posedge clk) begin
        if (rst) begin
            energy_acc     <= '0;
            scan_active_sr <= 2'b00;
        end else begin
            scan_active_sr <= {scan_active_sr[0], flat_ts_en};
            if (frame_pulse)
                energy_acc <= '0;
            if (scan_active_sr[1])
                energy_acc <= energy_acc + MOMENT_BITS'(ts_read_value);
        end
    end

    // =========================================================================
    // 4× Parallel Weight ROM instances (one per gesture class)
    // All share the same cell address (w_addr from systolic_array)
    // =========================================================================
    logic [ADDR_BITS-1:0]            w_addr;
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] w_data_flat;

    // Individual ROM outputs
    logic signed [WEIGHT_BITS-1:0] w0, w1, w2, w3;

    weight_rom #(.CLASS_IDX(0), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom0 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w0));

    weight_rom #(.CLASS_IDX(1), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom1 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w1));

    weight_rom #(.CLASS_IDX(2), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom2 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w2));

    weight_rom #(.CLASS_IDX(3), .NUM_CLASSES(NUM_CLASSES),
                 .NUM_CELLS(NUM_CELLS), .GRID_SIZE(GRID_SIZE),
                 .WEIGHT_BITS(WEIGHT_BITS)) u_wrom3 (
        .clk(clk), .rst(rst), .cell_addr(w_addr), .dout(w3));

    // Pack individual ROM outputs into flat bus for systolic_array
    assign w_data_flat = {w3, w2, w1, w0};  // k=0 at LSB

    // =========================================================================
    // Systolic Array (parallel 4-ROM, one cell per cycle)
    // =========================================================================
    logic                           sa_start;
    logic [VALUE_BITS-1:0]          sa_feature_in;
    logic                           sa_result_valid;
    logic [1:0]                     sa_best_class;
    logic [SCORE_BITS*NUM_CLASSES-1:0] sa_scores_flat;

    // Extract individual class scores for debug
    wire signed [SCORE_BITS-1:0] sa_score0 = $signed(sa_scores_flat[1*SCORE_BITS-1:0*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score1 = $signed(sa_scores_flat[2*SCORE_BITS-1:1*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score2 = $signed(sa_scores_flat[3*SCORE_BITS-1:2*SCORE_BITS]);
    wire signed [SCORE_BITS-1:0] sa_score3 = $signed(sa_scores_flat[4*SCORE_BITS-1:3*SCORE_BITS]);

    systolic_array #(
        .NUM_CLASSES(NUM_CLASSES),
        .NUM_CELLS  (NUM_CELLS),
        .VALUE_BITS (VALUE_BITS),
        .WEIGHT_BITS(WEIGHT_BITS),
        .ACC_BITS   (SCORE_BITS)
    ) u_systolic (
        .clk         (clk),
        .rst         (rst),
        .start       (sa_start),
        .feature_in  (sa_feature_in),
        .w_addr      (w_addr),
        .w_data_flat (w_data_flat),
        .result_valid(sa_result_valid),
        .best_class  (sa_best_class),
        .scores_flat (sa_scores_flat)
    );

    // =========================================================================
    // Feature Stream Driver
    // Feeds flat_data[cell] to systolic_array, one cell per cycle.
    // systolic_array drives w_addr; we use w_addr as the feature index.
    // =========================================================================
    typedef enum logic [1:0] {
        FS_IDLE = 2'd0,
        FS_FEED = 2'd1,
        FS_WAIT = 2'd2
    } fs_state_t;

    fs_state_t fs_state;
    logic [ADDR_BITS-1:0] fs_cell;

    always_ff @(posedge clk) begin
        if (rst) begin
            fs_state      <= FS_IDLE;
            fs_cell       <= '0;
            sa_start      <= 1'b0;
            sa_feature_in <= '0;
        end else begin
            sa_start <= 1'b0;

            case (fs_state)
                FS_IDLE: begin
                    if (flat_valid) begin
                        fs_cell       <= ADDR_BITS'(1);  // Cell 0 fed on start
                        sa_start      <= 1'b1;
                        sa_feature_in <= flat_data[0];
                        fs_state      <= FS_FEED;
                    end
                end

                FS_FEED: begin
                    // systolic_array drives w_addr; we feed the matching feature
                    // (w_addr lags our cell by 1 cycle; use fs_cell which mirrors w_addr+1)
                    sa_feature_in <= flat_data[fs_cell];
                    if (fs_cell < ADDR_BITS'(NUM_CELLS - 1)) begin
                        fs_cell <= fs_cell + 1'b1;
                    end else begin
                        fs_state <= FS_WAIT;
                    end
                end

                FS_WAIT: begin
                    if (sa_result_valid)
                        fs_state <= FS_IDLE;
                end

                default: fs_state <= FS_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Output Register
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            gesture_class      <= '0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
        end else begin
            gesture_valid <= 1'b0;
            if (sa_result_valid) begin
                if (energy_acc >= MOMENT_BITS'(MIN_MASS_THRESH)) begin
                    gesture_valid  <= 1'b1;
                    gesture_class  <= sa_best_class;
                    gesture_confidence <= (energy_acc > MOMENT_BITS'(65535))
                                          ? 8'd255
                                          : energy_acc[15:8];
                end
            end
        end
    end

    // =========================================================================
    // Debug Outputs
    // =========================================================================
    assign debug_m00   = energy_acc;
    assign debug_m10   = MOMENT_BITS'(sa_score0);
    assign debug_m01   = MOMENT_BITS'(sa_score1);
    assign debug_state = {1'b0, fs_state[1:0]};

endmodule
