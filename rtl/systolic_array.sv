`timescale 1ns/1ps

// =============================================================================
// Module: systolic_array
// =============================================================================
// Parallel-Feed Systolic Array Classifier
//
// With 4 independent weight ROM instances (one per class), all 4 class
// weights for the current cell are available simultaneously each clock cycle.
// This eliminates the 4-phase ROM multiplexing from the previous design and
// cuts inference to NUM_CELLS + 2 drain cycles.
//
// Operation timeline per cell i (0-indexed):
//   Cycle i  : Issue cell address w_addr = i
//              Latch feature_in = feature[i] into feat_pipe_r
//   Cycle i+1: ROM delivers weight_k[i] for all k (1-cycle latency)
//              Accumulate: acc[k] += weight_k[i] * feat_pipe_r[i]
//
// Total cycles: NUM_CELLS cycles (feed) + 1 drain cycle = NUM_CELLS+1
//
// Weight interface (flat packed bus — avoids iverilog uwire issues):
//   w_data_flat[(k+1)*WEIGHT_BITS-1 : k*WEIGHT_BITS] = weight for class k
//
// Scores output (flat packed bus — same reason):
//   scores_flat[(k+1)*ACC_BITS-1 : k*ACC_BITS] = final score for class k
// =============================================================================

module systolic_array #(
    parameter NUM_CLASSES = 4,
    parameter NUM_CELLS   = 1024,
    parameter VALUE_BITS  = 8,
    parameter WEIGHT_BITS = 8,
    parameter ACC_BITS    = 24
)(
    input  logic                                    clk,
    input  logic                                    rst,

    // Control
    input  logic                                    start,

    // Feature stream — one value per cycle, NUM_CELLS total
    input  logic [VALUE_BITS-1:0]                   feature_in,

    // Parallel weight ROM interface
    // Caller drives 4 ROMs in parallel; all share the same cell address
    output logic [$clog2(NUM_CELLS)-1:0]            w_addr,
    input  logic [NUM_CLASSES*WEIGHT_BITS-1:0]      w_data_flat,  // 4×8=32 bits

    // Results
    output logic                                    result_valid,
    output logic [1:0]                              best_class,
    output logic [NUM_CLASSES*ACC_BITS-1:0]         scores_flat   // 4×24=96 bits
);

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    localparam CNT_BITS = $clog2(NUM_CELLS + 4);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_RUNNING = 2'd1,
        S_DRAIN   = 2'd2,   // One drain cycle to flush last feature/weight
        S_ARGMAX  = 2'd3
    } state_t;

    state_t state;

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    logic [CNT_BITS-1:0] cell_cnt;   // Counts 0..NUM_CELLS-1

    // -------------------------------------------------------------------------
    // PE Accumulators
    // -------------------------------------------------------------------------
    logic signed [ACC_BITS-1:0] acc   [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] acc_r [0:NUM_CLASSES-1];  // Latched for output

    // -------------------------------------------------------------------------
    // Feature pipeline (1-cycle delay to align with ROM latency)
    // -------------------------------------------------------------------------
    logic [VALUE_BITS-1:0] feat_pipe_r;
    logic                  pipe_valid_r;  // Delayed 'running' flag

    // -------------------------------------------------------------------------
    // Helper: extract per-class weight from flat bus
    // -------------------------------------------------------------------------
    wire signed [WEIGHT_BITS-1:0] w [0:NUM_CLASSES-1];
    generate
        genvar gk;
        for (gk = 0; gk < NUM_CLASSES; gk = gk + 1) begin : gen_w_unpack
            assign w[gk] = $signed(w_data_flat[(gk+1)*WEIGHT_BITS-1 : gk*WEIGHT_BITS]);
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Argmax
    // -------------------------------------------------------------------------
    logic signed [ACC_BITS-1:0] max_score;
    integer k, i;

    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            cell_cnt     <= '0;
            w_addr       <= '0;
            feat_pipe_r  <= '0;
            pipe_valid_r <= 1'b0;
            result_valid <= 1'b0;
            best_class   <= '0;
            max_score    <= '0;
            for (k = 0; k < NUM_CLASSES; k = k + 1) begin
                acc  [k] <= '0;
                acc_r[k] <= '0;
            end
        end else begin
            result_valid <= 1'b0;

            case (state)
                // -----------------------------------------------------------
                S_IDLE: begin
                    pipe_valid_r <= 1'b0;
                    if (start) begin
                        for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= '0;
                        cell_cnt    <= '0;
                        w_addr      <= '0;
                        feat_pipe_r <= feature_in;  // Cell 0 feature
                        state       <= S_RUNNING;
                    end
                end

                // -----------------------------------------------------------
                S_RUNNING: begin
                    // ---- Accumulate (previous cycle's weight × feature) ----
                    // pipe_valid_r is low on first cycle (no prior data yet)
                    if (pipe_valid_r) begin
                        for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= acc[k] +
                                ACC_BITS'($signed({1'b0, feat_pipe_r}) * $signed(w[k]));
                    end

                    // ---- Advance address and latch feature ----
                    pipe_valid_r <= 1'b1;
                    feat_pipe_r  <= feature_in;
                    cell_cnt     <= cell_cnt + 1'b1;

                    if (cell_cnt < CNT_BITS'(NUM_CELLS - 1)) begin
                        w_addr <= w_addr + 1'b1;
                    end

                    if (cell_cnt == CNT_BITS'(NUM_CELLS - 1)) begin
                        // Last cell issued — one more drain cycle
                        state <= S_DRAIN;
                    end
                end

                // -----------------------------------------------------------
                S_DRAIN: begin
                    // Accumulate the last cell's weight×feature
                    for (k = 0; k < NUM_CLASSES; k = k + 1)
                        acc[k] <= acc[k] +
                            ACC_BITS'($signed({1'b0, feat_pipe_r}) * $signed(w[k]));
                    pipe_valid_r <= 1'b0;
                    state        <= S_ARGMAX;
                end

                // -----------------------------------------------------------
                S_ARGMAX: begin
                    // Latch final accs
                    for (k = 0; k < NUM_CLASSES; k = k + 1)
                        acc_r[k] <= acc[k];

                    // Argmax
                    max_score  <= acc[0];
                    best_class <= 2'd0;
                    for (i = 1; i < NUM_CLASSES; i = i + 1) begin
                        if ($signed(acc[i]) > $signed(max_score)) begin
                            max_score  <= acc[i];
                            best_class <= 2'(i);
                        end
                    end

                    result_valid <= 1'b1;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Drive flat scores bus from latched accumulators
    // -------------------------------------------------------------------------
    generate
        genvar gs;
        for (gs = 0; gs < NUM_CLASSES; gs = gs + 1) begin : gen_scores
            assign scores_flat[(gs+1)*ACC_BITS-1 : gs*ACC_BITS] = acc_r[gs];
        end
    endgenerate

endmodule
