`timescale 1ns/1ps

// =============================================================================
// Module: SystolicMatrixMultiply
// =============================================================================
// Matrix Multiplication for Feature Classification
//
// Description:
//   Multiplies the flattened feature vector (from TimeSurfaceBinning)
//   by the weight matrices for each gesture class.
//   Structure mimics the 'Systolic Array' design but adapted for the
//   old architecture integration.
//
// Parameters:
//   NUM_CLASSES  - Number of output classes (4)
//   NUM_CELLS    - Input vector duration (READOUT_BINS * GRID_SIZE^2)
//                  For 5 bins * 256 cells = 1280.
//   VALUE_BITS   - Input feature bit width (8)
//   WEIGHT_BITS  - Weight bit width (8)
//   ACC_BITS     - Accumulator width (24)
//
// =============================================================================

module SystolicMatrixMultiply #(
    parameter NUM_CLASSES = 4,
    // Default to 5 bins * 16x16 = 1280
    parameter NUM_CELLS   = 1280, 
    parameter VALUE_BITS  = 8,
    parameter WEIGHT_BITS = 8,
    parameter ACC_BITS    = 24
)(
    input  logic                                    clk,
    input  logic                                    rst,

    // Control
    input  logic                                    start,

    // Feature stream — one value per cycle
    input  logic [VALUE_BITS-1:0]                   feature_in,
    input  logic                                    feature_valid, // Optional qualification

    // Parallel weight ROM interface
    output logic [$clog2(NUM_CELLS)-1:0]            w_addr,
    input  logic [NUM_CLASSES*WEIGHT_BITS-1:0]      w_data_flat,  // 4x8=32 bits

    // Results
    output logic                                    result_valid,
    output logic [1:0]                              best_class,
    output logic [NUM_CLASSES*ACC_BITS-1:0]         scores_flat
);

    // =========================================================================
    // Constants
    // =========================================================================
    localparam CNT_BITS = $clog2(NUM_CELLS + 4);

    // =========================================================================
    // State machine
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_RUNNING = 2'd1,
        S_DRAIN   = 2'd2,   // One drain cycle to flush last feature/weight
        S_ARGMAX  = 2'd3
    } state_t;

    state_t state;

    // =========================================================================
    // Internal Signals
    // =========================================================================
    logic [CNT_BITS-1:0] cell_cnt;   // Counts 0..NUM_CELLS-1

    // Accumulators
    logic signed [ACC_BITS-1:0] acc   [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] acc_r [0:NUM_CLASSES-1];

    // Feature pipeline (1-cycle delay to align with ROM latency)
    logic [VALUE_BITS-1:0] feat_pipe_r;
    logic                  pipe_valid_r;

    // Helper: extract per-class weight from flat bus
    wire signed [WEIGHT_BITS-1:0] w [0:NUM_CLASSES-1];
    generate
        genvar gk;
        for (gk = 0; gk < NUM_CLASSES; gk = gk + 1) begin : gen_w_unpack
            assign w[gk] = $signed(w_data_flat[(gk+1)*WEIGHT_BITS-1 : gk*WEIGHT_BITS]);
        end
    endgenerate

    // Argmax Signals
    logic signed [ACC_BITS-1:0] max_score;
    integer k, i;

    // =========================================================================
    // Logic
    // =========================================================================
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
                        // For the first cycle, we latch input IF it's valid immediately with start.
                        // Assuming streaming protocol: Start pulses high, and data is valid on same cycle?
                        // Or start triggers it.
                        // In TimeSurfaceBinning -> Readout:
                        // Cycle 1: Start=1, Valid=0 (because of latency logic I added).
                        // Cycle 2: Start=0, Valid=1, Data=Valid.
                        // So we should probably wait for 'feature_valid'?
                        // But existing systolic assumes continuous stream.
                        // Let's rely on synchronized start.
                        // If TimeSurfaceBinning pulses START one cycle BEFORE valid data, we need to handle that.
                        // Checked TSB: Start=1, Valid=0. Next cycle Valid=1.
                        // So we should transition to RUNNING but *wait* to latch data?
                        // Or just latch 0s?
                        
                        // Simplest fix: Just start accumulating.
                        // If TSB sends Data valid on Cycle 2.
                        // Cycle 1 (Idle->Run): Latch Data (Invalid/Z). PipeValid=0.
                        // Cycle 2 (Run): PipeValid=1? Wait, PipeValid reflects PREVIOUS latch.
                        // If I latch Garbage on Cycle 1, and process it on Cycle 2, Acc gets Garbage.
                        // So I need to align correctly.
                        
                        // Let's assume START coincides with FIRST VALID DATA.
                        // In TSB, I should align ReadoutStart with ReadoutValid?
                        // "readout_start <= 1'b1; ... readout_valid_d <= readout_busy"
                        // Valid comes 1 cycle later.
                        
                        state <= S_RUNNING;
                        // On IDLE->RUNNING transition, we prepare address 0.
                        w_addr <= '0; 
                    end
                end

                // -----------------------------------------------------------
                S_RUNNING: begin
                    // We need to carefully handle the pipeline
                    // w_addr updates 0->1->2
                    // ROM returns w[0]->w[1] (latency 1)
                    
                    // We latch feature_in every cycle.
                    feat_pipe_r  <= feature_in;
                    
                    // Pipe valid controls accumulation
                    // We need to skip the very first calculation if data wasn't valid?
                    // Let's use a counter.
                    
                    // Accumulation Logic
                    // PipeValid_r means "feat_pipe_r contains valid data AND ROM has valid data for it"
                    if (pipe_valid_r) begin
                         for (k = 0; k < NUM_CLASSES; k = k + 1)
                            acc[k] <= acc[k] +
                                ACC_BITS'($signed({1'b0, feat_pipe_r}) * $signed(w[k]));
                    end
                    
                    // Advance Control
                    if (feature_valid) begin
                         pipe_valid_r <= 1'b1; // Once data flows, pipe is full next cycle
                         
                         cell_cnt <= cell_cnt + 1'b1;
                         if (cell_cnt < CNT_BITS'(NUM_CELLS - 1)) begin
                             w_addr <= w_addr + 1'b1;
                         end
                         
                         if (cell_cnt == CNT_BITS'(NUM_CELLS - 1)) begin
                             state <= S_DRAIN;
                         end
                    end else begin
                        // If data isn't valid, we stall?
                        // TSB sends continuous stream once started.
                        // But the first cycle (Start) had Valid=0.
                        // So we sit in S_RUNNING, waiting for Valid=1?
                        // Yes, gating by feature_valid is safer.
                    end
                end

                // -----------------------------------------------------------
                S_DRAIN: begin
                    // Process the last latched value
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
    // Drive flat scores bus
    // -------------------------------------------------------------------------
    generate
        genvar gs;
        for (gs = 0; gs < NUM_CLASSES; gs = gs + 1) begin : gen_scores
            assign scores_flat[(gs+1)*ACC_BITS-1 : gs*ACC_BITS] = acc_r[gs];
        end
    endgenerate

endmodule
