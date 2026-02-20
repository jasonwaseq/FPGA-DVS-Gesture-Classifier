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
//                  For 4 bins * 256 cells = 1024.
//   VALUE_BITS   - Input feature bit width (6, reduced from 8)
//   WEIGHT_BITS  - Weight bit width (8)
//   ACC_BITS     - Accumulator width (24)
//   PARALLEL_INPUTS - Number of parallel feature inputs per cycle (4 for speed)
//
// =============================================================================

module SystolicMatrixMultiply #(
    parameter NUM_CLASSES = 4,
    // Default to 4 bins * 16x16 = 1024
    parameter NUM_CELLS   = 1024, 
    parameter VALUE_BITS  = 6,        // Reduced from 8 to match COUNTER_BITS
    parameter WEIGHT_BITS = 8,
    parameter ACC_BITS    = 24,
    parameter PARALLEL_INPUTS = 4     // Parallel inputs per cycle (speed optimization)
)(
    input  logic                                    clk,
    input  logic                                    rst,

    // Control
    input  logic                                    start,

    // Feature stream — parallel values per cycle (optimized)
    input  logic [PARALLEL_INPUTS*VALUE_BITS-1:0]   feature_in,    // Parallel feature data
    input  logic                                    feature_valid, // Optional qualification

    // Parallel weight ROM interface (flattened for port compatibility)
    output logic [PARALLEL_INPUTS*$clog2(NUM_CELLS)-1:0]            w_addr_flat,  // Flattened addresses
    input  logic [PARALLEL_INPUTS*NUM_CLASSES*WEIGHT_BITS-1:0]      w_data_flat,  // Flattened parallel weight data

    // Results
    output logic                                    result_valid,
    output logic [1:0]                              best_class,
    output logic [NUM_CLASSES*ACC_BITS-1:0]         scores_flat
);

    // =========================================================================
    // Constants
    // =========================================================================
    localparam CNT_BITS = $clog2(NUM_CELLS + 4);
    localparam PARALLEL_BITS = $clog2(PARALLEL_INPUTS);
    localparam CYCLES_NEEDED = (NUM_CELLS + PARALLEL_INPUTS - 1) / PARALLEL_INPUTS;  // Ceiling division

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
    logic [CNT_BITS-1:0] cell_cnt;   // Counts cycles (0..CYCLES_NEEDED-1)

    // Accumulators
    logic signed [ACC_BITS-1:0] acc   [0:NUM_CLASSES-1];
    logic signed [ACC_BITS-1:0] acc_r [0:NUM_CLASSES-1];

    // Feature pipeline (1-cycle delay to align with ROM latency)
    logic [PARALLEL_INPUTS*VALUE_BITS-1:0] feat_pipe_r;
    logic                                   pipe_valid_r;

    // Unpack addresses and weight data from flattened buses
    wire [$clog2(NUM_CELLS)-1:0] w_addr [0:PARALLEL_INPUTS-1];
    wire [NUM_CLASSES*WEIGHT_BITS-1:0] w_data [0:PARALLEL_INPUTS-1];
    
    generate
        genvar pa;
        for (pa = 0; pa < PARALLEL_INPUTS; pa = pa + 1) begin : gen_unpack_addr
            assign w_addr[pa] = w_addr_flat[(pa+1)*$clog2(NUM_CELLS)-1 : pa*$clog2(NUM_CELLS)];
            assign w_data[pa] = w_data_flat[(pa+1)*NUM_CLASSES*WEIGHT_BITS-1 : pa*NUM_CLASSES*WEIGHT_BITS];
        end
    endgenerate
    
    // Helper: extract per-class weights from parallel buses
    wire signed [WEIGHT_BITS-1:0] w [0:PARALLEL_INPUTS-1][0:NUM_CLASSES-1];
    generate
        genvar pi, gk;
        for (pi = 0; pi < PARALLEL_INPUTS; pi = pi + 1) begin : gen_parallel_weights
            for (gk = 0; gk < NUM_CLASSES; gk = gk + 1) begin : gen_w_unpack
                assign w[pi][gk] = $signed(w_data[pi][(gk+1)*WEIGHT_BITS-1 : gk*WEIGHT_BITS]);
            end
        end
    endgenerate
    
    // Extract parallel feature values
    wire signed [VALUE_BITS-1:0] feat_vals [0:PARALLEL_INPUTS-1];
    generate
        genvar fi;
        for (fi = 0; fi < PARALLEL_INPUTS; fi = fi + 1) begin : gen_feat_unpack
            assign feat_vals[fi] = $signed(feat_pipe_r[(fi+1)*VALUE_BITS-1 : fi*VALUE_BITS]);
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
            w_addr_flat  <= '0;
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
                        // Initialize parallel addresses (flattened)
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1)
                            w_addr_flat[(p+1)*$clog2(NUM_CELLS)-1 : p*$clog2(NUM_CELLS)] <= p;
                        state <= S_RUNNING;
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
                    
                    // Accumulation Logic - process all parallel inputs
                    if (pipe_valid_r) begin
                        // Process each parallel input
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1) begin
                            // Check if this parallel input is valid (not beyond NUM_CELLS)
                            if ((cell_cnt * PARALLEL_INPUTS + p) < NUM_CELLS) begin
                                for (k = 0; k < NUM_CLASSES; k = k + 1) begin
                                    acc[k] <= acc[k] +
                                        ACC_BITS'($signed({1'b0, feat_vals[p]}) * $signed(w[p][k]));
                                end
                            end
                        end
                    end
                    
                    // Advance Control
                    if (feature_valid) begin
                        pipe_valid_r <= 1'b1; // Once data flows, pipe is full next cycle
                        
                        cell_cnt <= cell_cnt + 1'b1;
                        // Update parallel addresses (flattened)
                        for (int p = 0; p < PARALLEL_INPUTS; p = p + 1) begin
                            if ((cell_cnt * PARALLEL_INPUTS + p + PARALLEL_INPUTS) < NUM_CELLS) begin
                                w_addr_flat[(p+1)*$clog2(NUM_CELLS)-1 : p*$clog2(NUM_CELLS)] <= (cell_cnt + 1) * PARALLEL_INPUTS + p;
                            end
                        end
                        
                        if (cell_cnt >= CNT_BITS'(CYCLES_NEEDED - 1)) begin
                            state <= S_DRAIN;
                        end
                    end else begin
                        // Stall if data not valid
                    end
                end

                // -----------------------------------------------------------
                S_DRAIN: begin
                    // All parallel values should have been processed in S_RUNNING
                    // Just transition to argmax
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
