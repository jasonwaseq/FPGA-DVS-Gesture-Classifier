// =============================================================================
// Module: MotionComputer
// =============================================================================
// Computes motion vector from early/late window centroid differences
//
// Algorithm (integer arithmetic):
//   Δx = late_sum_x - early_sum_x
//   Δy = late_sum_y - early_sum_y
//   |Δx| = abs(Δx)
//   |Δy| = abs(Δy)
//   total = early_count + late_count
//
// Interpretation of Motion Vectors:
//   Positive Δx → Motion to the RIGHT
//   Negative Δx → Motion to the LEFT
//   Positive Δy → Motion DOWNWARD (Y increases downward in image coords)
//   Negative Δy → Motion UPWARD
//
// Pipeline:
//   Stage 1: Compute differences (subtraction)
//   Stage 2: Compute absolute values (for magnitude comparison)
//
// Note: Using sum differences rather than true centroid differences
// (sum/count) is valid when event counts are similar between windows.
// For significantly different counts, normalization would improve accuracy.
// =============================================================================

module MotionComputer #(
    parameter ACC_SUM_BITS   = 18,               // Width of sum accumulators (signed)
    parameter ACC_COUNT_BITS = 12                // Width of count accumulators (unsigned)
)(
    input  logic        clk,
    input  logic        rst,
    
    // Input from TemporalAccumulator
    input  logic        trigger,                 // Start computation
    input  logic signed [ACC_SUM_BITS-1:0] early_sum_x,  // Σx early window
    input  logic signed [ACC_SUM_BITS-1:0] early_sum_y,  // Σy early window
    input  logic [ACC_COUNT_BITS-1:0]      early_count,  // Count early window
    input  logic signed [ACC_SUM_BITS-1:0] late_sum_x,   // Σx late window
    input  logic signed [ACC_SUM_BITS-1:0] late_sum_y,   // Σy late window
    input  logic [ACC_COUNT_BITS-1:0]      late_count,   // Count late window
    
    // Output to GestureClassifier
    output logic signed [ACC_SUM_BITS-1:0] delta_x,      // Motion vector X (signed)
    output logic signed [ACC_SUM_BITS-1:0] delta_y,      // Motion vector Y (signed)
    output logic [ACC_SUM_BITS-1:0]        abs_delta_x,  // |Δx| (unsigned)
    output logic [ACC_SUM_BITS-1:0]        abs_delta_y,  // |Δy| (unsigned)
    output logic [ACC_COUNT_BITS-1:0]      total_events, // Total event count
    output logic                           valid         // Output valid pulse
);

    // -------------------------------------------------------------------------
    // Pipeline Stage 1: Compute differences
    // -------------------------------------------------------------------------
    logic signed [ACC_SUM_BITS-1:0] diff_x, diff_y;
    logic [ACC_COUNT_BITS-1:0]      sum_count;
    logic                           stage1_valid;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            diff_x       <= '0;
            diff_y       <= '0;
            sum_count    <= '0;
            stage1_valid <= 1'b0;
        end else begin
            stage1_valid <= trigger;
            if (trigger) begin
                // Signed subtraction: late - early
                diff_x    <= late_sum_x - early_sum_x;
                diff_y    <= late_sum_y - early_sum_y;
                // Unsigned addition for total count
                sum_count <= early_count + late_count;
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Pipeline Stage 2: Compute absolute values
    // -------------------------------------------------------------------------
    // Compute absolute values using conditional assignment
    
    wire [ACC_SUM_BITS-1:0] abs_diff_x;
    wire [ACC_SUM_BITS-1:0] abs_diff_y;
    
    // Compute absolute values combinatorially
    // Absolute value is always non-negative, so must be UNSIGNED
    assign abs_diff_x = (diff_x < 0) ? (-diff_x) : diff_x;
    assign abs_diff_y = (diff_y < 0) ? (-diff_y) : diff_y;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            delta_x      <= '0;
            delta_y      <= '0;
            abs_delta_x  <= '0;
            abs_delta_y  <= '0;
            total_events <= '0;
            valid        <= 1'b0;
        end else begin
            valid <= stage1_valid;
            if (stage1_valid) begin
                // Pass through signed deltas
                delta_x      <= diff_x;
                delta_y      <= diff_y;
                // Register the computed absolute values
                abs_delta_x  <= abs_diff_x;
                abs_delta_y  <= abs_diff_y;
                // Pass through total count
                total_events <= sum_count;
            end
        end
    end

endmodule
