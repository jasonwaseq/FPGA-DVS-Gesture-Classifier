// =============================================================================
// Module: OutputRegister
// =============================================================================
// Applies persistence filter and generates final gesture output
//
// Persistence Filter:
//   - Tracks consecutive windows with same classification
//   - Only asserts gesture_valid after PERSISTENCE_COUNT matches
//   - Resets count when classification changes or gates fail
//
// Purpose:
//   - Suppresses spurious single-frame detections
//   - Requires temporal consistency for reliable output
//   - Reduces false positives from noise or transient motion
//
// Confidence Calculation:
//   - Maps dominant motion magnitude to 4-bit confidence [0-15]
//   - confidence = min(15, magnitude >> 4)
//   - Higher values indicate stronger, more definitive motion
//
// State Machine:
//   ST_IDLE      - No consistent classification yet
//   ST_TRACKING  - Same gesture seen, building confidence
//   ST_CONFIRMED - Gesture confirmed, output asserted
// =============================================================================

module OutputRegister #(
    parameter ACC_SUM_BITS      = 18,            // Motion magnitude width
    parameter PERSISTENCE_COUNT = 2              // Consecutive matches required
)(
    input  logic        clk,
    input  logic        rst,
    
    // Input from GestureClassifier
    input  logic [1:0]  class_gesture,           // Classified gesture code
    input  logic        class_valid,             // Classification complete pulse
    input  logic        class_pass,              // Passed activity & motion gates
    input  logic [ACC_SUM_BITS-1:0] abs_delta_x, // |Δx| for confidence calc
    input  logic [ACC_SUM_BITS-1:0] abs_delta_y, // |Δy| for confidence calc
    
    // Final output
    output logic [1:0]  gesture,                 // Confirmed gesture code
    output logic        gesture_valid,           // Single-cycle pulse on confirmation
    output logic [3:0]  gesture_confidence,      // Confidence level [0-15]
    
    // Debug
    output logic [2:0]  debug_state              // Current FSM state
);

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE      = 3'd0,                     // No consistent classification
        ST_TRACKING  = 3'd1,                     // Building persistence count
        ST_CONFIRMED = 3'd2                      // Gesture confirmed
    } state_t;
    
    state_t state;
    assign debug_state = state;
    
    // -------------------------------------------------------------------------
    // Persistence tracking
    // -------------------------------------------------------------------------
    logic [1:0] last_gesture;                    // Previous classification
    logic [2:0] match_count;                     // Consecutive match counter
    
    // Latched motion magnitudes for confidence calculation
    logic [ACC_SUM_BITS-1:0] latched_abs_x, latched_abs_y;
    
    // -------------------------------------------------------------------------
    // Persistence Filter Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state              <= ST_IDLE;
            last_gesture       <= '0;
            match_count        <= '0;
            gesture            <= '0;
            gesture_valid      <= 1'b0;
            gesture_confidence <= '0;
            latched_abs_x      <= '0;
            latched_abs_y      <= '0;
        end else begin
            // Default: no output pulse
            gesture_valid <= 1'b0;
            
            if (class_valid) begin
                if (class_pass) begin
                    // ---------------------------------------------------------
                    // Valid classification that passed activity/motion gates
                    // ---------------------------------------------------------
                    
                    if (class_gesture == last_gesture) begin
                        // Same gesture as previous window
                        if (match_count < PERSISTENCE_COUNT) begin
                            match_count <= match_count + 1'b1;
                        end
                        
                        // Check if persistence threshold reached
                        if (match_count >= PERSISTENCE_COUNT - 1) begin
                            // Gesture confirmed!
                            state              <= ST_CONFIRMED;
                            gesture            <= class_gesture;
                            gesture_valid      <= 1'b1;
                            match_count        <= '0;  // Reset for next gesture
                            
                            // -------------------------------------------------
                            // Confidence Calculation
                            // Map dominant magnitude to [0-15] range
                            // confidence = min(15, magnitude >> 4)
                            // -------------------------------------------------
                            if (abs_delta_x > abs_delta_y) begin
                                // Horizontal dominant
                                gesture_confidence <= (abs_delta_x > 255) ? 4'd15 : abs_delta_x[7:4];
                            end else begin
                                // Vertical dominant
                                gesture_confidence <= (abs_delta_y > 255) ? 4'd15 : abs_delta_y[7:4];
                            end
                        end else begin
                            state <= ST_TRACKING;
                        end
                    end else begin
                        // Different classification - start new tracking
                        last_gesture <= class_gesture;
                        match_count  <= '0;
                        state        <= ST_TRACKING;
                    end
                    
                    // Latch magnitudes for potential confidence calc
                    latched_abs_x <= abs_delta_x;
                    latched_abs_y <= abs_delta_y;
                    
                end else begin
                    // ---------------------------------------------------------
                    // Classification failed gates - reset persistence
                    // ---------------------------------------------------------
                    match_count <= '0;
                    state       <= ST_IDLE;
                end
            end
        end
    end

endmodule
