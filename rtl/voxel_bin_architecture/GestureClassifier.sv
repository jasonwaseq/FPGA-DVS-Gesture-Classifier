// =============================================================================
// Module: GestureClassifier
// =============================================================================
// Classifies motion vector into cardinal gesture direction
//
// Decision Logic:
//   1. Activity gate: Reject if total_events < MIN_EVENT_THRESH
//   2. Motion gate: Reject if max(|Δx|, |Δy|) < MOTION_THRESH
//   3. Dominant axis: |Δy| > |Δx| → vertical, else horizontal
//   4. Direction: Sign of dominant component determines gesture
//
// Decision Tree:
//   IF events < threshold → NO GESTURE (noise rejection)
//   ELIF motion < threshold → NO GESTURE (stationary)
//   ELIF |Δy| > |Δx| → VERTICAL dominant
//       IF Δy > 0 → DOWN (motion toward bottom of image)
//       ELSE → UP (motion toward top of image)
//   ELSE → HORIZONTAL dominant
//       IF Δx > 0 → RIGHT (motion toward right of image)
//       ELSE → LEFT (motion toward left of image)
//
// Gesture Encoding:
//   GESTURE_UP    = 2'b00  (Δy < 0, vertical dominant)
//   GESTURE_DOWN  = 2'b01  (Δy > 0, vertical dominant)
//   GESTURE_LEFT  = 2'b10  (Δx < 0, horizontal dominant)
//   GESTURE_RIGHT = 2'b11  (Δx > 0, horizontal dominant)
// =============================================================================

module GestureClassifier #(
    parameter ACC_SUM_BITS     = 18,             // Motion vector width (signed)
    parameter ACC_COUNT_BITS   = 12,             // Event count width (unsigned)
    parameter MIN_EVENT_THRESH = 20,             // Min events for valid gesture [0-4095]
    parameter MOTION_THRESH    = 8               // Min motion magnitude for detection
)(
    input  logic        clk,
    input  logic        rst,
    
    // Input from MotionComputer
    input  logic        trigger,                 // Start classification
    input  logic signed [ACC_SUM_BITS-1:0] delta_x,      // Motion vector X (signed)
    input  logic signed [ACC_SUM_BITS-1:0] delta_y,      // Motion vector Y (signed)
    input  logic [ACC_SUM_BITS-1:0]        abs_delta_x,  // |Δx| (unsigned)
    input  logic [ACC_SUM_BITS-1:0]        abs_delta_y,  // |Δy| (unsigned)
    input  logic [ACC_COUNT_BITS-1:0]      total_events, // Total event count
    
    // Output to OutputRegister
    output logic [1:0]  gesture,                 // Classified gesture code
    output logic        valid,                   // Classification complete pulse
    output logic        pass                     // Passed activity & motion gates
);

    // -------------------------------------------------------------------------
    // Gesture codes (matches top-level encoding)
    // -------------------------------------------------------------------------
    localparam logic [1:0] GESTURE_UP    = 2'b00;  // Motion toward top
    localparam logic [1:0] GESTURE_DOWN  = 2'b01;  // Motion toward bottom
    localparam logic [1:0] GESTURE_LEFT  = 2'b10;  // Motion toward left
    localparam logic [1:0] GESTURE_RIGHT = 2'b11;  // Motion toward right
    
    // -------------------------------------------------------------------------
    // Classification Logic (single-cycle decision)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            gesture <= '0;
            valid   <= 1'b0;
            pass    <= 1'b0;
        end else begin
            valid <= trigger;
            pass  <= 1'b0;
            
            if (trigger) begin
                // ---------------------------------------------------------
                // Gate 1: Activity threshold
                // Require minimum number of events to suppress noise
                // ---------------------------------------------------------
                if (total_events >= MIN_EVENT_THRESH) begin
                    // -----------------------------------------------------
                    // Gate 2: Motion threshold
                    // Require minimum motion magnitude to suppress jitter
                    // -----------------------------------------------------
                    if ((abs_delta_x >= MOTION_THRESH) || (abs_delta_y >= MOTION_THRESH)) begin
                        pass <= 1'b1;
                        
                        // -------------------------------------------------
                        // Dominant Axis Classification
                        // Compare magnitudes to determine primary direction
                        // -------------------------------------------------
                        if (abs_delta_y > abs_delta_x) begin
                            // Vertical motion dominant
                            // Positive Δy means centroid moved downward
                            gesture <= (delta_y > 0) ? GESTURE_DOWN : GESTURE_UP;
                        end else begin
                            // Horizontal motion dominant (or equal)
                            // Positive Δx means centroid moved rightward
                            gesture <= (delta_x > 0) ? GESTURE_RIGHT : GESTURE_LEFT;
                        end
                    end
                    // else: pass remains 0, gesture unchanged
                end
                // else: pass remains 0, gesture unchanged
            end
        end
    end

endmodule
