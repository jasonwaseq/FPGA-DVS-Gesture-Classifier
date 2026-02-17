`timescale 1ns/1ps

// =============================================================================
// Module: feature_extractor
// =============================================================================
// Spatial Moment Computation and Linear Classifier
//
// Processing Pipeline:
//   1. Frame Pulse: Generated every FRAME_PERIOD_MS milliseconds
//   2. Scan: Iterate through all 256 time-surface cells
//   3. Compute: Accumulate spatial moments from decayed values
//   4. Classify: Dot-product of moments with weight matrix
//   5. Output: Argmax to determine gesture class
//
// Spatial Moments:
//   M_00 = Σ S(x,y)            - Total "energy" (mass)
//   M_10 = Σ x * S(x,y)        - X-weighted sum
//   M_01 = Σ y * S(x,y)        - Y-weighted sum
//   Centroid = (M_10/M_00, M_01/M_00)
//
// Grid Coordinate System:
//   Input: 320x320 sensor coordinates
//   Downsampling: bits [8:5] extract -> grid range [0-9]
//   Center: (4.5, 4.5) represented as (M00*9)>>1
//
// Linear Classifier:
//   Score[class] = Σ W[class][i] * M[i] + bias[class]
//   Output = argmax(Score)
//
// Gesture Classes:
//   0 = UP, 1 = DOWN, 2 = LEFT, 3 = RIGHT
// =============================================================================

module feature_extractor #(
    parameter CLK_FREQ_HZ     = 12_000_000,  // System clock frequency
    parameter FRAME_PERIOD_MS = 10,          // Frame period in milliseconds
    parameter GRID_SIZE       = 16,          // Grid dimension
    parameter VALUE_BITS      = 8,           // Surface value bits
    parameter MOMENT_BITS     = 24,          // Moment accumulator bits
    parameter WEIGHT_BITS     = 8,           // Weight precision (signed)
    parameter SCORE_BITS      = 24,          // Score accumulator bits
    parameter NUM_CLASSES     = 4,           // Number of gesture classes
    parameter MIN_MASS_THRESH = 100          // Minimum M_00 for valid gesture
)(
    input  logic                    clk,
    input  logic                    rst,
    
    // Time-Surface Read Interface
    output logic [7:0]              ts_read_addr,       // Address to read
    output logic                    ts_read_enable,     // Read enable
    input  logic [VALUE_BITS-1:0]   ts_read_value,      // Decayed value
    
    // Classification Output
    output logic [1:0]              gesture_class,      // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    output logic                    gesture_valid,      // Valid classification
    output logic [7:0]              gesture_confidence, // Confidence (M_00 scaled)
    
    // Debug Outputs
    output logic [MOMENT_BITS-1:0]  debug_m00,          // Total mass
    output logic [MOMENT_BITS-1:0]  debug_m10,          // X moment
    output logic [MOMENT_BITS-1:0]  debug_m01,          // Y moment
    output logic [2:0]              debug_state         // FSM state
);

    // -------------------------------------------------------------------------
    // Frame Period Counter
    // -------------------------------------------------------------------------
    localparam FRAME_CYCLES = (CLK_FREQ_HZ / 1000) * FRAME_PERIOD_MS;
    localparam FRAME_CNT_BITS = $clog2(FRAME_CYCLES + 1);
    
    logic [FRAME_CNT_BITS-1:0] frame_counter;
    logic frame_pulse;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            frame_counter <= '0;
            frame_pulse <= 1'b0;
        end else begin
            if (frame_counter >= FRAME_CYCLES - 1) begin
                frame_counter <= '0;
                frame_pulse <= 1'b1;
            end else begin
                frame_counter <= frame_counter + 1'b1;
                frame_pulse <= 1'b0;
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE      = 3'd0,     // Wait for frame pulse
        S_SCAN_WAIT = 3'd1,     // Wait for BRAM read latency
        S_SCAN      = 3'd2,     // Iterate through all cells
        S_COMPUTE   = 3'd3,     // Finalize moments
        S_CLASSIFY  = 3'd4,     // Compute classification scores
        S_OUTPUT    = 3'd5      // Output result
    } state_t;
    
    state_t state, next_state;
    
    // -------------------------------------------------------------------------
    // Scan Counter
    // -------------------------------------------------------------------------
    logic [8:0] scan_addr;      // 0-255 + 1 for end detection
    logic [3:0] scan_x, scan_y; // Current cell coordinates
    
    assign scan_x = scan_addr[3:0];
    assign scan_y = scan_addr[7:4];
    
    // -------------------------------------------------------------------------
    // Moment Accumulators
    // -------------------------------------------------------------------------
    logic [MOMENT_BITS-1:0] m00_acc;    // Σ value
    logic [MOMENT_BITS-1:0] m10_acc;    // Σ x * value
    logic [MOMENT_BITS-1:0] m01_acc;    // Σ y * value
    
    // -------------------------------------------------------------------------
    // Weight Storage (40 weights: 10 features × 4 classes)
    // For simplicity, using fixed weights (can be made programmable via SRAM)
    // 
    // Features: [M00, M10, M01, Cx, Cy, Cx-8, Cy-8, |Cx-8|, |Cy-8|, 1]
    // Simplified: Using just M00, centroid offsets for direction detection
    //
    // Weight encoding (signed 8-bit):
    //   Positive weight = feature contributes to class
    //   Negative weight = feature detracts from class
    // -------------------------------------------------------------------------
    
    // Simplified classifier using centroid displacement from center
    // UP:    negative Y displacement (centroid above center)
    // DOWN:  positive Y displacement (centroid below center)
    // LEFT:  negative X displacement (centroid left of center)
    // RIGHT: positive X displacement (centroid right of center)
    
    logic signed [SCORE_BITS-1:0] score_up, score_down, score_left, score_right;
    logic signed [MOMENT_BITS-1:0] centroid_x, centroid_y;
    logic signed [MOMENT_BITS-1:0] offset_x, offset_y;
    
    // -------------------------------------------------------------------------
    // FSM Sequential Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            scan_addr     <= '0;
            m00_acc       <= '0;
            m10_acc       <= '0;
            m01_acc       <= '0;
            gesture_class <= '0;
            gesture_valid <= 1'b0;
            gesture_confidence <= '0;
            ts_read_addr  <= '0;
            ts_read_enable <= 1'b0;
            score_up      <= '0;
            score_down    <= '0;
            score_left    <= '0;
            score_right   <= '0;
            centroid_x    <= '0;
            centroid_y    <= '0;
            offset_x      <= '0;
            offset_y      <= '0;
        end else begin
            // Default outputs
            gesture_valid <= 1'b0;
            
            case (state)
                // ---------------------------------------------------------
                // IDLE: Wait for frame pulse
                // ---------------------------------------------------------
                S_IDLE: begin
                    if (frame_pulse) begin
                        // Initialize accumulators
                        m00_acc    <= '0;
                        m10_acc    <= '0;
                        m01_acc    <= '0;
                        scan_addr  <= '0;
                        
                        // Start first read
                        ts_read_addr <= '0;
                        ts_read_enable <= 1'b1;
                        state <= S_SCAN_WAIT;
                    end
                end
                
                // ---------------------------------------------------------
                // SCAN_WAIT: Wait one cycle for BRAM read latency
                // ---------------------------------------------------------
                S_SCAN_WAIT: begin
                    ts_read_enable <= 1'b1;
                    state <= S_SCAN;
                end
                
                // ---------------------------------------------------------
                // SCAN: Read and accumulate moments for all 256 cells
                // ---------------------------------------------------------
                S_SCAN: begin
                    // Accumulate moments from previous read
                    m00_acc <= m00_acc + ts_read_value;
                    m10_acc <= m10_acc + (scan_x * ts_read_value);
                    m01_acc <= m01_acc + (scan_y * ts_read_value);
                    
                    // Advance to next cell
                    if (scan_addr == 8'd255) begin
                        // Done scanning
                        ts_read_enable <= 1'b0;
                        state <= S_COMPUTE;
                    end else begin
                        scan_addr <= scan_addr + 1'b1;
                        ts_read_addr <= scan_addr[7:0] + 1'b1;
                        ts_read_enable <= 1'b1;
                    end
                end
                
                // ---------------------------------------------------------
                // COMPUTE: Calculate centroid from moments
                // ---------------------------------------------------------
                S_COMPUTE: begin
                    // Compute centroid (avoid division, use scaled comparison)
                    // centroid_x = M10 / M00, centroid_y = M01 / M00
                    // Grid coordinates range 0-9 (from bit slice [8:5] of 320 pixels)
                    // Grid center is at approximately (4.5, 4.5)
                    // Use M00*4.5 ≈ M00*9/2 for center reference
                    
                    // Store offset from center: (M10 - M00*4.5, M01 - M00*4.5)
                    // Using fixed-point: M00*4.5 = (M00*9) >> 1
                    // Positive offset_x means centroid is right of center
                    // Positive offset_y means centroid is below center
                    offset_x <= $signed(m10_acc) - $signed((m00_acc * 5'd9) >> 1);
                    offset_y <= $signed(m01_acc) - $signed((m00_acc * 5'd9) >> 1);
                    
                    state <= S_CLASSIFY;
                end
                
                // ---------------------------------------------------------
                // CLASSIFY: Compute scores for each gesture class
                // ---------------------------------------------------------
                S_CLASSIFY: begin
                    // Simple direction-based classification
                    // UP:    large negative Y offset (centroid above center)
                    // DOWN:  large positive Y offset (centroid below center)  
                    // LEFT:  large negative X offset (centroid left of center)
                    // RIGHT: large positive X offset (centroid right of center)
                    
                    score_up    <= -offset_y;   // Negative Y = UP
                    score_down  <= offset_y;    // Positive Y = DOWN
                    score_left  <= -offset_x;   // Negative X = LEFT
                    score_right <= offset_x;    // Positive X = RIGHT
                    
                    state <= S_OUTPUT;
                end
                
                // ---------------------------------------------------------
                // OUTPUT: Determine winner and output result
                // ---------------------------------------------------------
                S_OUTPUT: begin
                    // Check minimum mass threshold
                    if (m00_acc >= MIN_MASS_THRESH) begin
                        gesture_valid <= 1'b1;
                        
                        // Confidence based on total mass (scaled)
                        if (m00_acc > 24'd65535)
                            gesture_confidence <= 8'd255;
                        else
                            gesture_confidence <= m00_acc[15:8];
                        
                        // Argmax: Find class with highest score
                        // Use unbiased comparison by checking all pairs
                        if (score_up > score_down && 
                            score_up > score_left && 
                            score_up > score_right) begin
                            gesture_class <= 2'd0;  // UP
                        end else if (score_down > score_up && 
                                     score_down > score_left && 
                                     score_down > score_right) begin
                            gesture_class <= 2'd1;  // DOWN
                        end else if (score_left > score_up && 
                                     score_left > score_down && 
                                     score_left > score_right) begin
                            gesture_class <= 2'd2;  // LEFT
                        end else if (score_right > score_up && 
                                     score_right > score_down && 
                                     score_right > score_left) begin
                            gesture_class <= 2'd3;  // RIGHT
                        end else begin
                            // Tie-break: use magnitude comparison
                            // Determine dominant axis and direction using absolute values
                            if (((offset_y < 0) ? -offset_y : offset_y) > 
                                ((offset_x < 0) ? -offset_x : offset_x)) begin
                                // Y axis is dominant
                                gesture_class <= (offset_y < 0) ? 2'd0 : 2'd1;  // UP or DOWN
                            end else begin
                                // X axis is dominant (or equal)
                                gesture_class <= (offset_x < 0) ? 2'd2 : 2'd3;  // LEFT or RIGHT
                            end
                        end
                    end
                    
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
    
    // -------------------------------------------------------------------------
    // Debug Outputs
    // -------------------------------------------------------------------------
    assign debug_m00   = m00_acc;
    assign debug_m10   = m10_acc;
    assign debug_m01   = m01_acc;
    assign debug_state = state;

endmodule
