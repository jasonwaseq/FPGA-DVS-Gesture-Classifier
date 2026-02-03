// =============================================================================
// DVS Gesture Accelerator - Asynchronous Spatiotemporal Motion-Energy Classifier
// =============================================================================
//
// Architecture Overview:
//   - Event-driven asynchronous processing pipeline
//   - InputFIFO: 16-entry shallow buffer for bursty DVS events
//   - SpatialCompressor: 320×320 → 16×16 grid mapping (shift-based, no dividers)
//   - TemporalAccumulator: Dual early/late window centroid tracking
//   - MotionComputer: Vector extraction (Δx, Δy) from centroid differences
//   - GestureClassifier: Threshold-based cardinal direction classification
//   - OutputRegister: Persistence filter and confidence computation
//
// Signal Conventions:
//   - All coordinates are UNSIGNED integers unless marked 'signed'
//   - Polarity: 1 = ON (brightness increase), 0 = OFF (brightness decrease)
//   - Grid positions are SIGNED relative to center (range: -8 to +7 for 16×16)
//   - All arithmetic uses fixed-point integers (no floating-point)
//
// Optimized for iCE40 UP5K FPGA with minimal resource usage (0 BRAM)
// =============================================================================

module dvs_gesture_accel #(
    // =========================================================================
    // Configuration Parameters
    // =========================================================================
    parameter CLK_FREQ_HZ        = 12_000_000,  // System clock frequency (Hz)
    parameter WINDOW_MS          = 400,          // Observation window duration (ms)
    parameter GRID_SIZE          = 16,           // Compressed spatial grid dimension
    parameter SENSOR_RES         = 320,          // Native DVS sensor resolution (pixels)
    parameter FIFO_DEPTH         = 16,           // Input FIFO depth (entries)
    parameter MIN_EVENT_THRESH   = 20,           // Min events for valid gesture [0-4095]
    parameter MOTION_THRESH      = 8,            // Min motion magnitude for detection
    parameter PERSISTENCE_COUNT  = 2             // Consecutive windows for stable output
)(
    // =========================================================================
    // Clock and Reset
    // =========================================================================
    input  logic        clk,                     // System clock (12 MHz nominal)
    input  logic        rst,                     // Synchronous reset, active-high
    
    // =========================================================================
    // DVS Event Input Interface (Ready/Valid Handshaking)
    // =========================================================================
    input  logic        event_valid,             // Event data valid (producer asserts)
    input  logic [8:0]  event_x,                 // X coordinate [0-319], 9-bit unsigned
    input  logic [8:0]  event_y,                 // Y coordinate [0-319], 9-bit unsigned
    input  logic        event_polarity,          // 1=ON (brighter), 0=OFF (darker)
    input  logic [15:0] event_ts,                // Timestamp (optional, for debug)
    output logic        event_ready,             // Backpressure: 1=can accept, 0=full
    
    // =========================================================================
    // Gesture Output Interface
    // =========================================================================
    output logic [1:0]  gesture,                 // Detected gesture code:
                                                 //   2'b00 = UP    (negative Δy)
                                                 //   2'b01 = DOWN  (positive Δy)
                                                 //   2'b10 = LEFT  (negative Δx)
                                                 //   2'b11 = RIGHT (positive Δx)
    output logic        gesture_valid,           // Single-cycle pulse when gesture detected
    output logic [3:0]  gesture_confidence,      // Confidence level [0-15], higher=stronger
    
    // =========================================================================
    // Debug Interface
    // =========================================================================
    output logic [7:0]  debug_event_count,       // Events processed in current window
    output logic [2:0]  debug_state,             // Current FSM state
    output logic        debug_fifo_empty,        // InputFIFO empty flag
    output logic        debug_fifo_full,         // InputFIFO full flag
    output logic        debug_temporal_phase     // Current temporal window phase (0=early, 1=late)
);

    // =========================================================================
    // Local Parameters (Derived Constants)
    // =========================================================================
    
    // Timing calculations for sliding window
    // Half-window duration in clock cycles: (CLK_FREQ_HZ / 1000) * (WINDOW_MS / 2)
    localparam integer HALF_WINDOW_MS   = WINDOW_MS / 2;
    localparam integer CYCLES_PER_HALF  = (CLK_FREQ_HZ / 1000) * HALF_WINDOW_MS;
    localparam integer TIMER_BITS       = $clog2(CYCLES_PER_HALF + 1);  // Timer width
    
    // Grid and FIFO sizing
    localparam integer GRID_BITS        = $clog2(GRID_SIZE);            // 4 bits for 16×16
    localparam integer FIFO_PTR_BITS    = $clog2(FIFO_DEPTH);           // 4 bits for 16 entries
    
    // Accumulator bit widths (sized for up to 4096 events per half-window)
    // Sum bits: signed position [-8,+7] * 4096 events = [-32768, +28672], needs 18 bits signed
    localparam integer ACC_COUNT_BITS   = 12;   // Event counter: [0-4095]
    localparam integer ACC_SUM_BITS     = 18;   // Position sums: signed [-131072, +131071]
    
    // Gesture encoding constants
    localparam logic [1:0] GESTURE_UP    = 2'b00;  // Motion toward top    (Δy < 0)
    localparam logic [1:0] GESTURE_DOWN  = 2'b01;  // Motion toward bottom (Δy > 0)
    localparam logic [1:0] GESTURE_LEFT  = 2'b10;  // Motion toward left   (Δx < 0)
    localparam logic [1:0] GESTURE_RIGHT = 2'b11;  // Motion toward right  (Δx > 0)

    // =========================================================================
    // Internal Signal Declarations
    // =========================================================================
    
    // InputFIFO → SpatialCompressor interface
    logic                      fifo_pop;           // Pop request from compressor
    logic                      fifo_empty;         // FIFO empty status
    logic                      fifo_full;          // FIFO full status
    logic [8:0]                fifo_out_x;         // Buffered X coordinate [0-319]
    logic [8:0]                fifo_out_y;         // Buffered Y coordinate [0-319]
    logic                      fifo_out_pol;       // Buffered polarity
    logic [15:0]               fifo_out_ts;        // Buffered timestamp
    
    // SpatialCompressor → TemporalAccumulator interface
    logic signed [GRID_BITS:0] compressed_x;       // Grid-relative X [-8 to +7], signed
    logic signed [GRID_BITS:0] compressed_y;       // Grid-relative Y [-8 to +7], signed
    logic                      compressed_pol;     // Polarity passthrough
    logic                      compressed_valid;   // Compressed data valid pulse
    
    // Window timing signals (from TemporalAccumulator to other modules)
    logic                      window_phase;       // 0=early half, 1=late half
    logic                      window_complete;    // Pulse when full window completes
    
    // TemporalAccumulator → MotionComputer interface
    logic signed [ACC_SUM_BITS-1:0] early_sum_x;   // Early window: Σ(x positions), signed
    logic signed [ACC_SUM_BITS-1:0] early_sum_y;   // Early window: Σ(y positions), signed
    logic [ACC_COUNT_BITS-1:0]      early_count;   // Early window: event count
    logic signed [ACC_SUM_BITS-1:0] late_sum_x;    // Late window: Σ(x positions), signed
    logic signed [ACC_SUM_BITS-1:0] late_sum_y;    // Late window: Σ(y positions), signed
    logic [ACC_COUNT_BITS-1:0]      late_count;    // Late window: event count
    logic                           accum_valid;   // Accumulator snapshot ready
    
    // MotionComputer → GestureClassifier interface
    logic signed [ACC_SUM_BITS-1:0] delta_x;       // Motion vector X: late_sum - early_sum
    logic signed [ACC_SUM_BITS-1:0] delta_y;       // Motion vector Y: late_sum - early_sum
    logic [ACC_SUM_BITS-1:0]        abs_delta_x;   // |Δx|, unsigned magnitude
    logic [ACC_SUM_BITS-1:0]        abs_delta_y;   // |Δy|, unsigned magnitude
    logic [ACC_COUNT_BITS-1:0]      total_events;  // Total events in window
    logic                           motion_valid;  // Motion computation complete
    
    // GestureClassifier → OutputRegister interface
    logic [1:0]                     class_gesture; // Classified gesture code
    logic                           class_valid;   // Classification valid
    logic                           class_pass;    // Passed activity/motion gates

    // =========================================================================
    // Module: InputFIFO
    // =========================================================================
    // Purpose: Buffer incoming DVS events to handle bursty arrival patterns
    // 
    // Behavior:
    //   - Asynchronous push when event_valid & !full (event-driven)
    //   - Synchronous pop when downstream requests data
    //   - Circular buffer with wrap-around pointers
    //
    // Data Format (35 bits per entry):
    //   [34:26] = x[8:0]      - X coordinate, unsigned, range [0-319]
    //   [25:17] = y[8:0]      - Y coordinate, unsigned, range [0-319]
    //   [16]    = polarity    - 1=ON (brightness increase), 0=OFF (decrease)
    //   [15:0]  = timestamp   - Optional timing information
    // =========================================================================
    
    InputFIFO #(
        .DEPTH(FIFO_DEPTH),
        .PTR_BITS(FIFO_PTR_BITS)
    ) u_input_fifo (
        .clk            (clk),
        .rst            (rst),
        // Push interface (from external DVS source)
        .push_valid     (event_valid),
        .push_x         (event_x),          // [8:0] unsigned, 0-319
        .push_y         (event_y),          // [8:0] unsigned, 0-319
        .push_polarity  (event_polarity),   // 1=ON, 0=OFF
        .push_ts        (event_ts),         // [15:0] timestamp
        .push_ready     (event_ready),      // Backpressure output
        // Pop interface (to SpatialCompressor)
        .pop_req        (fifo_pop),
        .pop_x          (fifo_out_x),
        .pop_y          (fifo_out_y),
        .pop_polarity   (fifo_out_pol),
        .pop_ts         (fifo_out_ts),
        // Status
        .empty          (fifo_empty),
        .full           (fifo_full)
    );
    
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;
    assign debug_temporal_phase = window_phase;

    // =========================================================================
    // Module: SpatialCompressor
    // =========================================================================
    // Purpose: Map 320×320 sensor coordinates to 16×16 grid
    //
    // Algorithm (fixed-point integer arithmetic):
    //   grid_coord = (sensor_coord × 13) >> 8
    //   This approximates sensor_coord / 20 (since 320/16 = 20)
    //   Error: max 0.6% compared to true division
    //
    // Output coordinates are SIGNED, relative to grid center:
    //   Range: [-8, +7] for 16×16 grid (center at grid position 8)
    //   This enables direct use in signed centroid accumulation
    //
    // Polarity Handling:
    //   - ON events (pol=1): Typically indicate leading edge of motion
    //   - OFF events (pol=0): Typically indicate trailing edge of motion
    //   - Both polarities contribute to centroid calculation
    // =========================================================================
    
    SpatialCompressor #(
        .SENSOR_RES     (SENSOR_RES),       // 320 pixels
        .GRID_SIZE      (GRID_SIZE),        // 16 grid cells
        .GRID_BITS      (GRID_BITS)         // 4 bits per coordinate
    ) u_spatial_compressor (
        .clk            (clk),
        .rst            (rst),
        // Input from FIFO (event-triggered)
        .in_valid       (!fifo_empty),
        .in_x           (fifo_out_x),       // [8:0] unsigned, 0-319
        .in_y           (fifo_out_y),       // [8:0] unsigned, 0-319
        .in_polarity    (fifo_out_pol),     // 1=ON, 0=OFF
        .in_ready       (fifo_pop),         // Triggers FIFO pop
        // Output to TemporalAccumulator
        .out_x          (compressed_x),     // [4:0] signed, -8 to +7
        .out_y          (compressed_y),     // [4:0] signed, -8 to +7
        .out_polarity   (compressed_pol),
        .out_valid      (compressed_valid)
    );

    // =========================================================================
    // Module: TemporalAccumulator
    // =========================================================================
    // Purpose: Maintain running centroid sums for early/late window halves
    //
    // Accumulator Contents (per window half):
    //   sum_x:  Σ(grid_x) for all events, signed 18-bit
    //   sum_y:  Σ(grid_y) for all events, signed 18-bit
    //   count:  Number of events, unsigned 12-bit
    //
    // Polarity Separation:
    //   Maintains separate accumulators for ON and OFF events
    //   This enables polarity-weighted motion analysis if needed
    //
    // Window Sliding:
    //   At window completion: early ← late, late ← 0
    //   This implements a continuously sliding observation window
    //
    // Fixed-Point Arithmetic:
    //   Position values: Q0.4 (4 fractional bits implied by grid mapping)
    //   Sum values: Q0.4 accumulated (actual centroid = sum/count)
    // =========================================================================
    
    TemporalAccumulator #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .HALF_WINDOW_MS (HALF_WINDOW_MS),
        .CYCLES_PER_HALF(CYCLES_PER_HALF),
        .TIMER_BITS     (TIMER_BITS),
        .GRID_BITS      (GRID_BITS),
        .ACC_SUM_BITS   (ACC_SUM_BITS),
        .ACC_COUNT_BITS (ACC_COUNT_BITS)
    ) u_temporal_accumulator (
        .clk            (clk),
        .rst            (rst),
        // Input from SpatialCompressor (event-driven)
        .event_valid    (compressed_valid),
        .event_x        (compressed_x),     // [4:0] signed, -8 to +7
        .event_y        (compressed_y),     // [4:0] signed, -8 to +7
        .event_polarity (compressed_pol),
        // Window timing outputs
        .window_phase   (window_phase),     // 0=early, 1=late
        .window_complete(window_complete),  // Pulse at window end
        // Accumulator snapshots (valid when accum_valid=1)
        .early_sum_x    (early_sum_x),      // [17:0] signed
        .early_sum_y    (early_sum_y),
        .early_count    (early_count),      // [11:0] unsigned
        .late_sum_x     (late_sum_x),
        .late_sum_y     (late_sum_y),
        .late_count     (late_count),
        .accum_valid    (accum_valid),
        // Debug
        .debug_event_count(debug_event_count)
    );

    // =========================================================================
    // Module: MotionComputer
    // =========================================================================
    // Purpose: Extract motion vector from early/late centroid difference
    //
    // Algorithm (fixed-point integer arithmetic):
    //   Δx = late_sum_x - early_sum_x
    //   Δy = late_sum_y - early_sum_y
    //
    // Interpretation:
    //   Positive Δx → Motion to the RIGHT
    //   Negative Δx → Motion to the LEFT
    //   Positive Δy → Motion DOWNWARD (Y increases downward in image coords)
    //   Negative Δy → Motion UPWARD
    //
    // Note: Using sum differences rather than true centroid differences
    // (sum/count) is valid when event counts are similar between windows.
    // For significantly different counts, normalization would improve accuracy.
    // =========================================================================
    
    MotionComputer #(
        .ACC_SUM_BITS   (ACC_SUM_BITS),
        .ACC_COUNT_BITS (ACC_COUNT_BITS)
    ) u_motion_computer (
        .clk            (clk),
        .rst            (rst),
        // Input from TemporalAccumulator
        .trigger        (accum_valid),
        .early_sum_x    (early_sum_x),
        .early_sum_y    (early_sum_y),
        .early_count    (early_count),
        .late_sum_x     (late_sum_x),
        .late_sum_y     (late_sum_y),
        .late_count     (late_count),
        // Output to GestureClassifier
        .delta_x        (delta_x),          // [17:0] signed motion
        .delta_y        (delta_y),
        .abs_delta_x    (abs_delta_x),      // [17:0] unsigned magnitude
        .abs_delta_y    (abs_delta_y),
        .total_events   (total_events),
        .valid          (motion_valid)
    );

    // =========================================================================
    // Module: GestureClassifier
    // =========================================================================
    // Purpose: Classify motion vector into cardinal gesture direction
    //
    // Classification Logic:
    //   1. Activity Gate: total_events >= MIN_EVENT_THRESH
    //   2. Motion Gate: max(|Δx|, |Δy|) >= MOTION_THRESH
    //   3. Dominant Axis: Compare |Δx| vs |Δy|
    //   4. Direction: Sign of dominant component
    //
    // Decision Tree:
    //   IF events < threshold → NO GESTURE (noise rejection)
    //   ELIF motion < threshold → NO GESTURE (stationary)
    //   ELIF |Δy| > |Δx| → VERTICAL dominant
    //       IF Δy > 0 → DOWN
    //       ELSE → UP
    //   ELSE → HORIZONTAL dominant
    //       IF Δx > 0 → RIGHT
    //       ELSE → LEFT
    // =========================================================================
    
    GestureClassifier #(
        .ACC_SUM_BITS     (ACC_SUM_BITS),
        .ACC_COUNT_BITS   (ACC_COUNT_BITS),
        .MIN_EVENT_THRESH (MIN_EVENT_THRESH),
        .MOTION_THRESH    (MOTION_THRESH)
    ) u_gesture_classifier (
        .clk            (clk),
        .rst            (rst),
        // Input from MotionComputer
        .trigger        (motion_valid),
        .delta_x        (delta_x),
        .delta_y        (delta_y),
        .abs_delta_x    (abs_delta_x),
        .abs_delta_y    (abs_delta_y),
        .total_events   (total_events),
        // Output to OutputRegister
        .gesture        (class_gesture),
        .valid          (class_valid),
        .pass           (class_pass)        // Passed activity & motion gates
    );

    // =========================================================================
    // Module: OutputRegister
    // =========================================================================
    // Purpose: Apply persistence filter and generate final gesture output
    //
    // Persistence Filter:
    //   Requires PERSISTENCE_COUNT consecutive windows with same classification
    //   before asserting gesture_valid. This suppresses spurious detections.
    //
    // Confidence Calculation:
    //   Based on magnitude of dominant motion component
    //   confidence = min(15, dominant_magnitude >> 4)
    //   Higher values indicate stronger, more definitive motion
    // =========================================================================
    
    OutputRegister #(
        .ACC_SUM_BITS     (ACC_SUM_BITS),
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT)
    ) u_output_register (
        .clk              (clk),
        .rst              (rst),
        // Input from GestureClassifier
        .class_gesture    (class_gesture),
        .class_valid      (class_valid),
        .class_pass       (class_pass),
        .abs_delta_x      (abs_delta_x),
        .abs_delta_y      (abs_delta_y),
        // Final output
        .gesture          (gesture),
        .gesture_valid    (gesture_valid),
        .gesture_confidence(gesture_confidence),
        // Debug
        .debug_state      (debug_state)
    );

endmodule
