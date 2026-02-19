
// =============================================================================
// DVS Gesture Accelerator - Asynchronous Spatiotemporal Motion-Energy Classifier
// =============================================================================
//
// Architecture Overview:
//   - Event-driven asynchronous processing pipeline
//   - InputFIFO: 16-entry shallow buffer for bursty DVS events
//   - SpatialCompressor: 320x320 -> 16x16 grid mapping
//   - TimeSurfaceBinning: Accumulates events into 5x16x16 histogram bins
//   - SystolicMatrixMultiply: Classifies the flattened bin vector
//   - WeightROM: Stores learned weights for 4 gestures
//   - OutputRegister: Persistence filter and confidence computation
//
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
                                                 //   2'b00 = UP    (negative Delta y)
                                                 //   2'b01 = DOWN  (positive Delta y)
                                                 //   2'b10 = LEFT  (negative Delta x)
                                                 //   2'b11 = RIGHT (positive Delta x)
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
    // Local Parameters
    // =========================================================================
    localparam integer GRID_BITS        = $clog2(GRID_SIZE);            // 4 bits for 16x16
    localparam integer FIFO_PTR_BITS    = $clog2(FIFO_DEPTH);           // 4 bits for 16 entries
    
    // =========================================================================
    // Internal Signal Declarations
    // =========================================================================
    
    // InputFIFO -> SpatialCompressor interface
    logic                      fifo_pop;
    logic                      fifo_empty;
    logic                      fifo_full;
    logic [8:0]                fifo_out_x;
    logic [8:0]                fifo_out_y;
    logic                      fifo_out_pol;
    logic [15:0]               fifo_out_ts;
    
    // SpatialCompressor -> TimeSurfaceBinning interface
    logic signed [GRID_BITS:0] compressed_x;
    logic signed [GRID_BITS:0] compressed_y;
    logic                      compressed_pol;
    logic                      compressed_valid;
    
    // TimeSurfaceBinning -> SystolicMatrixMultiply interface
    logic        readout_start;
    logic [7:0]  readout_data;
    logic        readout_valid;

    // Systolic Output
    localparam NUM_CLASSES = 4;
    localparam NUM_CELLS   = 5 * GRID_SIZE * GRID_SIZE; // 5 * 256 = 1280
    localparam WEIGHT_BITS = 8;
    localparam ACC_BITS    = 24;

    logic [$clog2(NUM_CELLS)-1:0]       w_addr;
    logic [NUM_CLASSES*WEIGHT_BITS-1:0] w_data_flat;
    logic                               sys_result_valid;
    logic [1:0]                         sys_best_class;
    logic [NUM_CLASSES*ACC_BITS-1:0]    sys_scores_flat;

    // =========================================================================
    // Module: InputFIFO
    // =========================================================================
    
    InputFIFO #(
        .DEPTH(FIFO_DEPTH),
        .PTR_BITS(FIFO_PTR_BITS)
    ) u_input_fifo (
        .clk            (clk),
        .rst            (rst),
        .push_valid     (event_valid),
        .push_x         (event_x),
        .push_y         (event_y),
        .push_polarity  (event_polarity),
        .push_ts        (event_ts),
        .push_ready     (event_ready),
        .pop_req        (fifo_pop),
        .pop_x          (fifo_out_x),
        .pop_y          (fifo_out_y),
        .pop_polarity   (fifo_out_pol),
        .pop_ts         (fifo_out_ts),
        .empty          (fifo_empty),
        .full           (fifo_full)
    );
    
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;
    assign debug_temporal_phase = 1'b0; 

    // =========================================================================
    // Module: SpatialCompressor
    // =========================================================================
    
    SpatialCompressor #(
        .SENSOR_RES     (SENSOR_RES),
        .GRID_SIZE      (GRID_SIZE),
        .GRID_BITS      (GRID_BITS)
    ) u_spatial_compressor (
        .clk            (clk),
        .rst            (rst),
        .in_valid       (!fifo_empty),
        .in_x           (fifo_out_x),
        .in_y           (fifo_out_y),
        .in_polarity    (fifo_out_pol),
        .in_ready       (fifo_pop),
        .out_x          (compressed_x),
        .out_y          (compressed_y),
        .out_polarity   (compressed_pol),
        .out_valid      (compressed_valid)
    );

    // =========================================================================
    // Module: TimeSurfaceBinning
    // =========================================================================
    // Spatio-Temporal Pooling: 5 bins of 16x16 counters
    
    TimeSurfaceBinning #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .WINDOW_MS      (WINDOW_MS),
        .NUM_BINS       (8),
        .READOUT_BINS   (5),
        .GRID_SIZE      (GRID_SIZE),
        .COUNTER_BITS   (8)
    ) u_time_surface_binning (
        .clk            (clk),
        .rst            (rst),
        .event_valid    (compressed_valid),
        .event_x        (compressed_x),
        .event_y        (compressed_y),
        .event_polarity (compressed_pol),
        .readout_start  (readout_start),
        .readout_data   (readout_data),
        .readout_valid  (readout_valid)
    );
    
    assign debug_event_count = 8'd0; 

    // =========================================================================
    // Module: SystolicMatrixMultiply
    // =========================================================================
    
    SystolicMatrixMultiply #(
        .NUM_CLASSES    (NUM_CLASSES),
        .NUM_CELLS      (NUM_CELLS),
        .VALUE_BITS     (8),
        .WEIGHT_BITS    (WEIGHT_BITS),
        .ACC_BITS       (ACC_BITS)
    ) u_systolic_array (
        .clk            (clk),
        .rst            (rst),
        .start          (readout_start),
        .feature_in     (readout_data),
        .feature_valid  (readout_valid),
        .w_addr         (w_addr),
        .w_data_flat    (w_data_flat),
        .result_valid   (sys_result_valid),
        .best_class     (sys_best_class),
        .scores_flat    (sys_scores_flat)
    );

    // =========================================================================
    // Modules: WeightROMs (4 Parallel Instances)
    // =========================================================================
    
    generate
        genvar k;
        for (k = 0; k < NUM_CLASSES; k = k + 1) begin : gen_weight_roms
            WeightROM #(
                .CLASS_IDX   (k),
                .NUM_CELLS   (NUM_CELLS),
                .GRID_SIZE   (GRID_SIZE),
                .WEIGHT_BITS (WEIGHT_BITS)
            ) u_weight_rom (
                .clk         (clk),
                .cell_addr   (w_addr),
                .dout        (w_data_flat[(k+1)*WEIGHT_BITS-1 : k*WEIGHT_BITS])
            );
        end
    endgenerate

    // =========================================================================
    // Module: OutputRegister
    // =========================================================================
    
    // Extract score of best class for confidence
    logic signed [ACC_BITS-1:0] best_score;
    logic signed [ACC_BITS-1:0] score_slice;
    logic [ACC_BITS-1:0] abs_best_score;
    
    always_comb begin
        case (sys_best_class)
            2'd0: score_slice = sys_scores_flat[1*ACC_BITS-1:0*ACC_BITS];
            2'd1: score_slice = sys_scores_flat[2*ACC_BITS-1:1*ACC_BITS];
            2'd2: score_slice = sys_scores_flat[3*ACC_BITS-1:2*ACC_BITS];
            2'd3: score_slice = sys_scores_flat[4*ACC_BITS-1:3*ACC_BITS];
            default: score_slice = '0;
        endcase
        best_score = score_slice;
        
        if (best_score < 0)
            abs_best_score = -best_score;
        else
            abs_best_score = best_score;
    end
    
    // Simple confidence mapping
    // Take bits [15:12] as a proxy for normalized confidence
    logic [17:0] pseudo_mag_x, pseudo_mag_y;
    assign pseudo_mag_x = {2'b0, abs_best_score[15:0]}; // Zero pad to 18 bits
    assign pseudo_mag_y = 18'd0; 

    OutputRegister #(
        .ACC_SUM_BITS     (18), 
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT)
    ) u_output_register (
        .clk              (clk),
        .rst              (rst),
        .class_gesture    (sys_best_class),
        .class_valid      (sys_result_valid),
        .class_pass       (sys_result_valid), // Assume all valid results pass
        .abs_delta_x      (pseudo_mag_x),
        .abs_delta_y      (pseudo_mag_y),
        .gesture          (gesture),
        .gesture_valid    (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_state      (debug_state)
    );

endmodule
