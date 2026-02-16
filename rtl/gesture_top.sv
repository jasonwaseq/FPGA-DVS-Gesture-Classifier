`timescale 1ns/1ps

// =============================================================================
// Module: gesture_top
// =============================================================================
// Top-Level Module for Time-Surface DVS Gesture Classifier
//
// Target Device: Lattice iCE40UP5K
// Sensor Input: Prophesee GenX320 (EVT 2.0 32-bit data bus)
//
// Architecture:
//   1. input_fifo: Buffers EVT 2.0 data from sensor (256 x 32-bit BRAM)
//   2. evt2_decoder: Decodes packets, reconstructs timestamps, downsamples
//   3. time_surface_memory: 16x16 grid storing last event timestamps
//   4. feature_extractor: Computes spatial moments, classifies gestures
//   5. uart_debug: Sends classification results over UART
//
// Data Flow:
//   Sensor -> FIFO -> Decoder -> Time Surface -> Feature Extractor -> UART
//
// Clock: Single global clock (12 MHz recommended for iCE40)
// =============================================================================

module gesture_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,  // System clock frequency
    parameter BAUD_RATE       = 115200,      // UART baud rate
    parameter FRAME_PERIOD_MS = 10,          // Classification period (ms)
    parameter DECAY_SHIFT     = 6,           // Time surface decay rate
    parameter MIN_MASS_THRESH = 100          // Minimum mass for valid gesture
)(
    input  logic        clk,                // System clock
    input  logic        rst_n,              // Active-low reset
    
    // EVT 2.0 Sensor Interface
    input  logic [31:0] evt_data,           // EVT 2.0 32-bit word
    input  logic        evt_valid,          // Data valid strobe
    output logic        evt_ready,          // Ready to accept data
    
    // UART Debug Output
    output logic        uart_tx,            // UART transmit
    
    // LED Indicators
    output logic        led_heartbeat,      // Heartbeat blink
    output logic        led_activity,       // Event activity
    output logic        led_up,             // UP gesture detected
    output logic        led_down,           // DOWN gesture detected
    output logic        led_left,           // LEFT gesture detected
    output logic        led_right           // RIGHT gesture detected
);

    // -------------------------------------------------------------------------
    // Internal Reset (active-high, synchronized)
    // -------------------------------------------------------------------------
    logic rst;
    logic [3:0] rst_sync;
    
    always_ff @(posedge clk) begin
        rst_sync <= {rst_sync[2:0], ~rst_n};
        rst <= rst_sync[3];
    end
    
    // -------------------------------------------------------------------------
    // Global Timestamp Counter (16-bit, wraps)
    // Used for time-surface decay computation
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;
    
    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end
    
    // -------------------------------------------------------------------------
    // Input FIFO (EVT 2.0 buffering)
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    
    input_fifo #(
        .DEPTH(256),
        .PTR_BITS(8),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_valid && !fifo_full),
        .wr_data (evt_data),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()  // Unused
    );
    
    // Ready signal: accept data if FIFO not full
    assign evt_ready = !fifo_full;
    
    // -------------------------------------------------------------------------
    // EVT 2.0 Decoder
    // -------------------------------------------------------------------------
    logic       decoder_ready;
    logic [3:0] decoded_x, decoded_y;
    logic       decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic       decoded_valid;
    
    evt2_decoder #(
        .GRID_BITS(4)
    ) u_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (!fifo_empty),
        .data_ready (decoder_ready),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );
    
    // Read from FIFO when decoder is ready
    assign fifo_rd_en = decoder_ready && !fifo_empty;
    
    // -------------------------------------------------------------------------
    // Time-Surface Memory
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;
    
    time_surface_memory #(
        .GRID_SIZE(16),
        .ADDR_BITS(8),
        .TS_BITS(16),
        .VALUE_BITS(8),
        .MAX_VALUE(255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_time_surface (
        .clk         (clk),
        .rst         (rst),
        .t_now       (global_timestamp),
        .event_valid (decoded_valid),
        .event_x     (decoded_x),
        .event_y     (decoded_y),
        .event_ts    (global_timestamp),  // Use global timestamp for consistency
        .read_enable (ts_read_enable),
        .read_addr   (ts_read_addr),
        .read_value  (ts_read_value),
        .read_ts_raw (ts_read_raw)
    );
    
    // -------------------------------------------------------------------------
    // Feature Extractor & Classifier
    // -------------------------------------------------------------------------
    logic [1:0] gesture_class;
    logic       gesture_valid;
    logic [7:0] gesture_confidence;
    logic [23:0] debug_m00, debug_m10, debug_m01;
    logic [2:0] debug_state;
    
    feature_extractor #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .FRAME_PERIOD_MS(FRAME_PERIOD_MS),
        .GRID_SIZE(16),
        .VALUE_BITS(8),
        .MOMENT_BITS(24),
        .MIN_MASS_THRESH(MIN_MASS_THRESH)
    ) u_feature_extractor (
        .clk               (clk),
        .rst               (rst),
        .ts_read_addr      (ts_read_addr),
        .ts_read_enable    (ts_read_enable),
        .ts_read_value     (ts_read_value),
        .gesture_class     (gesture_class),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_m00         (debug_m00),
        .debug_m10         (debug_m10),
        .debug_m01         (debug_m01),
        .debug_state       (debug_state)
    );
    
    // -------------------------------------------------------------------------
    // UART Debug Output
    // -------------------------------------------------------------------------
    uart_debug #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_debug (
        .clk               (clk),
        .rst               (rst),
        .gesture_class     (gesture_class),
        .gesture_valid     (gesture_valid),
        .gesture_confidence(gesture_confidence),
        .uart_tx           (uart_tx)
    );
    
    // -------------------------------------------------------------------------
    // Heartbeat LED (~1.5 Hz blink at 12 MHz)
    // -------------------------------------------------------------------------
    logic [22:0] heartbeat_cnt;
    
    always_ff @(posedge clk) begin
        if (rst)
            heartbeat_cnt <= '0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end
    
    assign led_heartbeat = heartbeat_cnt[22];
    
    // -------------------------------------------------------------------------
    // Activity LED (pulses on events)
    // -------------------------------------------------------------------------
    logic [19:0] activity_cnt;
    
    always_ff @(posedge clk) begin
        if (rst)
            activity_cnt <= '0;
        else if (decoded_valid)
            activity_cnt <= {20{1'b1}};  // Reset to max
        else if (activity_cnt > 0)
            activity_cnt <= activity_cnt - 1'b1;
    end
    
    assign led_activity = (activity_cnt > 0);
    
    // -------------------------------------------------------------------------
    // Gesture Direction LEDs (pulse stretch for visibility)
    // -------------------------------------------------------------------------
    logic [23:0] led_up_cnt, led_down_cnt, led_left_cnt, led_right_cnt;
    localparam LED_STRETCH = 24'd6_000_000;  // ~500ms at 12MHz
    
    always_ff @(posedge clk) begin
        if (rst) begin
            led_up_cnt    <= '0;
            led_down_cnt  <= '0;
            led_left_cnt  <= '0;
            led_right_cnt <= '0;
        end else begin
            // UP LED
            if (gesture_valid && gesture_class == 2'd0)
                led_up_cnt <= LED_STRETCH;
            else if (led_up_cnt > 0)
                led_up_cnt <= led_up_cnt - 1'b1;
                
            // DOWN LED
            if (gesture_valid && gesture_class == 2'd1)
                led_down_cnt <= LED_STRETCH;
            else if (led_down_cnt > 0)
                led_down_cnt <= led_down_cnt - 1'b1;
                
            // LEFT LED
            if (gesture_valid && gesture_class == 2'd2)
                led_left_cnt <= LED_STRETCH;
            else if (led_left_cnt > 0)
                led_left_cnt <= led_left_cnt - 1'b1;
                
            // RIGHT LED
            if (gesture_valid && gesture_class == 2'd3)
                led_right_cnt <= LED_STRETCH;
            else if (led_right_cnt > 0)
                led_right_cnt <= led_right_cnt - 1'b1;
        end
    end
    
    assign led_up    = (led_up_cnt > 0);
    assign led_down  = (led_down_cnt > 0);
    assign led_left  = (led_left_cnt > 0);
    assign led_right = (led_right_cnt > 0);

endmodule
