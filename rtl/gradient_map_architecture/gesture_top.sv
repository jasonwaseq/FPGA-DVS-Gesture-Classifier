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
//   1. input_fifo:                Buffers EVT 2.0 data (256 x 32-bit BRAM)
//   2. evt2_decoder:              Decodes packets, reconstructs timestamps
//   3. time_surface_encoder:      16x16 grid with exponential-decay timestamps
//   4. spatio_temporal_classifier: Flatten → systolic MAC → argmax classifier
//   5. uart_debug:                Sends classifications over UART
//
// Data Flow:
//   Sensor -> FIFO -> Decoder -> Time Surface Encoder -> Spatio-Temporal Classifier -> UART
//
// Clock: Single global clock (12 MHz recommended for iCE40)
// =============================================================================

module gesture_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,  // System clock frequency
    parameter BAUD_RATE       = 115200,      // UART baud rate
    parameter FRAME_PERIOD_MS = 10,          // Classification period (ms)
    parameter DECAY_SHIFT     = 6,           // Time surface decay rate
    parameter MIN_MASS_THRESH = 100,         // Minimum mass for valid gesture
    parameter UART_RX_ENABLE  = 1'b0         // Enable UART mock event input
)(
    input  logic        clk,                // System clock
    input  logic        rst_n,              // Active-low reset
    
    // EVT 2.0 Sensor Interface
    input  logic [31:0] evt_data,           // EVT 2.0 32-bit word
    input  logic        evt_valid,          // Data valid strobe
    output logic        evt_ready,          // Ready to accept data
    
    // UART Interface
    input  logic        uart_rx,            // UART receive (event input)
    output logic        uart_tx,            // UART transmit (debug output)
    
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
    // Input FIFO signals (declared early - referenced in UART RX generate block)
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_wr_en;
    logic [31:0] fifo_wr_data;

    // -------------------------------------------------------------------------
    // UART RX (5-byte legacy event protocol)
    // -------------------------------------------------------------------------
    localparam EVT_CD_OFF = 4'h0;
    localparam EVT_CD_ON  = 4'h1;

    logic [7:0] uart_rx_data;
    logic       uart_rx_valid;
    logic [2:0] uart_byte_idx;
    logic [7:0] uart_x_hi, uart_x_lo;
    logic [7:0] uart_y_hi, uart_y_lo;
    logic [7:0] uart_pol;
    logic [31:0] uart_evt_word;
    logic       uart_evt_pending;

    wire [10:0] uart_x = {uart_x_hi[0], uart_x_lo};
    wire [10:0] uart_y = {uart_y_hi[0], uart_y_lo};
    wire [5:0]  uart_ts_lsb = global_timestamp[5:0];

    generate
        if (UART_RX_ENABLE) begin : gen_uart_rx
            uart_rx #(
                .CLKS_PER_BIT(CLK_FREQ_HZ / BAUD_RATE)
            ) u_uart_rx (
                .clk   (clk),
                .rst   (rst),
                .rx    (uart_rx),
                .data  (uart_rx_data),
                .valid (uart_rx_valid)
            );

            always_ff @(posedge clk) begin
                if (rst) begin
                    uart_byte_idx   <= 3'd0;
                    uart_x_hi       <= 8'd0;
                    uart_x_lo       <= 8'd0;
                    uart_y_hi       <= 8'd0;
                    uart_y_lo       <= 8'd0;
                    uart_pol        <= 8'd0;
                    uart_evt_word   <= 32'd0;
                    uart_evt_pending <= 1'b0;
                end else begin
                    if (uart_rx_valid) begin
                        case (uart_byte_idx)
                            3'd0: uart_x_hi <= uart_rx_data;
                            3'd1: uart_x_lo <= uart_rx_data;
                            3'd2: uart_y_hi <= uart_rx_data;
                            3'd3: uart_y_lo <= uart_rx_data;
                            3'd4: begin
                                uart_pol <= uart_rx_data;
                                if (!uart_evt_pending) begin
                                    uart_evt_word <= {uart_rx_data[0] ? EVT_CD_ON : EVT_CD_OFF, uart_ts_lsb, uart_x, uart_y};
                                    uart_evt_pending <= 1'b1;
                                end
                            end
                        endcase

                        if (uart_byte_idx == 3'd4)
                            uart_byte_idx <= 3'd0;
                        else
                            uart_byte_idx <= uart_byte_idx + 1'b1;
                    end

                    if (uart_evt_pending && !fifo_full)
                        uart_evt_pending <= 1'b0;
                end
            end
        end else begin : gen_uart_rx_off
            always_comb begin
                uart_rx_data = 8'd0;
                uart_rx_valid = 1'b0;
            end

            always_ff @(posedge clk) begin
                if (rst) begin
                    uart_byte_idx   <= 3'd0;
                    uart_x_hi       <= 8'd0;
                    uart_x_lo       <= 8'd0;
                    uart_y_hi       <= 8'd0;
                    uart_y_lo       <= 8'd0;
                    uart_pol        <= 8'd0;
                    uart_evt_word   <= 32'd0;
                    uart_evt_pending <= 1'b0;
                end else begin
                    uart_evt_pending <= 1'b0;
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Input FIFO (EVT 2.0 buffering)
    // -------------------------------------------------------------------------
    input_fifo #(
        .DEPTH(128),
        .PTR_BITS(7),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (fifo_wr_en),
        .wr_data (fifo_wr_data),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()  // Unused
    );
    
    // Mux UART events and external EVT2 input
    always_comb begin
        if (UART_RX_ENABLE && uart_evt_pending) begin
            fifo_wr_en = !fifo_full;
            fifo_wr_data = uart_evt_word;
        end else begin
            fifo_wr_en = evt_valid && !fifo_full;
            fifo_wr_data = evt_data;
        end
    end

    // Ready signal: accept external data if FIFO not full and UART has no pending word
    assign evt_ready = !fifo_full && !(UART_RX_ENABLE && uart_evt_pending);
    
    // -------------------------------------------------------------------------
    // EVT 2.0 Decoder
    // -------------------------------------------------------------------------
    logic       decoder_ready;
    logic [3:0] decoded_x, decoded_y;   // 4 bits for 16×16 grid (ice40up5k fit)
    logic       decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic       decoded_valid;
    logic        fifo_rd_valid;
    
    evt2_decoder #(
        .GRID_BITS(4)   // 16 cells per axis (ice40up5k fit)
    ) u_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (decoder_ready),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    // Read from FIFO whenever data is available; align valid with BRAM read latency
    assign fifo_rd_en = !fifo_empty;

    always_ff @(posedge clk) begin
        if (rst) begin
            fifo_rd_valid <= 1'b0;
        end else begin
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
        end
    end
    
    // -------------------------------------------------------------------------
    // Time-Surface Encoder (Exponential Decay)
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;   // 8-bit for 256 cells (16×16, ice40up5k fit)
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    time_surface_encoder #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_time_surface (
        .clk         (clk),
        .rst         (rst),
        .t_now       (global_timestamp),
        .event_valid (decoded_valid),
        .event_x     (decoded_x),
        .event_y     (decoded_y),
        .event_ts    (global_timestamp),
        .read_enable (ts_read_enable),
        .read_addr   (ts_read_addr),
        .read_value  (ts_read_value),
        .read_ts_raw (ts_read_raw)
    );

    // -------------------------------------------------------------------------
    // Spatio-Temporal Classifier
    // (Flatten -> Systolic Array MAC -> Argmax)
    // -------------------------------------------------------------------------
    logic [1:0]  gesture_class;
    logic        gesture_valid;
    logic [7:0]  gesture_confidence;
    logic [23:0] debug_m00, debug_m10, debug_m01;
    logic [2:0]  debug_state;

    spatio_temporal_classifier #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .FRAME_PERIOD_MS(FRAME_PERIOD_MS),
        .GRID_SIZE      (16),
        .ADDR_BITS      (8),
        .VALUE_BITS     (8),
        .MOMENT_BITS    (24),
        .WEIGHT_BITS    (8),
        .SCORE_BITS     (24),
        .NUM_CLASSES    (4),
        .MIN_MASS_THRESH(MIN_MASS_THRESH)
    ) u_spatio_classifier (
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
