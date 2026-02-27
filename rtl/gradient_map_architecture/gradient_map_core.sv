`timescale 1ns/1ps

// Gradient-map architecture core.
// Data path:
//   input_fifo -> evt2_decoder -> gradient_mapping -> gesture_classifier

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    logic [15:0] global_timestamp;
    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(.GRID_BITS(4)) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw()
    );

    gesture_classifier #(
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
    ) u_gesture_classifier (
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

endmodule
*/

/* Duplicate/legacy content below is intentionally commented out.
   This file must contain only module gradient_map_core (matching the filename). */
`timescale 1ns/1ps

// Gradient-map architecture core.
// Data path:
//   input_fifo -> evt2_decoder -> gradient_mapping -> gesture_classifier

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    logic [15:0] global_timestamp;
    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(.GRID_BITS(4)) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw(ts_read_raw)
    );

    gesture_classifier #(
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
    ) u_gesture_classifier (
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

endmodule

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    // Global timestamp for gradient mapping decay (local free-running counter).
    logic [15:0] global_timestamp;
    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------
    // input_fifo
    // -------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // -------------------------
    // evt2_decoder
    // -------------------------
    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    // -------------------------
    // gradient_mapping
    // -------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw(ts_read_raw)
    );

    // -------------------------
    // gesture_classifier
    // -------------------------
    gesture_classifier #(
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
    ) u_gesture_classifier (
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

endmodule

`timescale 1ns/1ps

// Full gradient-map processing core.
// Data flow:
//   EVT2 word stream
//     -> input_fifo
//     -> evt2_decoder
//     -> gradient_mapping (time-surface encoder)
//     -> spatio_temporal_classifier (includes systolic_array + gesture decision)

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    // -------------------------------------------------------------------------
    // Global timestamp counter used by gradient mapping decay.
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;

    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------------------------------------------------------
    // EVT2 input FIFO
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // -------------------------------------------------------------------------
    // EVT2 decode
    // -------------------------------------------------------------------------
    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    // -------------------------------------------------------------------------
    // Gradient mapping (time surface with exponential decay on read)
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw(ts_read_raw)
    );

    // -------------------------------------------------------------------------
    // Gesture classifier (drives systolic array internally)
    // -------------------------------------------------------------------------
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

endmodule

`timescale 1ns/1ps

// Full gradient-map processing core.
// Data flow:
//   EVT2 word stream
//     -> input_fifo
//     -> evt2_decoder
//     -> gradient_mapping (time-surface encoder)
//     -> spatio_temporal_classifier (includes systolic_array + gesture decision)

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    // -------------------------------------------------------------------------
    // Global timestamp counter used by gradient mapping decay.
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;

    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------------------------------------------------------
    // EVT2 input FIFO
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // -------------------------------------------------------------------------
    // EVT2 decode
    // -------------------------------------------------------------------------
    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    // -------------------------------------------------------------------------
    // Gradient mapping (time surface with exponential decay on read)
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw(ts_read_raw)
    );

    // -------------------------------------------------------------------------
    // Gesture classifier (drives systolic array internally)
    // -------------------------------------------------------------------------
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

endmodule

`timescale 1ns/1ps

// Full gradient-map processing core.
// Data flow:
//   EVT2 word stream
//     -> input_fifo
//     -> evt2_decoder
//     -> gradient_mapping (time-surface encoder)
//     -> spatio_temporal_classifier (includes systolic_array + gesture decision)

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    // -------------------------------------------------------------------------
    // Global timestamp counter used by gradient mapping decay.
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;

    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------------------------------------------------------
    // EVT2 input FIFO
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready   = !fifo_full;
    assign fifo_rd_en       = !fifo_empty;
    assign debug_fifo_empty = fifo_empty;
    assign debug_fifo_full  = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // -------------------------------------------------------------------------
    // EVT2 decode
    // -------------------------------------------------------------------------
    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    // -------------------------------------------------------------------------
    // Gradient mapping (time surface with exponential decay on read)
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
    logic        ts_read_enable;
    logic [7:0]  ts_read_value;
    logic [15:0] ts_read_raw;

    gradient_mapping #(
        .GRID_SIZE  (16),
        .ADDR_BITS  (8),
        .TS_BITS    (16),
        .VALUE_BITS (8),
        .MAX_VALUE  (255),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_gradient_mapping (
        .clk        (clk),
        .rst        (rst),
        .t_now      (global_timestamp),
        .event_valid(decoded_valid),
        .event_x    (decoded_x),
        .event_y    (decoded_y),
        .event_ts   (global_timestamp),
        .read_enable(ts_read_enable),
        .read_addr  (ts_read_addr),
        .read_value (ts_read_value),
        .read_ts_raw(ts_read_raw)
    );

    // -------------------------------------------------------------------------
    // Gesture classifier (drives systolic array internally)
    // -------------------------------------------------------------------------
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

endmodule

`timescale 1ns/1ps

// Exponential-decay time surface: S(x,y,t) = 255 * 2^(-Dt / 2^DECAY_SHIFT)
// Wraps time_surface_memory; replaces its linear decay with exponential shift.
// Two-stage pipeline preserves the same 2-cycle read latency.

module time_surface_encoder #(
    parameter GRID_SIZE   = 16,
    parameter ADDR_BITS   = 8,
    parameter TS_BITS     = 16,
    parameter VALUE_BITS  = 8,
    parameter MAX_VALUE   = 255,
    parameter DECAY_SHIFT = 6
)(
    input  logic                    clk,
    input  logic                    rst,
    input  logic [TS_BITS-1:0]      t_now,
    input  logic                              event_valid,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_x,
    input  logic [$clog2(GRID_SIZE)-1:0]      event_y,
    input  logic [TS_BITS-1:0]                event_ts,
    input  logic                    read_enable,
    input  logic [ADDR_BITS-1:0]    read_addr,
    output logic [VALUE_BITS-1:0]   read_value,
    output logic [TS_BITS-1:0]      read_ts_raw
);

    logic [TS_BITS-1:0]   raw_ts;
    logic [VALUE_BITS-1:0] ts_mem_value;
    logic                  ts_mem_cell_valid;

    time_surface_memory #(
        .GRID_SIZE  (GRID_SIZE),
        .ADDR_BITS  (ADDR_BITS),
        .TS_BITS    (TS_BITS),
        .VALUE_BITS (VALUE_BITS),
        .MAX_VALUE  (MAX_VALUE),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) u_ts_mem (
        .clk            (clk),
        .rst            (rst),
        .t_now          (t_now),
        .event_valid    (event_valid),
        .event_x        (event_x),
        .event_y        (event_y),
        .event_ts       (event_ts),
        .read_enable    (read_enable),
        .read_addr      (read_addr),
        .read_value     (ts_mem_value),
        .read_ts_raw    (raw_ts),
        .read_cell_valid(ts_mem_cell_valid)
    );

    assign read_ts_raw = raw_ts;

    // Stage 1: compute Δt and decay_steps (16-bit wrap-around safe)
    logic [TS_BITS-1:0] delta_t_r1;
    logic [TS_BITS-1:0] decay_steps_r1;
    logic               cell_valid_r1;

    always_ff @(posedge clk) begin
        if (rst) begin
            delta_t_r1     <= '0;
            decay_steps_r1 <= '0;
            cell_valid_r1  <= 1'b0;
        end else if (read_enable) begin
            delta_t_r1     <= t_now - raw_ts;
            decay_steps_r1 <= (t_now - raw_ts) >> DECAY_SHIFT;
            cell_valid_r1  <= ts_mem_cell_valid;
        end
    end

    // Stage 2: apply exponential decay via right-shift
    always_ff @(posedge clk) begin
        if (rst) begin
            read_value <= '0;
        end else begin
            if (!cell_valid_r1)
                read_value <= '0;
            else if (decay_steps_r1 >= TS_BITS'(VALUE_BITS))
                read_value <= '0;
            else
                read_value <= VALUE_BITS'(MAX_VALUE) >> decay_steps_r1[3:0];
        end
    end

endmodule

// Full gradient-map processing core.
// Data flow:
// EVT2 word stream -> input_fifo -> evt2_decoder -> time_surface_encoder
// -> spatio_temporal_classifier (includes systolic_array + gesture decision)

module gradient_map_core #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000,
    parameter FIFO_DEPTH      = 128
)(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] evt_word,
    input  logic        evt_word_valid,
    output logic        evt_word_ready,
    output logic [1:0]  gesture_class,
    output logic        gesture_valid,
    output logic [7:0]  gesture_confidence,
    output logic        decoded_event_valid,
    output logic [23:0] debug_m00,
    output logic [23:0] debug_m10,
    output logic [23:0] debug_m01,
    output logic [2:0]  debug_state,
    output logic        debug_fifo_empty,
    output logic        debug_fifo_full
);

    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);

    // -------------------------------------------------------------------------
    // Global timestamp counter used by gradient mapping decay.
    // -------------------------------------------------------------------------
    logic [15:0] global_timestamp;

    always_ff @(posedge clk) begin
        if (rst)
            global_timestamp <= '0;
        else
            global_timestamp <= global_timestamp + 1'b1;
    end

    // -------------------------------------------------------------------------
    // EVT2 input FIFO
    // -------------------------------------------------------------------------
    logic        fifo_rd_en;
    logic [31:0] fifo_rd_data;
    logic        fifo_empty;
    logic        fifo_full;
    logic        fifo_rd_valid;

    assign evt_word_ready    = !fifo_full;
    assign fifo_rd_en        = !fifo_empty;
    assign debug_fifo_empty  = fifo_empty;
    assign debug_fifo_full   = fifo_full;

    input_fifo #(
        .DEPTH     (FIFO_DEPTH),
        .PTR_BITS  (FIFO_PTR_BITS),
        .DATA_WIDTH(32)
    ) u_input_fifo (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (evt_word_valid && evt_word_ready),
        .wr_data (evt_word),
        .rd_en   (fifo_rd_en),
        .rd_data (fifo_rd_data),
        .empty   (fifo_empty),
        .full    (fifo_full),
        .count   ()
    );

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

    // -------------------------------------------------------------------------
    // EVT2 decode
    // -------------------------------------------------------------------------
    logic [3:0]  decoded_x, decoded_y;
    logic        decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic        decoded_valid;

    evt2_decoder #(
        .GRID_BITS(4)
    ) u_evt2_decoder (
        .clk        (clk),
        .rst        (rst),
        .data_in    (fifo_rd_data),
        .data_valid (fifo_rd_valid),
        .data_ready (),
        .x_out      (decoded_x),
        .y_out      (decoded_y),
        .polarity   (decoded_polarity),
        .timestamp  (decoded_timestamp),
        .event_valid(decoded_valid)
    );

    assign decoded_event_valid = decoded_valid;

    // -------------------------------------------------------------------------
    // Gradient mapping (time surface with exponential decay on read)
    // -------------------------------------------------------------------------
    logic [7:0]  ts_read_addr;
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
    ) u_gradient_mapping (
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
    // Gesture classifier (drives systolic array internally)
    // -------------------------------------------------------------------------
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

endmodule
