`timescale 1ns/1ps

// Top-level for Time-Surface DVS Gesture Classifier
// Target: Lattice iCE40UP5K, Sensor: Prophesee GenX320 (EVT 2.0)

module gesture_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter BAUD_RATE       = 115200,
    parameter FRAME_PERIOD_MS = 10,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 100,
    parameter UART_RX_ENABLE  = 1'b0
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [31:0] evt_data,
    input  logic        evt_valid,
    output logic        evt_ready,
    input  logic        uart_rx,
    output logic        uart_tx,
    output logic        led_heartbeat,
    output logic        led_activity,
    output logic        led_up,
    output logic        led_down,
    output logic        led_left,
    output logic        led_right
);

    // Synchronize active-low reset to active-high
    logic rst;
    logic [3:0] rst_sync;

    always_ff @(posedge clk) begin
        rst_sync <= {rst_sync[2:0], ~rst_n};
        rst <= rst_sync[3];
    end

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
    logic        fifo_wr_en;
    logic [31:0] fifo_wr_data;

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
                    uart_byte_idx    <= 3'd0;
                    uart_x_hi        <= 8'd0;
                    uart_x_lo        <= 8'd0;
                    uart_y_hi        <= 8'd0;
                    uart_y_lo        <= 8'd0;
                    uart_pol         <= 8'd0;
                    uart_evt_word    <= 32'd0;
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
                uart_rx_data  = 8'd0;
                uart_rx_valid = 1'b0;
            end

            always_ff @(posedge clk) begin
                if (rst) begin
                    uart_byte_idx    <= 3'd0;
                    uart_x_hi        <= 8'd0;
                    uart_x_lo        <= 8'd0;
                    uart_y_hi        <= 8'd0;
                    uart_y_lo        <= 8'd0;
                    uart_pol         <= 8'd0;
                    uart_evt_word    <= 32'd0;
                    uart_evt_pending <= 1'b0;
                end else begin
                    uart_evt_pending <= 1'b0;
                end
            end
        end
    endgenerate

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
        .count   ()
    );

    always_comb begin
        if (UART_RX_ENABLE && uart_evt_pending) begin
            fifo_wr_en   = !fifo_full;
            fifo_wr_data = uart_evt_word;
        end else begin
            fifo_wr_en   = evt_valid && !fifo_full;
            fifo_wr_data = evt_data;
        end
    end

    assign evt_ready = !fifo_full && !(UART_RX_ENABLE && uart_evt_pending);

    logic       decoder_ready;
    logic [3:0] decoded_x, decoded_y;
    logic       decoded_polarity;
    logic [15:0] decoded_timestamp;
    logic       decoded_valid;
    logic        fifo_rd_valid;

    evt2_decoder #(
        .GRID_BITS(4)
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

    assign fifo_rd_en = !fifo_empty;

    always_ff @(posedge clk) begin
        if (rst)
            fifo_rd_valid <= 1'b0;
        else
            fifo_rd_valid <= fifo_rd_en && !fifo_empty;
    end

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

    // Heartbeat ~1.5 Hz at 12 MHz
    logic [22:0] heartbeat_cnt;

    always_ff @(posedge clk) begin
        if (rst)
            heartbeat_cnt <= '0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end

    assign led_heartbeat = heartbeat_cnt[22];

    // Activity LED: pulses on events
    logic [19:0] activity_cnt;

    always_ff @(posedge clk) begin
        if (rst)
            activity_cnt <= '0;
        else if (decoded_valid)
            activity_cnt <= {20{1'b1}};
        else if (activity_cnt > 0)
            activity_cnt <= activity_cnt - 1'b1;
    end

    assign led_activity = (activity_cnt > 0);

    // Gesture direction LEDs with ~500ms pulse stretch
    logic [23:0] led_up_cnt, led_down_cnt, led_left_cnt, led_right_cnt;
    localparam LED_STRETCH = 24'd6_000_000;

    always_ff @(posedge clk) begin
        if (rst) begin
            led_up_cnt    <= '0;
            led_down_cnt  <= '0;
            led_left_cnt  <= '0;
            led_right_cnt <= '0;
        end else begin
            if (gesture_valid && gesture_class == 2'd0)
                led_up_cnt <= LED_STRETCH;
            else if (led_up_cnt > 0)
                led_up_cnt <= led_up_cnt - 1'b1;

            if (gesture_valid && gesture_class == 2'd1)
                led_down_cnt <= LED_STRETCH;
            else if (led_down_cnt > 0)
                led_down_cnt <= led_down_cnt - 1'b1;

            if (gesture_valid && gesture_class == 2'd2)
                led_left_cnt <= LED_STRETCH;
            else if (led_left_cnt > 0)
                led_left_cnt <= led_left_cnt - 1'b1;

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
