`timescale 1ns/1ps

// =============================================================================
// Module: gradient_map_top
// =============================================================================
// UART-Only Synthesis Wrapper for Hardware Validation
//
// Wraps gesture_top with:
//   - UART_RX_ENABLE = 1 (5-byte event input via UART)
//   - Parallel EVT2 bus tied off (not used)
//   - Internal power-on reset (no external reset pin needed)
//
// Pin usage (iCEBreaker):
//   uart_rx (pin 6): Receives 5-byte event packets from PC
//   uart_tx (pin 9): Sends gesture ASCII output from uart_debug
//   LEDs: heartbeat, activity, direction indicators
//
// Target: Lattice iCE40UP5K on iCEBreaker board
// =============================================================================

module gradient_map_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter BAUD_RATE       = 115200,
    parameter FRAME_PERIOD_MS = 50,   // Increased for webcam emulation (was 10ms)
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000  // Increased for webcam emulation (was 100)
)(
    input  logic clk,

    // UART
    input  logic uart_rx,
    output logic uart_tx,

    // LEDs
    output logic led_heartbeat,
    output logic led_activity,
    output logic led_up,
    output logic led_down,
    output logic led_left,
    output logic led_right
);

    // -------------------------------------------------------------------------
    // Internal Power-On Reset
    // Counts up on every clock edge; rst_n goes high once counter saturates.
    // -------------------------------------------------------------------------
    reg [4:0] por_cnt = 5'd0;
    wire rst_n_internal = &por_cnt;

    always @(posedge clk)
        if (!rst_n_internal)
            por_cnt <= por_cnt + 1'b1;

    // -------------------------------------------------------------------------
    // Gesture Processing Core
    // UART_RX_ENABLE=1: events arrive as 5-byte UART packets
    // Parallel EVT2 bus is tied off (unused in UART mode)
    // -------------------------------------------------------------------------
    gesture_top #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .BAUD_RATE        (BAUD_RATE),
        .FRAME_PERIOD_MS  (FRAME_PERIOD_MS),
        .DECAY_SHIFT      (DECAY_SHIFT),
        .MIN_MASS_THRESH  (MIN_MASS_THRESH),
        .UART_RX_ENABLE   (1'b1)
    ) u_gesture (
        .clk            (clk),
        .rst_n          (rst_n_internal),

        // Parallel bus tied off
        .evt_data       (32'd0),
        .evt_valid      (1'b0),
        .evt_ready      (),

        // UART
        .uart_rx        (uart_rx),
        .uart_tx        (uart_tx),

        // LEDs
        .led_heartbeat  (led_heartbeat),
        .led_activity   (led_activity),
        .led_up         (led_up),
        .led_down       (led_down),
        .led_left       (led_left),
        .led_right      (led_right)
    );

endmodule
