`timescale 1ns/1ps

// UART-only synthesis wrapper for iCEBreaker validation
// Events arrive as 5-byte UART packets; parallel EVT2 bus tied off

module gradient_map_top #(
    parameter CLK_FREQ_HZ     = 12_000_000,
    parameter BAUD_RATE       = 115200,
    parameter FRAME_PERIOD_MS = 50,
    parameter DECAY_SHIFT     = 6,
    parameter MIN_MASS_THRESH = 2000
)(
    input  logic clk,
    input  logic uart_rx,
    output logic uart_tx,
    output logic led_heartbeat,
    output logic led_activity,
    output logic led_up,
    output logic led_down,
    output logic led_left,
    output logic led_right
);

    // Internal power-on reset: rst_n goes high once counter saturates
    reg [4:0] por_cnt = 5'd0;
    wire rst_n_internal = &por_cnt;

    always @(posedge clk)
        if (!rst_n_internal)
            por_cnt <= por_cnt + 1'b1;

    gesture_top #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .BAUD_RATE      (BAUD_RATE),
        .FRAME_PERIOD_MS(FRAME_PERIOD_MS),
        .DECAY_SHIFT    (DECAY_SHIFT),
        .MIN_MASS_THRESH(MIN_MASS_THRESH),
        .UART_RX_ENABLE (1'b1)
    ) u_gesture (
        .clk          (clk),
        .rst_n        (rst_n_internal),
        .evt_data     (32'd0),
        .evt_valid    (1'b0),
        .evt_ready    (),
        .uart_rx      (uart_rx),
        .uart_tx      (uart_tx),
        .led_heartbeat(led_heartbeat),
        .led_activity (led_activity),
        .led_up       (led_up),
        .led_down     (led_down),
        .led_left     (led_left),
        .led_right    (led_right)
    );

endmodule
