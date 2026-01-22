// UART Top Wrapper for DVS Gesture Accelerator
// Receives DVS events via UART, sends gesture results back
//
// UART Protocol (115200 baud, 8N1):
//   RX: 4 bytes per event [X, Y, POL, TS_unused]
//   TX: 1 byte when gesture detected [0xA0 | gesture]
//
// TEST MODE: Send 0xFF to get 0x55 back (verifies UART works)
//
// CRITICAL: Uses internal power-on reset (no external reset pin needed)
// This matches the proven working design from jasonwaseq/fpga_sorting

module uart_gesture_top #(
    parameter CLK_FREQ = 12_000_000,
    parameter BAUD_RATE = 115200,
    parameter WINDOW_EVENTS = 100
)(
    input  logic clk,
    
    // UART interface
    input  logic uart_rx,
    output logic uart_tx,
    
    // Debug LED (active low for iCEBreaker accent LEDs)
    output logic led_heartbeat
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // =========================================================
    // Power-on reset (synchronous, active-high)
    // Same approach as proven working fpga_sorting design
    // Holds reset high for first 15 clock cycles
    // =========================================================
    reg [3:0] por_cnt = 4'd0;
    wire      rst;
    
    always @(posedge clk) begin
        if (&por_cnt)
            por_cnt <= por_cnt;
        else
            por_cnt <= por_cnt + 1'b1;
    end
    assign rst = ~&por_cnt;  // Active high reset for first 15 clocks

    // UART RX signals
    logic [7:0] rx_data;
    logic rx_valid;
    
    // UART TX signals
    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;
    
    // Gesture output
    logic [1:0] gesture;
    logic gesture_valid;
    
    // Heartbeat counter (shows FPGA is running)
    logic [23:0] heartbeat_cnt;
    always @(posedge clk) begin
        if (rst)
            heartbeat_cnt <= 24'd0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end
    assign led_heartbeat = ~heartbeat_cnt[22];  // Active low for iCEBreaker
    
    // State for TX
    typedef enum logic [1:0] {
        TX_IDLE,
        TX_SEND_ECHO,
        TX_SEND_GESTURE
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [1:0] pending_gesture;
    
    // Packet assembly state
    logic [1:0] byte_cnt;
    logic [6:0] saved_x, saved_y;
    logic saved_pol;
    
    // Event to accelerator
    logic event_valid;
    logic [6:0] event_x, event_y;
    logic event_polarity;

    // UART Receiver (synchronous reset, active high)
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_rx (
        .clk(clk),
        .rst(rst),
        .rx(uart_rx),
        .data(rx_data),
        .valid(rx_valid)
    );
    
    // UART Transmitter (synchronous reset, active high)
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_tx (
        .clk(clk),
        .rst(rst),
        .data(tx_data),
        .valid(tx_valid),
        .tx(uart_tx),
        .busy(tx_busy)
    );

    // DVS Gesture Accelerator (synchronous reset, active high)
    dvs_gesture_accel #(
        .X_BITS(7),
        .Y_BITS(7),
        .TS_BITS(16),
        .WINDOW_EVENTS(WINDOW_EVENTS)
    ) u_accel (
        .clk(clk),
        .rst(rst),
        .event_valid(event_valid),
        .event_x(event_x),
        .event_y(event_y),
        .event_polarity(event_polarity),
        .event_ts(16'd0),
        .gesture(gesture),
        .gesture_valid(gesture_valid)
    );

    // Main state machine - handles RX packets and TX responses
    // SYNCHRONOUS reset (active high) - critical for iCE40
    always @(posedge clk) begin
        if (rst) begin
            byte_cnt <= 2'd0;
            saved_x <= 7'd0;
            saved_y <= 7'd0;
            saved_pol <= 1'b0;
            event_valid <= 1'b0;
            event_x <= 7'd0;
            event_y <= 7'd0;
            event_polarity <= 1'b0;
            tx_state <= TX_IDLE;
            tx_data <= 8'd0;
            tx_valid <= 1'b0;
            pending_gesture <= 2'd0;
        end else begin
            // Default: clear single-cycle signals
            event_valid <= 1'b0;
            tx_valid <= 1'b0;
            
            // Handle received bytes
            if (rx_valid) begin
                if (byte_cnt == 2'd0 && rx_data == 8'hFF) begin
                    // Echo test mode - respond with 0x55
                    if (tx_state == TX_IDLE) begin
                        tx_state <= TX_SEND_ECHO;
                    end
                end else begin
                    // Normal packet assembly
                    case (byte_cnt)
                        2'd0: begin
                            saved_x <= rx_data[6:0];
                            byte_cnt <= 2'd1;
                        end
                        2'd1: begin
                            saved_y <= rx_data[6:0];
                            byte_cnt <= 2'd2;
                        end
                        2'd2: begin
                            saved_pol <= rx_data[0];
                            byte_cnt <= 2'd3;
                        end
                        2'd3: begin
                            event_x <= saved_x;
                            event_y <= saved_y;
                            event_polarity <= saved_pol;
                            event_valid <= 1'b1;
                            byte_cnt <= 2'd0;
                        end
                    endcase
                end
            end
            
            // Capture gesture when detected
            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture <= gesture;
                tx_state <= TX_SEND_GESTURE;
            end
            
            // TX state machine
            case (tx_state)
                TX_IDLE: begin
                    // Wait for something to send
                end
                
                TX_SEND_ECHO: begin
                    if (!tx_busy) begin
                        tx_data <= 8'h55;
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_SEND_GESTURE: begin
                    if (!tx_busy) begin
                        tx_data <= {4'hA, 2'b00, pending_gesture};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
