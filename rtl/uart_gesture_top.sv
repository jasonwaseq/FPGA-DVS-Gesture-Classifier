// =============================================================================
// UART Top Module for DVS Gesture Accelerator
// Asynchronous Spatiotemporal Motion-Energy Classifier Interface
// =============================================================================
//
// UART Protocol (115200 baud, 8N1):
//   RX Commands:
//     - DVS Event: 5 bytes [X_HI, X_LO, Y_HI, Y_LO, POL]
//       X_HI[0] = X[8], X_LO = X[7:0] (9-bit coordinate, 0-319)
//       Y_HI[0] = Y[8], Y_LO = Y[7:0] (9-bit coordinate, 0-319)
//       POL[0] = polarity (1=ON, 0=OFF)
//     - Echo test: Send 0xFF -> Receive 0x55
//     - Status query: Send 0xFE -> Receive [0xB0 | state]
//     - Config query: Send 0xFD -> Receive [min_thresh, motion_thresh]
//     - Reset: Send 0xFC -> Soft reset accelerator
//   TX Responses:
//     - Gesture detected: [0xA0 | gesture, confidence]
//       gesture: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
//       confidence: 0-15
// =============================================================================

`timescale 1ns/1ps

module uart_gesture_top #(
    parameter CLK_FREQ       = 12_000_000,
    parameter BAUD_RATE      = 115200,
    parameter WINDOW_MS      = 400,            // 400ms observation window
    parameter GRID_SIZE      = 16,
    parameter SENSOR_RES     = 320,
    parameter MIN_EVENT_THRESH  = 20,          // Minimum events for gesture
    parameter MOTION_THRESH     = 8,           // Minimum motion magnitude
    parameter PERSISTENCE_COUNT = 2,           // Consecutive windows needed
    parameter CYCLES_PER_BIN = 600             // For simulation override
)(
    input  logic clk,
    input  logic uart_rx,
    output logic uart_tx,
    output logic led_heartbeat,
    output logic led_gesture_valid,
    output logic led_activity
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // =========================================================================
    // Power-On Reset
    // =========================================================================
    
    reg [4:0] por_cnt = 5'd0;
    wire rst_por = ~&por_cnt;
    logic soft_rst;
    wire rst = rst_por | soft_rst;
    
    always @(posedge clk) begin
        if (!(&por_cnt))
            por_cnt <= por_cnt + 1'b1;
    end

    // =========================================================================
    // UART RX/TX
    // =========================================================================
    
    logic [7:0] rx_data;
    logic rx_valid;
    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rx(uart_rx),
        .data(rx_data), .valid(rx_valid)
    );
    
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst), .data(tx_data), .valid(tx_valid),
        .tx(uart_tx), .busy(tx_busy)
    );

    // =========================================================================
    // DVS Gesture Accelerator
    // =========================================================================
    
    logic [1:0]  gesture;
    logic        gesture_valid;
    logic [3:0]  gesture_confidence;
    logic [7:0]  debug_event_count;
    logic [2:0]  debug_state;
    logic        debug_fifo_empty;
    logic        debug_fifo_full;
    logic        event_ready;
    
    logic        event_valid;
    logic [8:0]  event_x, event_y;
    logic        event_polarity;
    logic [15:0] event_ts;

    dvs_gesture_accel #(
        .CLK_FREQ_HZ(CLK_FREQ),
        .WINDOW_MS(WINDOW_MS),
        .GRID_SIZE(GRID_SIZE),
        .SENSOR_RES(SENSOR_RES),
        .MIN_EVENT_THRESH(MIN_EVENT_THRESH),
        .MOTION_THRESH(MOTION_THRESH),
        .PERSISTENCE_COUNT(PERSISTENCE_COUNT)
    ) u_accel (
        .clk(clk),
        .rst(rst),
        .event_valid(event_valid),
        .event_x(event_x),
        .event_y(event_y),
        .event_polarity(event_polarity),
        .event_ts(event_ts),
        .event_ready(event_ready),
        .gesture(gesture),
        .gesture_valid(gesture_valid),
        .gesture_confidence(gesture_confidence),
        .debug_event_count(debug_event_count),
        .debug_state(debug_state),
        .debug_fifo_empty(debug_fifo_empty),
        .debug_fifo_full(debug_fifo_full)
    );

    // =========================================================================
    // Heartbeat LED (~3Hz blink)
    // =========================================================================
    
    logic [23:0] heartbeat_cnt;
    always @(posedge clk)
        heartbeat_cnt <= rst ? 24'd0 : heartbeat_cnt + 1'b1;
    assign led_heartbeat = ~heartbeat_cnt[22];

    // =========================================================================
    // Gesture LED Indicator (pulse stretch for visibility)
    // =========================================================================
    
    logic [19:0] gesture_led_cnt;
    always @(posedge clk) begin
        if (rst)
            gesture_led_cnt <= '0;
        else if (gesture_valid)
            gesture_led_cnt <= {20{1'b1}};
        else if (gesture_led_cnt > 0)
            gesture_led_cnt <= gesture_led_cnt - 1'b1;
    end
    assign led_gesture_valid = ~(gesture_led_cnt > 0);

    // =========================================================================
    // Activity LED (blinks when events received)
    // =========================================================================
    
    logic [17:0] activity_led_cnt;
    always @(posedge clk) begin
        if (rst)
            activity_led_cnt <= '0;
        else if (event_valid)
            activity_led_cnt <= {18{1'b1}};
        else if (activity_led_cnt > 0)
            activity_led_cnt <= activity_led_cnt - 1'b1;
    end
    assign led_activity = ~(activity_led_cnt > 0);

    // =========================================================================
    // Packet RX State Machine (5-byte packets)
    // =========================================================================
    
    typedef enum logic [2:0] {
        PKT_X_HI, PKT_X_LO, PKT_Y_HI, PKT_Y_LO, PKT_POL
    } pkt_state_t;
    
    pkt_state_t pkt_state;
    logic [8:0] pkt_x, pkt_y;
    logic [15:0] pkt_ts;  // Timestamp counter

    // =========================================================================
    // TX State Machine
    // =========================================================================
    
    typedef enum logic [3:0] {
        TX_IDLE, 
        TX_ECHO, 
        TX_STATUS, 
        TX_GESTURE_CMD,
        TX_GESTURE_CMD_WAIT,
        TX_GESTURE_CONF,
        TX_CONFIG_1,
        TX_CONFIG_1_WAIT,
        TX_CONFIG_2
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [1:0] pending_gesture;
    logic [3:0] pending_confidence;
    logic [2:0] pending_state;

    // =========================================================================
    // Timestamp Counter (wrapping 16-bit counter for event timestamps)
    // =========================================================================
    
    logic [15:0] ts_counter;
    always @(posedge clk) begin
        if (rst)
            ts_counter <= '0;
        else
            ts_counter <= ts_counter + 1'b1;
    end

    // =========================================================================
    // Main Control Logic
    // =========================================================================
    
    always @(posedge clk) begin
        if (rst) begin
            pkt_state         <= PKT_X_HI;
            pkt_x             <= 9'd0;
            pkt_y             <= 9'd0;
            pkt_ts            <= 16'd0;
            event_valid       <= 1'b0;
            event_x           <= 9'd0;
            event_y           <= 9'd0;
            event_polarity    <= 1'b0;
            event_ts          <= 16'd0;
            tx_state          <= TX_IDLE;
            tx_data           <= 8'd0;
            tx_valid          <= 1'b0;
            pending_gesture   <= 2'd0;
            pending_confidence<= 4'd0;
            pending_state     <= 3'd0;
            soft_rst          <= 1'b0;
        end else begin
            // Defaults
            event_valid <= 1'b0;
            tx_valid    <= 1'b0;
            soft_rst    <= 1'b0;
            
            // Handle RX bytes
            if (rx_valid) begin
                case (pkt_state)
                    PKT_X_HI: begin
                        case (rx_data)
                            8'hFF: begin
                                // Echo command
                                if (tx_state == TX_IDLE)
                                    tx_state <= TX_ECHO;
                            end
                            8'hFE: begin
                                // Status query
                                if (tx_state == TX_IDLE) begin
                                    pending_state <= debug_state;
                                    tx_state <= TX_STATUS;
                                end
                            end
                            8'hFD: begin
                                // Config query
                                if (tx_state == TX_IDLE) begin
                                    tx_state <= TX_CONFIG_1;
                                end
                            end
                            8'hFC: begin
                                // Soft reset
                                soft_rst <= 1'b1;
                            end
                            default: begin
                                // Normal packet - X high byte
                                pkt_x[8] <= rx_data[0];
                                pkt_state <= PKT_X_LO;
                            end
                        endcase
                    end
                    PKT_X_LO: begin
                        pkt_x[7:0] <= rx_data;
                        pkt_state <= PKT_Y_HI;
                    end
                    PKT_Y_HI: begin
                        pkt_y[8] <= rx_data[0];
                        pkt_state <= PKT_Y_LO;
                    end
                    PKT_Y_LO: begin
                        pkt_y[7:0] <= rx_data;
                        pkt_state <= PKT_POL;
                    end
                    PKT_POL: begin
                        // Complete packet - send to accelerator
                        if (event_ready) begin
                            event_x <= pkt_x;
                            event_y <= pkt_y;
                            event_polarity <= rx_data[0];
                            event_ts <= ts_counter;
                            event_valid <= 1'b1;
                        end
                        pkt_state <= PKT_X_HI;
                    end
                    default: pkt_state <= PKT_X_HI;
                endcase
            end
            
            // Capture gesture for TX (two-byte response)
            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture <= gesture;
                pending_confidence <= gesture_confidence;
                tx_state <= TX_GESTURE_CMD;
            end
            
            // TX state machine
            case (tx_state)
                TX_IDLE: ;  // Wait
                
                TX_ECHO: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h55;
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_STATUS: begin
                    if (!tx_busy) begin
                        // Status byte: 0xB0 | state[2:0] | fifo_empty
                        // Bits: [7:4]=0xB, [3:1]=state, [0]=fifo_empty
                        tx_data  <= {4'hB, pending_state, debug_fifo_empty};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_GESTURE_CMD: begin
                    if (!tx_busy) begin
                        // Gesture command byte: 0xA0 | gesture[1:0]
                        tx_data  <= {4'hA, 2'b00, pending_gesture};
                        tx_valid <= 1'b1;
                        tx_state <= TX_GESTURE_CMD_WAIT;
                    end
                end
                
                TX_GESTURE_CMD_WAIT: begin
                    // Wait for UART TX to assert busy before sending next byte
                    if (tx_busy)
                        tx_state <= TX_GESTURE_CONF;
                end
                
                TX_GESTURE_CONF: begin
                    if (!tx_busy) begin
                        // Confidence byte: confidence[3:0] | event_count[3:0]
                        tx_data  <= {pending_confidence, debug_event_count[7:4]};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_CONFIG_1: begin
                    if (!tx_busy) begin
                        tx_data  <= MIN_EVENT_THRESH[7:0];
                        tx_valid <= 1'b1;
                        tx_state <= TX_CONFIG_1_WAIT;
                    end
                end
                
                TX_CONFIG_1_WAIT: begin
                    // Wait for UART TX to assert busy before sending next byte
                    if (tx_busy)
                        tx_state <= TX_CONFIG_2;
                end
                
                TX_CONFIG_2: begin
                    if (!tx_busy) begin
                        tx_data  <= MOTION_THRESH[7:0];
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
