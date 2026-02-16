`timescale 1ns/1ps

// =============================================================================
// Module: uart_debug
// =============================================================================
// UART Debug Output for Gesture Classification Results
//
// Output Format:
//   When a valid gesture is detected, sends ASCII string:
//   "UP\r\n", "DOWN\r\n", "LEFT\r\n", or "RIGHT\r\n"
//
// Interface:
//   - Receives gesture_class and gesture_valid from classifier
//   - Uses existing uart_tx module for transmission
//   - Queues messages to avoid dropping gestures during transmission
// =============================================================================

module uart_debug #(
    parameter CLK_FREQ_HZ = 12_000_000,
    parameter BAUD_RATE   = 115200
)(
    input  logic        clk,
    input  logic        rst,
    
    // Gesture Input
    input  logic [1:0]  gesture_class,      // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    input  logic        gesture_valid,      // Valid pulse
    input  logic [7:0]  gesture_confidence, // Confidence value
    
    // UART TX Output
    output logic        uart_tx
);

    localparam CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    
    // -------------------------------------------------------------------------
    // Message Storage (ROM)
    // Each message: up to 7 characters including \r\n
    // -------------------------------------------------------------------------
    // Message bytes for each gesture
    // UP\r\n     = 'U', 'P', '\r', '\n'
    // DOWN\r\n   = 'D', 'O', 'W', 'N', '\r', '\n'
    // LEFT\r\n   = 'L', 'E', 'F', 'T', '\r', '\n'
    // RIGHT\r\n  = 'R', 'I', 'G', 'H', 'T', '\r', '\n'
    
    localparam MSG_UP_LEN    = 4;
    localparam MSG_DOWN_LEN  = 6;
    localparam MSG_LEFT_LEN  = 6;
    localparam MSG_RIGHT_LEN = 7;
    localparam MAX_MSG_LEN   = 7;
    
    // Message ROM
    logic [7:0] msg_up    [0:3];
    logic [7:0] msg_down  [0:5];
    logic [7:0] msg_left  [0:5];
    logic [7:0] msg_right [0:6];
    
    // Initialize message ROMs
    initial begin
        msg_up[0]    = "U"; msg_up[1]    = "P"; 
        msg_up[2]    = 8'h0D; msg_up[3]  = 8'h0A;  // \r\n
        
        msg_down[0]  = "D"; msg_down[1]  = "O"; 
        msg_down[2]  = "W"; msg_down[3]  = "N";
        msg_down[4]  = 8'h0D; msg_down[5] = 8'h0A;
        
        msg_left[0]  = "L"; msg_left[1]  = "E";
        msg_left[2]  = "F"; msg_left[3]  = "T";
        msg_left[4]  = 8'h0D; msg_left[5] = 8'h0A;
        
        msg_right[0] = "R"; msg_right[1] = "I";
        msg_right[2] = "G"; msg_right[3] = "H";
        msg_right[4] = "T"; msg_right[5] = 8'h0D; 
        msg_right[6] = 8'h0A;
    end
    
    // -------------------------------------------------------------------------
    // FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,      // Wait for gesture
        S_LOAD = 2'd1,      // Load message byte
        S_SEND = 2'd2,      // Wait for TX complete
        S_NEXT = 2'd3       // Move to next byte
    } state_t;
    
    state_t state;
    
    // -------------------------------------------------------------------------
    // Message Transmission State
    // -------------------------------------------------------------------------
    logic [1:0] current_gesture;        // Latched gesture class
    logic [2:0] byte_idx;               // Current byte index [0-6]
    logic [2:0] msg_length;             // Length of current message
    logic [7:0] current_byte;           // Current byte to send
    
    // -------------------------------------------------------------------------
    // UART TX Interface
    // -------------------------------------------------------------------------
    logic [7:0] tx_data;
    logic       tx_valid;
    logic       tx_busy;
    
    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_tx (
        .clk   (clk),
        .rst   (rst),
        .data  (tx_data),
        .valid (tx_valid),
        .tx    (uart_tx),
        .busy  (tx_busy)
    );
    
    // -------------------------------------------------------------------------
    // Get message length for gesture class
    // -------------------------------------------------------------------------
    function automatic [2:0] get_msg_length(input [1:0] gesture);
        case (gesture)
            2'd0: get_msg_length = MSG_UP_LEN;
            2'd1: get_msg_length = MSG_DOWN_LEN;
            2'd2: get_msg_length = MSG_LEFT_LEN;
            2'd3: get_msg_length = MSG_RIGHT_LEN;
        endcase
    endfunction
    
    // -------------------------------------------------------------------------
    // Get message byte for gesture class and index
    // -------------------------------------------------------------------------
    function automatic [7:0] get_msg_byte(input [1:0] gesture, input [2:0] idx);
        case (gesture)
            2'd0: get_msg_byte = msg_up[idx];
            2'd1: get_msg_byte = msg_down[idx];
            2'd2: get_msg_byte = msg_left[idx];
            2'd3: get_msg_byte = msg_right[idx];
        endcase
    endfunction
    
    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state           <= S_IDLE;
            current_gesture <= '0;
            byte_idx        <= '0;
            msg_length      <= '0;
            tx_data         <= '0;
            tx_valid        <= 1'b0;
        end else begin
            // Default: don't start new transmission
            tx_valid <= 1'b0;
            
            case (state)
                // ---------------------------------------------------------
                // IDLE: Wait for valid gesture
                // ---------------------------------------------------------
                S_IDLE: begin
                    if (gesture_valid && !tx_busy) begin
                        current_gesture <= gesture_class;
                        msg_length <= get_msg_length(gesture_class);
                        byte_idx <= '0;
                        state <= S_LOAD;
                    end
                end
                
                // ---------------------------------------------------------
                // LOAD: Load current byte and start transmission
                // ---------------------------------------------------------
                S_LOAD: begin
                    // Get byte from message ROM
                    case (current_gesture)
                        2'd0: current_byte <= msg_up[byte_idx];
                        2'd1: current_byte <= msg_down[byte_idx];
                        2'd2: current_byte <= msg_left[byte_idx];
                        2'd3: current_byte <= msg_right[byte_idx];
                    endcase
                    state <= S_SEND;
                end
                
                // ---------------------------------------------------------
                // SEND: Start transmission and wait for completion
                // ---------------------------------------------------------
                S_SEND: begin
                    if (!tx_busy) begin
                        tx_data <= current_byte;
                        tx_valid <= 1'b1;
                        state <= S_NEXT;
                    end
                end
                
                // ---------------------------------------------------------
                // NEXT: Wait for TX to start, then advance
                // ---------------------------------------------------------
                S_NEXT: begin
                    if (tx_busy) begin
                        // TX has started, check if more bytes
                        if (byte_idx >= msg_length - 1) begin
                            // Message complete, wait for TX to finish
                            state <= S_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            state <= S_LOAD;
                        end
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
