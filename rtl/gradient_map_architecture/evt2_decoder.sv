`timescale 1ns/1ps

// =============================================================================
// Module: evt2_decoder
// =============================================================================
// EVT 2.0 Protocol Decoder for Prophesee GenX320 Sensor
//
// EVT 2.0 Packet Format (32-bit words):
//   Type [31:28] | Payload [27:0]
//
// Packet Types:
//   0x0 (CD_OFF):     Negative polarity event
//   0x1 (CD_ON):      Positive polarity event
//   0x8 (EVT_TIME_HIGH): Upper timestamp bits
//   0xA (EXT_TRIGGER): External trigger (ignored)
//   0xE (OTHERS):     Miscellaneous (ignored)
//   0xF (CONTINUED):  Continuation (ignored)
//
// CD Event Payload [27:0]:
//   [27:22] = timestamp LSB (6 bits)
//   [21:11] = X coordinate (11 bits, 0-319 for GenX320)
//   [10:0]  = Y coordinate (11 bits, 0-319 for GenX320)
//
// Timestamp Reconstruction:
//   Full 34-bit timestamp = {time_high[27:0], timestamp_lsb[5:0]}
//   time_high updated by EVT_TIME_HIGH packets
//
// Output Downsampling:
//   320×320 sensor → 32×32 grid
//   X_grid = min(x_raw >> 3, 31)   (≈10 sensor pixels per grid cell)
//   Y_grid = min(y_raw >> 3, 31)
// =============================================================================

module evt2_decoder #(
    parameter GRID_BITS = 5                 // Output grid coordinate bits (5 → 32 cells)
)(
    input  logic        clk,
    input  logic        rst,
    
    // Input from FIFO
    input  logic [31:0] data_in,            // EVT 2.0 word
    input  logic        data_valid,         // Word available
    output logic        data_ready,         // Consumed this cycle
    
    // Output: Decoded & Downsampled Event
    output logic [GRID_BITS-1:0] x_out,     // Grid X [0-15]
    output logic [GRID_BITS-1:0] y_out,     // Grid Y [0-15]
    output logic                 polarity,  // 1=ON, 0=OFF
    output logic [15:0]          timestamp, // Lower 16 bits of timestamp
    output logic                 event_valid // Valid event output
);

    // -------------------------------------------------------------------------
    // EVT 2.0 Packet Type Definitions
    // -------------------------------------------------------------------------
    localparam [3:0] EVT_CD_OFF      = 4'h0;    // Negative polarity event
    localparam [3:0] EVT_CD_ON       = 4'h1;    // Positive polarity event
    localparam [3:0] EVT_TIME_HIGH   = 4'h8;    // Timestamp high bits
    localparam [3:0] EVT_EXT_TRIGGER = 4'hA;    // External trigger
    localparam [3:0] EVT_OTHERS      = 4'hE;    // Miscellaneous
    localparam [3:0] EVT_CONTINUED   = 4'hF;    // Continuation
    
    // -------------------------------------------------------------------------
    // Packet Field Extraction
    // -------------------------------------------------------------------------
    wire [3:0]  pkt_type   = data_in[31:28];    // Packet type
    wire [5:0]  ts_lsb     = data_in[27:22];    // Timestamp LSB (6 bits)
    wire [10:0] x_raw      = data_in[21:11];    // Raw X coordinate
    wire [10:0] y_raw      = data_in[10:0];     // Raw Y coordinate
    wire [27:0] time_high_payload = data_in[27:0]; // TIME_HIGH payload
    
    // -------------------------------------------------------------------------
    // Internal State
    // -------------------------------------------------------------------------
    logic [27:0] time_high_reg;                 // Stored high timestamp bits
    
    // -------------------------------------------------------------------------
    // Downsampling: 320×320 → 32×32
    // Strategy: x_raw >> 3  (divide by 8) → range 0..39 for 320 coords
    //           clamp to 31 so all 32 cells are used.
    // Cell 0..31 each covers ≈10 sensor pixels; cells beyond 255px clamp to 31.
    // -------------------------------------------------------------------------
    wire [5:0] x_grid_raw6 = x_raw[8:3];   // 6 bits, value 0..39
    wire [5:0] y_grid_raw6 = y_raw[8:3];

    wire [GRID_BITS-1:0] x_grid = (x_grid_raw6 > 6'd31) ? 5'd31 : x_grid_raw6[GRID_BITS-1:0];
    wire [GRID_BITS-1:0] y_grid = (y_grid_raw6 > 6'd31) ? 5'd31 : y_grid_raw6[GRID_BITS-1:0];
    
    // -------------------------------------------------------------------------
    // Timestamp Reconstruction
    // Full timestamp = {time_high[9:0], ts_lsb[5:0]} for 16-bit output
    // (Using lower bits for practical 16-bit timestamp representation)
    // -------------------------------------------------------------------------
    wire [15:0] full_ts = {time_high_reg[9:0], ts_lsb};
    
    // -------------------------------------------------------------------------
    // Event Detection
    // -------------------------------------------------------------------------
    wire is_cd_event = (pkt_type == EVT_CD_OFF) || (pkt_type == EVT_CD_ON);
    wire is_time_high = (pkt_type == EVT_TIME_HIGH);
    
    // -------------------------------------------------------------------------
    // Always ready to consume data (single-cycle processing)
    // -------------------------------------------------------------------------
    assign data_ready = data_valid;
    
    // -------------------------------------------------------------------------
    // Main Decode Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            time_high_reg <= '0;
            x_out         <= '0;
            y_out         <= '0;
            polarity      <= '0;
            timestamp     <= '0;
            event_valid   <= '0;
        end else begin
            // Default: no valid event
            event_valid <= 1'b0;
            
            if (data_valid) begin
                case (pkt_type)
                    EVT_TIME_HIGH: begin
                        // Update timestamp high bits
                        time_high_reg <= time_high_payload;
                    end
                    
                    EVT_CD_OFF: begin
                        // Negative polarity event
                        x_out       <= x_grid;
                        y_out       <= y_grid;
                        polarity    <= 1'b0;
                        timestamp   <= full_ts;
                        event_valid <= 1'b1;
                    end
                    
                    EVT_CD_ON: begin
                        // Positive polarity event
                        x_out       <= x_grid;
                        y_out       <= y_grid;
                        polarity    <= 1'b1;
                        timestamp   <= full_ts;
                        event_valid <= 1'b1;
                    end
                    
                    default: begin
                        // Ignore other packet types
                        event_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
