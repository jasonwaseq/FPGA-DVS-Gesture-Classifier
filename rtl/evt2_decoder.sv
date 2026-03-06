`timescale 1ns/1ps

// EVT2.0 decoder with spatial compression from sensor coordinates to GRID_SIZE.
// EVT word fields:
//   [31:28] type, [27:22] ts_lsb, [21:11] x, [10:0] y
// Supported packet types:
//   0x0 CD OFF, 0x1 CD ON, 0x8 TIME_HIGH

module evt2_decoder #(
    parameter int GRID_BITS         = 4,
    parameter bit SWAP_INPUT_BYTES  = 1'b0,
    parameter int SENSOR_WIDTH      = 320,
    parameter int SENSOR_HEIGHT     = 320,
    parameter bit REQUIRE_TIME_HIGH = 1'b1
)(
    input  logic                 clk,
    input  logic                 rst,
    input  logic [31:0]          data_in,
    input  logic                 data_valid,
    input  logic                 event_ready_i,
    output logic                 data_ready,
    output logic [GRID_BITS-1:0] x_out,
    output logic [GRID_BITS-1:0] y_out,
    output logic                 polarity,
    output logic [33:0]          timestamp,
    output logic                 event_valid
);

    localparam logic [3:0] EVT_CD_OFF    = 4'h0;
    localparam logic [3:0] EVT_CD_ON     = 4'h1;
    localparam logic [3:0] EVT_TIME_HIGH = 4'h8;
    localparam int GRID_SIZE             = (1 << GRID_BITS);
    localparam int SENSOR_W_M1           = SENSOR_WIDTH  - 1;
    localparam int SENSOR_H_M1           = SENSOR_HEIGHT - 1;
    localparam int X_BIN_DIV             = (SENSOR_WIDTH  / GRID_SIZE);
    localparam int Y_BIN_DIV             = (SENSOR_HEIGHT / GRID_SIZE);

    wire [31:0] evt_word = SWAP_INPUT_BYTES
                         ? {data_in[7:0], data_in[15:8], data_in[23:16], data_in[31:24]}
                         : data_in;

    wire [3:0]  pkt_type          = evt_word[31:28];
    wire [5:0]  ts_lsb            = evt_word[27:22];
    wire [10:0] x_raw             = evt_word[21:11];
    wire [10:0] y_raw             = evt_word[10:0];
    wire [27:0] time_high_payload = evt_word[27:0];
    wire        is_cd             = (pkt_type == EVT_CD_OFF) || (pkt_type == EVT_CD_ON);

    logic [27:0] time_high_reg;
    logic        have_time_high;

    logic [10:0] x_clamped;
    logic [10:0] y_clamped;
    logic [GRID_BITS-1:0] x_grid;
    logic [GRID_BITS-1:0] y_grid;

    always_comb begin
        if (x_raw >= SENSOR_WIDTH)
            x_clamped = SENSOR_W_M1[10:0];
        else
            x_clamped = x_raw;

        if (y_raw >= SENSOR_HEIGHT)
            y_clamped = SENSOR_H_M1[10:0];
        else
            y_clamped = y_raw;

        x_grid = (x_clamped / X_BIN_DIV);
        y_grid = (y_clamped / Y_BIN_DIV);

        if (x_grid > GRID_SIZE-1)
            x_grid = GRID_SIZE-1;
        if (y_grid > GRID_SIZE-1)
            y_grid = GRID_SIZE-1;
    end

    // Backpressure only for CD events that generate downstream samples.
    assign data_ready = (!is_cd) || event_ready_i;

    always_ff @(posedge clk) begin
        if (rst) begin
            time_high_reg <= '0;
            have_time_high <= 1'b0;
            x_out         <= '0;
            y_out         <= '0;
            polarity      <= 1'b0;
            timestamp     <= '0;
            event_valid   <= 1'b0;
        end else begin
            event_valid <= 1'b0;

            if (data_valid && data_ready) begin
                case (pkt_type)
                    EVT_TIME_HIGH: begin
                        time_high_reg <= time_high_payload;
                        have_time_high <= 1'b1;
                    end

                    EVT_CD_OFF,
                    EVT_CD_ON: begin
                        if (!REQUIRE_TIME_HIGH || have_time_high) begin
                            x_out       <= x_grid;
                            y_out       <= y_grid;
                            polarity    <= (pkt_type == EVT_CD_ON);
                            timestamp   <= {time_high_reg, ts_lsb};
                            event_valid <= 1'b1;
                        end
                    end

                    default: begin
                        event_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
