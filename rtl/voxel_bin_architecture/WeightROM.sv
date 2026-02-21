`timescale 1ns/1ps

// Weight ROM for one gesture class. Address = bin * 256 + spatial_addr.
// Spatial pattern replicated across all time bins.

module WeightROM #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CELLS   = 1024,
    parameter GRID_SIZE   = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    logic signed [WEIGHT_BITS-1:0] rom [0:NUM_CELLS-1];

    integer i, cx, cy;
    integer cells_per_bin  = GRID_SIZE * GRID_SIZE;
    integer spatial_addr;
    integer centre         = GRID_SIZE / 2;
    integer dist_from_edge;
    integer raw_val;

    initial begin
        for (i = 0; i < NUM_CELLS; i = i + 1) begin
            spatial_addr = i % cells_per_bin;
            cy = spatial_addr / GRID_SIZE;
            cx = spatial_addr % GRID_SIZE;

            case (CLASS_IDX)
                0: begin // UP
                    if (cy < centre) begin
                        dist_from_edge = centre - cy;
                        raw_val = dist_from_edge * 12;
                    end else begin
                        dist_from_edge = cy - centre + 1;
                        raw_val = -(dist_from_edge * 8);
                    end
                end
                1: begin // DOWN
                    if (cy >= centre) begin
                        dist_from_edge = cy - centre + 1;
                        raw_val = dist_from_edge * 12;
                    end else begin
                        dist_from_edge = centre - cy;
                        raw_val = -(dist_from_edge * 8);
                    end
                end
                2: begin // LEFT
                    if (cx < centre) begin
                        dist_from_edge = centre - cx;
                        raw_val = dist_from_edge * 12;
                    end else begin
                        dist_from_edge = cx - centre + 1;
                        raw_val = -(dist_from_edge * 8);
                    end
                end
                3: begin // RIGHT
                    if (cx >= centre) begin
                        dist_from_edge = cx - centre + 1;
                        raw_val = dist_from_edge * 12;
                    end else begin
                        dist_from_edge = centre - cx;
                        raw_val = -(dist_from_edge * 8);
                    end
                end
                default: raw_val = 0;
            endcase

            if      (raw_val >  127) rom[i] = 8'sd127;
            else if (raw_val < -128) rom[i] = -8'sd128;
            else                    rom[i] = WEIGHT_BITS'(raw_val);
        end
    end

    always_ff @(posedge clk) begin
        dout <= rom[cell_addr];
    end

endmodule
