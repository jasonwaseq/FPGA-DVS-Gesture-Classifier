`timescale 1ns/1ps

// =============================================================================
// Module: WeightROM
// =============================================================================
// Weights for Spatio-Temporal Classification
//
// Description:
//   Stores weights for one class.
//   Address Space: NUM_BINS * GRID_SIZE * GRID_SIZE
//   Mapping: Bin major or Cell major?
//   TSB reads: Bin 0 (Cell 0..255), Bin 1 (Cell 0..255)...
//   So Address = BinIdx * 256 + GridIdx.
//
//   Weight Generation:
//   Replicates the 2D spatial pattern across all time bins.
//
// =============================================================================

module WeightROM #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CELLS   = 1280,   // 5 bins * 256
    parameter GRID_SIZE   = 16,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    // -------------------------------------------------------------------------
    // ROM Array
    // -------------------------------------------------------------------------
    logic signed [WEIGHT_BITS-1:0] rom [0:NUM_CELLS-1];

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------
    integer i, cx, cy, bin;
    integer cells_per_bin = GRID_SIZE * GRID_SIZE;
    integer spatial_addr;
    integer centre = GRID_SIZE / 2;
    integer dist_from_edge;
    integer raw_val;

    initial begin
        for (i = 0; i < NUM_CELLS; i = i + 1) begin
            // Decode address to spatial (x,y)
            // addr = bin * 256 + spatial_addr
            spatial_addr = i % cells_per_bin;
            cy = spatial_addr / GRID_SIZE;
            cx = spatial_addr % GRID_SIZE;
            
            // Generate pattern (copied/adapted from weight_rom.sv for 16x16)
             case (CLASS_IDX)
                0: begin // UP — top half positive, bottom negative
                    if (cy < centre) begin
                        dist_from_edge = centre - cy; 
                        raw_val = dist_from_edge * 12; // Scaled for 16x16 (smaller grid, higher scaler?)
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

            // Saturate
            if      (raw_val >  127) rom[i] = 8'sd127;
            else if (raw_val < -128) rom[i] = -8'sd128;
            else                    rom[i] = WEIGHT_BITS'(raw_val);
        end
    end

    // -------------------------------------------------------------------------
    // Synchronous Read
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        dout <= rom[cell_addr];
    end

endmodule
