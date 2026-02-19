`timescale 1ns/1ps

// =============================================================================
// Module: weight_rom
// =============================================================================
// Single-Class Weight ROM
//
// Stores learned gesture weights for ONE gesture class (CLASS_IDX).
// Instantiate four of these (one per class) for parallel inference.
//
// Parameters:
//   CLASS_IDX   - Which gesture class this ROM stores (0=UP, 1=DOWN,
//                 2=LEFT, 3=RIGHT)
//   NUM_CLASSES - Total number of gesture classes (4)
//   NUM_CELLS   - Total grid cells = GRID_SIZE² (1024 for 32×32)
//   GRID_SIZE   - Grid dimension (32)
//   WEIGHT_BITS - Signed weight width (8)
//
// Weight Encoding (geometric average):
//   Each cell's weight reflects how strongly it correlates with the gesture:
//     UP    → top half of grid positive, bottom negative
//     DOWN  → bottom half positive, top negative
//     LEFT  → left half positive, right negative
//     RIGHT → right half positive, left negative
//   Distance bonus amplifies cells deeper in each gesture's active zone.
//   Values clamped to [-128, 127].
//
// Read Interface:
//   cell_addr [9:0] — synchronous read; dout valid next cycle
// =============================================================================

module weight_rom #(
    parameter CLASS_IDX   = 0,
    parameter NUM_CLASSES = 4,
    parameter NUM_CELLS   = 1024,
    parameter GRID_SIZE   = 32,
    parameter WEIGHT_BITS = 8
)(
    input  logic                          clk,
    input  logic                          rst,
    input  logic [$clog2(NUM_CELLS)-1:0]  cell_addr,
    output logic signed [WEIGHT_BITS-1:0] dout
);

    // -------------------------------------------------------------------------
    // Weight ROM — 1024-entry, signed 8-bit
    // Initialised using an 'initial' block so synthesis can infer BRAM/LUT-RAM
    // -------------------------------------------------------------------------
    logic signed [WEIGHT_BITS-1:0] rom [0:NUM_CELLS-1];

    // Generate analytical weights at elaboration time
    // Pattern per class:
    //   positive_zone : signed value = +clamp(distance_bonus, 10, 120)
    //   negative_zone : signed value = -clamp(distance_bonus, 10, 80)
    //   where distance_bonus = how far the cell is from the border of its zone
    integer cx, cy, addr, centre, dist_from_edge;
    integer raw_val;

    initial begin
        // Centre of grid
        centre = GRID_SIZE / 2;   // 16 for 32-cell grid

        for (cy = 0; cy < GRID_SIZE; cy = cy + 1) begin
            for (cx = 0; cx < GRID_SIZE; cx = cx + 1) begin
                addr = cy * GRID_SIZE + cx;

                case (CLASS_IDX)
                    0: begin // UP — top half positive, bottom negative
                        if (cy < centre) begin
                            dist_from_edge = centre - cy;  // 1..16
                            raw_val = dist_from_edge * 6;  // 6..96
                        end else begin
                            dist_from_edge = cy - centre + 1;
                            raw_val = -(dist_from_edge * 4); // -4..-64
                        end
                    end
                    1: begin // DOWN — bottom half positive, top negative
                        if (cy >= centre) begin
                            dist_from_edge = cy - centre + 1;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = centre - cy;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    2: begin // LEFT — left half positive, right negative
                        if (cx < centre) begin
                            dist_from_edge = centre - cx;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = cx - centre + 1;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    3: begin // RIGHT — right half positive, left negative
                        if (cx >= centre) begin
                            dist_from_edge = cx - centre + 1;
                            raw_val = dist_from_edge * 6;
                        end else begin
                            dist_from_edge = centre - cx;
                            raw_val = -(dist_from_edge * 4);
                        end
                    end
                    default: raw_val = 0;
                endcase

                // Saturate to signed 8-bit [-128, 127]
                if      (raw_val >  127) rom[addr] = 8'sd127;
                else if (raw_val < -128) rom[addr] = -8'sd128;
                else                    rom[addr] = WEIGHT_BITS'(raw_val);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Synchronous Read (1-cycle latency)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        dout <= rom[cell_addr];
    end

endmodule
