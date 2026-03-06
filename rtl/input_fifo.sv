`timescale 1ns/1ps

// Synchronous ring-buffer FIFO for EVT2.0 32-bit words.

module input_fifo #(
    parameter [31:0] width_p = 32,
    parameter [31:0] depth_log2_p = 8
)(  input  [0:0]         clk_i,
    input  [0:0]         reset_i,
    input  [width_p-1:0] data_i,
    input  [0:0]         ready_i,
    input  [0:0]         valid_i,
    output [0:0]         ready_o, 
    output [0:0]         valid_o,
    output [width_p-1:0] data_o 
);

    logic [depth_log2_p:0] wr_ptr, rd_ptr;  
    logic wr_en;
    assign wr_en = valid_i & ready_o;
    logic rd_en;
    assign rd_en = valid_o & ready_i;   

    logic [0:0] full;
    assign full = (wr_ptr[depth_log2_p] ^ rd_ptr[depth_log2_p]) 
                  && (wr_ptr[depth_log2_p-1:0] == rd_ptr[depth_log2_p-1:0]);  

    logic [0:0] empty;
    assign empty = ~(wr_ptr[depth_log2_p] ^ rd_ptr[depth_log2_p]) 
                   && (wr_ptr[depth_log2_p-1:0] == rd_ptr[depth_log2_p-1:0]);   

    assign ready_o = ~full;
    assign valid_o = ~empty;    

    logic [width_p-1:0] rd_data_l;
    logic [depth_log2_p:0] mux;
    assign mux = (rd_en) ? (rd_ptr + 1) : rd_ptr;

    ram_1r1w_sync #(
        .width_p(width_p),
        .depth_p(1<<depth_log2_p),
        .filename_p("")
    ) ram_inst (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .wr_valid_i(wr_en),
        .wr_data_i(data_i),
        .wr_addr_i(wr_ptr[depth_log2_p-1:0]),
        .rd_valid_i(1'b1),
        .rd_addr_i(mux[depth_log2_p-1:0]),
        .rd_data_o(rd_data_l)
    );  

    logic [0:0] trail;
    logic [width_p-1:0] data_l;
    logic [0:0] firstwrite;

    always_ff @(posedge clk_i) begin
        if (reset_i) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            data_l <= '0;
            trail <= '0;
            firstwrite <= '0;
        end
        else begin
            if (wr_en) begin
                wr_ptr <= wr_ptr + 1;
                data_l <= data_i;
            end
            if (rd_en) begin
                rd_ptr <= rd_ptr + 1;
            end
            firstwrite <= (wr_en & empty) | (firstwrite & rd_en);
            trail <= (mux == wr_ptr) & rd_en;
        end
    end 
    
    assign data_o = (firstwrite | trail) ? data_l : rd_data_l;

endmodule
