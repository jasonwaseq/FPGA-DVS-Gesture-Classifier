module cocotb_iverilog_dump();
initial begin
    $dumpfile("sim_build_input_fifo/input_fifo.fst");
    $dumpvars(0, input_fifo);
end
endmodule
