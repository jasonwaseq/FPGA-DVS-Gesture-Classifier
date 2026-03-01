module cocotb_iverilog_dump();
initial begin
    $dumpfile("sim_build_core/voxel_bin_core.fst");
    $dumpvars(0, voxel_bin_core);
end
endmodule
