module top(
    input  logic clk,
    input  logic reset,
    output logic [15:0] dbg_address,
    output logic        dbg_hit,
    output logic [31:0] dbg_read_data
);

    logic read_en, write_en;
    logic [15:0] address;
    logic [31:0] write_data;
    logic [31:0] read_data;
    logic hit;

    processor CPU(
        .clk(clk),
        .reset(reset),
        .read_en(read_en),
        .write_en(write_en),
        .address(address),
        .write_data(write_data)
    );

    memory_hierarchy MH(
        .clk(clk),
        .reset(reset),
        .read_en(read_en),
        .write_en(write_en),
        .address(address),
        .write_data(write_data),
        .read_data(read_data),
        .hit(hit)
    );

    assign dbg_address   = address;
    assign dbg_hit       = hit;
    assign dbg_read_data = read_data;

endmodule
