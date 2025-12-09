
module cache #(
    parameter NUM_LINES = 32,
    parameter BLOCK_SIZE = 32
)(
    input  logic             clk,
    input  logic             rst,
    input  logic             read_en,
    input  logic             write_en,
    input  logic [15:0]      address,
    input  logic [31:0]      write_data,
    output logic [31:0]      read_data,
    output logic             hit,
    output logic             miss,
    output logic             need_writeback,
    output logic [15:0]      wb_address,
    output logic [255:0]     wb_block_data,
    input  logic [255:0]     refill_block,
    input  logic             refill_done // Puerto de control de la FSM
);

    logic [5:0] tag;
    logic [4:0] index;
    logic [2:0] word_index;

    assign tag        = address[15:10];
    assign index      = address[9:5];
    assign word_index = address[4:2];

    logic             valid_array [0:NUM_LINES-1];
    logic             dirty_array [0:NUM_LINES-1];
    logic [5:0]       tag_array   [0:NUM_LINES-1];
    logic [255:0]     data_array  [0:NUM_LINES-1];

    assign hit  = valid_array[index] && (tag_array[index] == tag);
    assign miss = (read_en || write_en) && !hit;

    always_comb begin
        read_data = data_array[index][word_index*32 +: 32];
    end

    always_ff @(posedge clk or posedge rst) begin
        integer ii;
        if (rst) begin
            // ... (reset logic)
            for (ii = 0; ii < NUM_LINES; ii = ii + 1) begin
                valid_array[ii] <= 0;
                dirty_array[ii] <= 0;
                tag_array[ii]   <= '0;
                data_array[ii]  <= '0;
            end
        end else begin

            // WRITE HIT
            if (hit && write_en) begin
                dirty_array[index] <= 1;
                data_array[index][word_index*32 +: 32] <= write_data;
            end

            // REFILL (Controlado por FSM)
            if (refill_done) begin
                data_array[index]  <= refill_block;
                tag_array[index]   <= tag;
                valid_array[index] <= 1;
                dirty_array[index] <= 0;
            end
        end
    end

    assign need_writeback = miss && valid_array[index] && dirty_array[index];
    assign wb_block_data = data_array[index];
    assign wb_address = {tag_array[index], index, 5'b00000};

endmodule