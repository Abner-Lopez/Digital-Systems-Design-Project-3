// Memoria organizada en bloques de 256 bits (32 bytes).
// 2048 bloques * 32 bytes = 65536 bytes total.

module main_memory_bram(
    input  logic        clk,
    input  logic        reset,

    // write word (synchronous) - para writes de 32-bit dentro de un bloque
    input  logic        write_en_word,
    input  logic [15:0] byte_addr,      
    input  logic [31:0] write_word,

    // write block (synchronous) - writeback de 256-bit
    input  logic        write_en_block,
    input  logic [15:0] block_addr_wr,  
    input  logic [255:0] write_block,

    // read block (synchronous)
    input  logic        read_block_en,
    input  logic [15:0] block_addr_rd,  
    output logic [255:0] read_block_out,

    // read word (synchronous)
    input  logic        read_word_en,
    input  logic [15:0] byte_addr_rd,
    output logic [31:0] read_word_out
);

    // 2048 bloques de 256 bits
    logic [255:0] mem_block [0:2047];

    // registros de salida (síncronos)
    logic [255:0] read_block_reg;
    logic [31:0]  read_word_reg;

    assign read_block_out = read_block_reg;
    assign read_word_out  = read_word_reg;

    // indices
    logic [10:0] block_idx_wr;
    logic [10:0] block_idx_rd;
    logic [10:0] block_idx_word_wr;
    logic [10:0] block_idx_word_rd;
    logic [2:0]  word_idx_wr;
    logic [2:0]  word_idx_rd;

    assign block_idx_wr = block_addr_wr[15:5];
    assign block_idx_rd = block_addr_rd[15:5];
    assign block_idx_word_wr = byte_addr[15:5];
    assign block_idx_word_rd = byte_addr_rd[15:5];

    assign word_idx_wr = byte_addr[4:2];
    assign word_idx_rd = byte_addr_rd[4:2];

    // Synchronous memory operations
    always_ff @(posedge clk) begin
        if (reset) begin
            read_block_reg <= '0;
            read_word_reg  <= '0;
            
        end else begin
            // block write (writeback)
            if (write_en_block) begin
                mem_block[block_idx_wr] <= write_block;
            end

            // word write 
            if (write_en_word) begin
                mem_block[block_idx_word_wr][word_idx_wr*32 +: 32] <= write_word;
            end

            // synchronous block read
            if (read_block_en) begin
                read_block_reg <= mem_block[block_idx_rd];
            end

            // synchronous word read
            if (read_word_en) begin
                read_word_reg <= mem_block[block_idx_word_rd][word_idx_rd*32 +: 32];
            end
        end
    end


    initial begin
        integer blk, w;
        logic [31:0] words [0:7];
        logic [255:0] built_block;

        // Inicializar toda la memoria a cero
        for (blk = 0; blk < 2048; blk = blk + 1) begin
            for (w = 0; w < 8; w = w + 1) begin
                words[w] = 32'h0;
            end
            built_block = {words[7], words[6], words[5], words[4],
                           words[3], words[2], words[1], words[0]};
            mem_block[blk] = built_block;
        end

        // BLOQUE 0: patrón 0x1000_0000 + w
        for (w = 0; w < 8; w = w + 1) words[w] = 32'h1000_0000 + w;
        built_block = {words[7], words[6], words[5], words[4],
                       words[3], words[2], words[1], words[0]};
        mem_block[0] = built_block;

        // BLOQUE 1024: patrón 0x2000_0000 + w
        for (w = 0; w < 8; w = w + 1) words[w] = 32'h2000_0000 + w;
        built_block = {words[7], words[6], words[5], words[4],
                       words[3], words[2], words[1], words[0]};
        mem_block[1024] = built_block;

    end

endmodule
