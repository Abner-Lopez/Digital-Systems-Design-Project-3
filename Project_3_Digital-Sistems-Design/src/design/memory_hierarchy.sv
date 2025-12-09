
module memory_hierarchy(
    input  logic clk,
    input  logic reset,
    input  logic read_en,
    input  logic write_en,
    input  logic [15:0] address,
    input  logic [31:0] write_data,
    output logic [31:0] read_data,
    output logic hit
);
    
    // Señales internas
    logic miss, need_wb;

    logic [15:0]  wb_addr;
    logic [255:0] wb_data;

    // Control
    logic miss_reg; 
    logic refill_done;
    
    // Señales de Memoria
    logic mem_write_en_word, mem_write_en_block, mem_read_block_en, mem_read_word_en;
    logic [15:0] mem_byte_addr_word, mem_block_addr_wr, mem_block_addr_rd, mem_byte_addr_rd;
    logic [31:0] mem_write_word, mem_read_word_out;
    logic [255:0] mem_write_block, mem_read_block_out;
    
    // ---------------------------------------------------------
    // 1. Instancia de CACHE 
    // ---------------------------------------------------------
    cache CACHE(
        .clk(clk), .rst(reset),
        .read_en(read_en), .write_en(write_en),
        .address(address), .write_data(write_data),
        .read_data(read_data), .hit(hit), .miss(miss),
        .need_writeback(need_wb), .wb_address(wb_addr),
        .wb_block_data(wb_data), 
        
        .refill_block(mem_read_block_out), // Conexión Directa
        
        .refill_done(refill_done)
    );

    // 2. Instancia de MEMORIA
    main_memory_bram MEM(
        .clk(clk), .reset(reset),
        .write_en_word(mem_write_en_word),
        .byte_addr(mem_byte_addr_word),
        .write_word(mem_write_word),
        .write_en_block(mem_write_en_block),
        .block_addr_wr(mem_block_addr_wr),
        .write_block(mem_write_block),
        .read_block_en(mem_read_block_en),
        .block_addr_rd(mem_block_addr_rd),
        .read_block_out(mem_read_block_out),
        .read_word_en(mem_read_word_en),
        .byte_addr_rd(mem_byte_addr_rd),
        .read_word_out(mem_read_word_out)
    );

    // 3. FSM
    typedef enum logic [1:0] {
        IDLE        = 2'b00,
        WRITEBACK   = 2'b01,
        REFILL_REQ  = 2'b10,
        REFILL_WAIT = 2'b11
    } state_t;

    state_t state, next_state;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) state <= IDLE;
        else state <= next_state;
    end

    // Registro del Miss 
    always_ff @(posedge clk or posedge reset) begin
        if (reset) 
            miss_reg <= 0;
        else 
            if (state == IDLE) 
                miss_reg <= (read_en || write_en) ? miss : 0; 
            else
                miss_reg <= 0;
    end
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (miss_reg) begin
                    if (need_wb) next_state = WRITEBACK;
                    else next_state = REFILL_REQ;
                end
            end
            WRITEBACK:   next_state = REFILL_REQ;
            REFILL_REQ:  next_state = REFILL_WAIT;
            REFILL_WAIT: next_state = IDLE;
            default:     next_state = IDLE;
        endcase
    end

    assign refill_done = (state == REFILL_WAIT) && (next_state == IDLE);
    
    // 4. Salidas 
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_write_en_word  <= 0; mem_byte_addr_word <= 0; mem_write_word <= 0;
            mem_write_en_block <= 0; mem_block_addr_wr  <= 0; mem_write_block <= 0;
            mem_read_block_en  <= 0; mem_block_addr_rd  <= 0;
            mem_read_word_en   <= 0; mem_byte_addr_rd   <= 0;
        end else begin
            // Defaults
            mem_write_en_word  <= 0; mem_write_en_block <= 0;
            mem_read_block_en  <= 0; mem_read_word_en   <= 0;
            
            case (state)
                WRITEBACK: begin
                    mem_write_en_block <= 1;
                    mem_block_addr_wr  <= wb_addr; 
                    mem_write_block    <= wb_data;
                end

                REFILL_REQ: begin
                    mem_block_addr_rd <= {address[15:5], 5'b0};
                    mem_read_block_en <= 1;
                end

                REFILL_WAIT: begin
                    // La caché lo toma directo de mem_read_block_out gracias al wire.
                end

                default: begin end
            endcase
        end
    end

endmodule