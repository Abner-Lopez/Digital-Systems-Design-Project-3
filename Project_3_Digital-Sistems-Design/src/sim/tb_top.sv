`timescale 1ns/1ps

module tb_top;

    // Clock and reset
    logic clk;
    logic reset;

    // Debug outputs from top
    logic [15:0] dbg_address;
    logic        dbg_hit;
    logic [31:0] dbg_read_data;

    // Cycle counter and helpers
    integer cycle_count = 0;
    integer i, j;
    logic [15:0] last_addr;
    logic        last_hit;

    // Helper signals 
    logic [5:0] tag_now;
    logic [4:0] idx_now;
    logic [4:0] off_now;

    // DUT
    top DUT (
        .clk(clk),
        .reset(reset),
        .dbg_address(dbg_address),
        .dbg_hit(dbg_hit),
        .dbg_read_data(dbg_read_data)
    );

    // Clock generator (100 MHz -> 10 ns period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Waveform dump
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top); 
    end

    // Small debug preload check 
    initial begin
        #1;
        $display("\n[TB] Quick preload check (mem_block[0] and mem_block[1024])");
        $display("    mem_block[0]  word1 = 0x%h", DUT.MH.MEM.mem_block[0][63 -: 32]);
        $display("    mem_block[1024] word1 = 0x%h\n", DUT.MH.MEM.mem_block[1024][63 -: 32]);
    end

    // ---------------------------
    // Task: print mapping table for your config
    // (offset=5, index=5, tag=6)
    // ---------------------------
    task automatic print_mapping_table();
        logic [15:0] addr_list [0:6];
        integer k;
        logic [5:0]  tag;
        logic [4:0]  index;
        logic [4:0]  offset;
        logic [15:0] a16;

        // Direcciones de ejemplo (16-bit)
        addr_list[0] = 16'h0004;  // usado en processor
        addr_list[1] = 16'h0008;  // usado en processor
        addr_list[2] = 16'h8004;  // usado en processor (otro tag)
        addr_list[3] = 16'h2118;  // ejemplo
        addr_list[4] = 16'h211C;  // ejemplo
        addr_list[5] = 16'h403C;  // ejemplo
        addr_list[6] = 16'h7F00;  // ejemplo 

        $display("\n--- MAPPING TABLE (cache config: block=32B, lines=32) ---");
        $display("+----------------------+-----------+-----------+-----------+---------+");
        $display("| Address (hex)        | Tag (hex) | Index(hex)| Offset(hex)| Comment |");
        $display("+----------------------+-----------+-----------+-----------+---------+");

        for (k = 0; k <= 6; k = k + 1) begin
            a16 = addr_list[k][15:0];
            tag    = a16[15:10];
            index  = a16[9:5];
            offset = a16[4:0];

            $display("| 0x%04h               | 0x%02h     | 0x%02h      | 0x%02h      |         |",
                     a16, tag, index, offset);
        end

        $display("+----------------------+-----------+-----------+-----------+---------+");
        $display("| Note: Hit/Miss depends on cache state at the time of access.      |");
        $display("+-------------------------------------------------------------------+\n");
    endtask

    // ---------------------------
    // Task: print cache inspector 
    // Also prints writeback block on demand
    // ---------------------------
    task automatic print_cache_inspector();
        integer line;
        integer word;
        logic val, dirty;
        logic [5:0] tag_field;
        logic [31:0] wdata;
        integer valid_count;
        $display("\n--- CACHE INSPECTOR (only valid lines) ---");
        $display("MH.state = %0d (enum numeric), mem_read_block_en=%0b, mem_write_en_block=%0b",
                  DUT.MH.state, DUT.MH.mem_read_block_en, DUT.MH.mem_write_en_block);

        // Count valid lines to give context
        valid_count = 0;
        for (line = 0; line < 32; line = line + 1) begin
            if (DUT.MH.CACHE.valid_array[line]) valid_count = valid_count + 1;
        end
        $display("Valid lines: %0d / 32", valid_count);

        for (line = 0; line < 32; line = line + 1) begin
            val = DUT.MH.CACHE.valid_array[line];
            if (!val) continue; // skip non-valid lines to reduce noise
            dirty = DUT.MH.CACHE.dirty_array[line];
            tag_field = DUT.MH.CACHE.tag_array[line];
            $write("LINE %0d | V=%0b D=%0b TAG=0x%02h : ", line, val, dirty, tag_field);
            // print 8 words
            for (word = 0; word < 8; word = word + 1) begin
                wdata = DUT.MH.CACHE.data_array[line][(word+1)*32-1 -: 32];
                $write(" %08h", wdata);
            end
            $write("\n");
        end

        // If there is a pending writeback this cycle, print the block being written to MEM
        if (DUT.MH.mem_write_en_block) begin
            $display("\n--- WRITEBACK BLOCK (to mem_block_idx=%0d addr=0x%04h) ---",
                     DUT.MH.mem_block_addr_wr[15:5], DUT.MH.mem_block_addr_wr);
            // print the 8 words from mem_write_block
            for (word = 0; word < 8; word = word + 1) begin
                wdata = DUT.MH.mem_write_block[(word+1)*32-1 -: 32];
                $display("  wb_word %0d = 0x%08h", word, wdata);
            end
            $display("--- END WRITEBACK BLOCK ---\n");
        end

        $display("--- END CACHE INSPECTOR ---\n");
    endtask

    // ---------------------------
    // Monitor: prints key events, prints cache inspector on demand
    // ---------------------------
    always @(posedge clk) begin
        if (reset) begin
            cycle_count <= 0;
            last_addr <= 16'hffff;
            last_hit  <= 1'bx;
        end else begin
            cycle_count <= cycle_count + 1;

            // Key-cycle messages (same sequence used by processor)
            case (cycle_count)
                2:  $display("C%0d: READ 0x0004 -> expect: MISS (refill)", cycle_count);
                4:  $display("C%0d: READ 0x0008 -> expect: HIT", cycle_count);
                6:  $display("C%0d: WRITE 0x0004 -> expect: HIT (write DEADBEEF)", cycle_count);
                8: begin
                    $display("C%0d: READ 0x0004 -> verify DEADBEEF", cycle_count);
                    if (dbg_read_data === 32'hDEADBEEF) begin
                        $display("   >>> OK: read 0xDEADBEEF");
                    end else begin
                        $error("   >>> FAIL: read 0x%h (expected 0xDEADBEEF)", dbg_read_data);
                    end
                    // print cache inspector after the write/read sequence to observe the written line
                    print_cache_inspector();
                end
                10: $display("C%0d: READ 0x8004 -> expect: MISS + dirty eviction", cycle_count);
                11: begin
                    // print cache inspector just before eviction/refill
                    if (DUT.MH.mem_read_block_en || DUT.MH.mem_write_en_block) begin
                        $display("[C%0d] BEFORE eviction/refill, cache state:", cycle_count);
                        print_cache_inspector();
                    end
                end
                15: begin
                    $display("C%0d: READ 0x8004 (post-refill) -> expect: HIT", cycle_count);
                    // after refill, inspect cache to see updated line
                    print_cache_inspector();
                end
                default: ;
            endcase

            // Print when address or hit changes (reduces log noise)
            if ((dbg_address != last_addr) || (dbg_hit != last_hit)) begin
                // compute tag/index/offset for this dbg_address
                tag_now = dbg_address[15:10];
                idx_now = dbg_address[9:5];
                off_now = dbg_address[4:0];

                $display("[%0t] C%0d: ADDR=0x%04h  TAG=0x%02h IDX=0x%02h OFF=0x%02h  HIT=%0b  DATA=0x%h",
                         $time, cycle_count, dbg_address, tag_now, idx_now, off_now, dbg_hit, dbg_read_data);

                last_addr = dbg_address;
                last_hit  = dbg_hit;
            end

            // Observe when MH asks memory for a block read
            if (DUT.MH.mem_read_block_en) begin
                $display("[%0t] TB: MH requested block read index=%0d (addr 0x%04h)",
                         $time, DUT.MH.mem_block_addr_rd[15:5],
                         DUT.MH.mem_block_addr_rd);
            end

            // Observe when MEM executes a block write (writeback)
            if (DUT.MH.mem_write_en_block) begin
                $display("[%0t] TB: MEM write_block asserted -> block_idx_wr=%0d (addr 0x%04h)",
                         $time, DUT.MH.mem_block_addr_wr[15:5], DUT.MH.mem_block_addr_wr);
                // now print the actual wb block as well (redundant with inspector but handy)
                for (j = 0; j < 8; j = j + 1) begin
                    $display("      wb_word %0d = 0x%08h", j, DUT.MH.mem_write_block[(j+1)*32-1 -: 32]);
                end
            end
        end
    end

    // ---------------------------
    // Reset / run control (calls mapping table)
    // ---------------------------
    initial begin
        reset = 1;
        repeat (3) @(posedge clk);
        reset = 0;

        // Give one cycle to let design settle, then print mapping table
        @(posedge clk);
        print_mapping_table();

        $display("\n--- SIMULATION START ---\n");

        // Run enough cycles to exercise the sequence in processor
        repeat (160) @(posedge clk);

        $display("\n--- SIMULATION END ---\n");
        $finish;
    end

endmodule
