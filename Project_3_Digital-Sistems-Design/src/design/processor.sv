module processor(
    input  logic clk,
    input  logic reset,
    output logic read_en,
    output logic write_en,
    output logic [15:0] address,
    output logic [31:0] write_data
);

    logic [31:0] cycle;  // Contador de ciclos

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            cycle <= 0;
        else
            cycle <= cycle + 1;
    end

    always_ff @(posedge clk or posedge reset) begin
        if(reset) begin
            read_en    <= 0;
            write_en   <= 0;
            address    <= 0;
            write_data <= 32'h0;
        end else begin
            
            read_en  <= 0;
            write_en <= 0;
            
            case (cycle)
                2: begin  
                    read_en  <= 1;  
                    address  <= 16'h0004;  
                end
                4: begin  
                    read_en  <= 1;  
                    address  <= 16'h0008;  
                end
                6: begin  
                    write_en   <= 1;  
                    address    <= 16'h0004;  
                    write_data <= 32'hDEADBEEF;  
                end
                8: begin  
                    read_en  <= 1;  
                    address  <= 16'h0004;  
                end
                10: begin  
                    read_en  <= 1;  
                    address  <= 16'h8004;  
                end
                default: begin
                    // read_en y write_en ya son 0.
                end
            endcase
        end
    end

endmodule