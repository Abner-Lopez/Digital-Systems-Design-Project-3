# Digital-Systems-Design-Project-3
Este repositorio contiene el tercer proyecto del curso Diseño de Sistemas Digitales, el cual consiste en la implementación de un sistema de jerarquía de memoria. El proyecto aborda la construcción y simulación de la interacción entre una memoria principal y una memoria caché, con el objetivo de optimizar el acceso a los datos y mejorar el rendimiento del sistema.

## Módulos

## Módulo cache.sv
Este módulo implementa la memoria **caché de mapeo directo** del sistema. Gestiona accesos de lectura y escritura a palabras de 32 bits, mantiene los bits `valid` y `dirty`, detecta condiciones de `hit` y `miss`, y proporciona toda la información necesaria para que una FSM externa maneje las operaciones de reemplazo, write-back y refill desde la memoria principal.

El diseño es parametrizable, modular y adecuado para una jerarquía de memoria con bloques de 32 bytes.
---
- Mapeo directo (1 línea por índice)
- Tamaño por defecto:
  - `NUM_LINES = 32`
  - `BLOCK_SIZE = 32 B` (256 bits)
- Detección de hit/miss
- Soporte para **Write-Back** mediante `dirty_array`
- Exposición completa de señales para write-back y refill:
  - `need_writeback`
  - `wb_address`
  - `wb_block_data`
  - `refill_block`
  - `refill_done`
- Lectura combinacional de la palabra solicitada
- Escrituras solo en caso de `write hit`

---
Con bloques de 32 bytes, cada línea contiene 8 palabras de 32 bits.

---

### Tabla de Señales

| Señal | Dir | Bits | Descripción |
|-------|-----|------|-------------|
| clk | in | 1 | Reloj |
| rst | in | 1 | Reset de la caché |
| read_en | in | 1 | Solicitud de lectura |
| write_en | in | 1 | Solicitud de escritura |
| address | in | 16 | Dirección por byte |
| write_data | in | 32 | Datos para escritura |
| read_data | out | 32 | Palabra leída desde el bloque |
| hit | out | 1 | Indica coincidencia válida línea-tag |
| miss | out | 1 | Indica acceso incorrecto (read/write sin hit) |
| need_writeback | out | 1 | Indica que debe escribirse el bloque sucio |
| wb_address | out | 16 | Dirección base del bloque a escribir |
| wb_block_data | out | 256 | Bloque completo para write-back |
| refill_block | in | 256 | Bloque traído de la memoria principal |
| refill_done | in | 1 | Indica que el refill debe aplicarse |

---
#### Lectura
Salida combinacional basada en la palabra seleccionada por `word_index`.

#### Escritura (solo si hay hit)
- Actualiza la palabra correspondiente del bloque
- Marca la línea como `dirty`

#### Miss & Write-Back
Si hay miss:
- `need_writeback = 1` si la línea era válida y dirty
- Se exponen dirección y datos del bloque a la FSM

#### Refill
Cuando `refill_done = 1`:
- Se copia el bloque `refill_block` a la línea correspondiente
- Se actualiza `tag`, `valid = 1` y `dirty = 0`

---

```systemverilog

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

