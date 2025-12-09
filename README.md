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
- Soporte nativo para **Write-Back** mediante `dirty_array`
- Exposición completa de señales para write-back y refill:
  - `need_writeback`
  - `wb_address`
  - `wb_block_data`
  - `refill_block`
  - `refill_done`
- Lectura combinacional de la palabra solicitada
- Escrituras solo en caso de `write hit`

---



