# Detector AXI-Lite Register Map

This is the register map used by `snick_python/fpga_buffer_manager.py`.

| Offset |        Name        | Direction | Meaning |
-----------------------------------------------------
| `0x00` | `CTRL`             |   Write   | Bit 0 starts detector processing for the submitted frame |
| `0x04` | `STATUS`           |   Read    | Bit 0 should be `1` while busy and `0` when idle. Optional: bit 1 done, bit 2 result valid, bit 3 error. |
| `0x08` | `LEFT_FRAME_ADDR`  |   Write   | Low 32 bits of left DDR buffer address. AXI DMA still receives the real source address from PYNQ. |
| `0x0C` | `RIGHT_FRAME_ADDR` |   Write   | Low 32 bits of right DDR buffer address. |
| `0x10` | `FRAME_WIDTH`      |   Write   | Frame width in pixels. Default is `1920`. |
| `0x14` | `FRAME_HEIGHT`     |   Write   | Frame height in pixels. Default is `1080`. |
| `0x18` | `FRAME_CHANS`      |   Write   | Number of image channels. Default is `3` for RGB. |
| `0x1C` | `FRAME_ID`         |   Write   | Frame number from MATLAB/PC cache. `0xFFFFFFFF` means unknown. |
| `0x20` | `BUFFER_INDEX`     |   Write   | Ping pong buffer index used by Python. |
| `0x24` | `RESULT_X`         |   Read    | float32 x result bits. |
| `0x28` | `RESULT_Y`         |   Read    | float32 y result bits. |
| `0x2C` | `RESULT_Z`         |   Read    | float32 z result bits. |

For the current Python code to work unchanged, the result registers must contain raw IEEE-754 single precision float bit patterns. If the FPGA returns fixed point or integer centroids instead, we will need to update the Python to decode these values instead of using the current `read_reg_f32`.

