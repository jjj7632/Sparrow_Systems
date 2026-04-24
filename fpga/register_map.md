# Detector AXI-Lite Register Map

This is the register map used by `snick_python/fpga_buffer_manager.py`.

| Offset |        Name        | Direction | Meaning |
-----------------------------------------------------
| `0x00` | `CTRL`             |   Write   | Bit 0 starts detector processing for the submitted frame |
| `0x04` | `STATUS`           |   Read    | Bit 0 busy, bit 1 done, bit 2 result valid, bit 3 base frame valid. |
| `0x08` | `LEFT_FRAME_ADDR`  |   Write   | Low 32 bits of left DDR buffer address. AXI DMA still receives the real source address from PYNQ. |
| `0x0C` | `RIGHT_FRAME_ADDR` |   Write   | Low 32 bits of right DDR buffer address. |
| `0x10` | `FRAME_WIDTH`      |   Write   | Frame width in pixels. Default is `1920`. |
| `0x14` | `FRAME_HEIGHT`     |   Write   | Frame height in pixels. Default is `1080`. |
| `0x18` | `FRAME_CHANS`      |   Write   | Number of channels streamed into the FPGA. The fast path now uses `1` because Python extracts the red channel before DMA. |
| `0x1C` | `FRAME_ID`         |   Write   | Frame number from MATLAB/PC cache. `0xFFFFFFFF` means unknown. |
| `0x20` | `BUFFER_INDEX`     |   Write   | Ping pong buffer index used by Python. |
| `0x24` | `RESULT_X`         |   Read    | float32 left-image candidate x pixel. |
| `0x28` | `RESULT_Y`         |   Read    | float32 left-image candidate y pixel. |
| `0x2C` | `RESULT_Z`         |   Read    | float32 right-image candidate x pixel. Python uses this with the left centroid to compute world-space XYZ. |
| `0x30` | `LEFT_BASE_FRAME_ADDR`  | Write | Low 32 bits of the persistent left base-frame DDR buffer address. The detector AXI master uses this as the left base-image read pointer. |
| `0x34` | `RIGHT_BASE_FRAME_ADDR` | Write | Low 32 bits of the persistent right base-frame DDR buffer address. The detector AXI master uses this as the right base-image read pointer. |
| `0x38` | `BASE_FRAME_VALID`      | Write | `1` after frame `0` has loaded the real base image into FPGA-accessible memory, otherwise `0`. When set, the FPGA reads the base image directly from DDR instead of expecting it as a live DMA stream. |

For the current Python code to work unchanged, the result registers must contain raw IEEE-754 single precision float bit patterns. The FPGA returns image-space centroid values, and `shared_protocol/soc_protocol.py` converts those stereo pixel coordinates into world-space XYZ.

Frame `0` is reserved as the stereo base image. When Python receives frame `0`, it updates the persistent base buffers and does not start the normal current-frame DMA processing path for that frame. Later frames stream only the live left/right images; the custom IP fetches the stored base images from DDR through its AXI master read ports.
