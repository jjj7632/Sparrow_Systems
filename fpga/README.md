# FPGA AXI/DMA Integration

This folder documents the hardware contract between the SoC Python code and the FPGA fast image processing path.

## Target Data Flow

```text
MATLAB / PC cache
    sends stereo frame over TCP
SoC Python
    receives {left_image, right_image}
    copies both images into PYNQ DDR buffers
    starts left and right AXI DMA transfers
FPGA fabric
    left AXI DMA MM2S stream -> detector left stream input
    right AXI DMA MM2S stream -> detector right stream input
    detector writes x/y/z result registers
SoC Python
    reads x/y/z through AXI-Lite register window
    returns command 21 to MATLAB
```

## Vivado Block Design

Minimum blocks needed:

- `Zynq Processing System`: owns DDR and runs Python/PYNQ.
- `Processor System Reset`: reset synchronization.
- `AXI SmartConnect` or `AXI Interconnect`: connects PS AXI master to control registers.
- `axi_dma_left`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `axi_dma_right`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `stereo_ball_detector`: custom IP with two AXI4-Stream slave inputs and AXI-Lite control/result registers.

The MVP hardware path intentionally requires two DMA IPs. A single shared DMA path is not supported in the SoC Python code because it would force the detector to decode left/right image boundaries from one serialized stream.

Important notes on the connections:

- PS `M_AXI_GP0` -> AXI-Lite control bus for both DMA IPs and the detector registers.
- Both DMA `M_AXI_MM2S` ports -> PS high-performance DDR access port through SmartConnect.
- `axi_dma_left/M_AXIS_MM2S` -> detector left AXI stream input.
- `axi_dma_right/M_AXIS_MM2S` -> detector right AXI stream input.
- Common clock/reset for DMA and detector stream interfaces.

## Address Map Contract

The SoC Python code currently expects the custom detector AXI-Lite register base to default to:

```text
0x43C60000
```

You can change this when running the server:

```bash
sudo python3 matlab_server_adapter.py \
  --host 0.0.0.0 \
  --port 9999 \
  --image-width 1920 \
  --image-height 1080 \
  --image-channels 3 \
  --overlay-bit /home/xilinx/overlays/sparrow/sparrow.bit \
  --left-dma-name axi_dma_left \
  --right-dma-name axi_dma_right \
  --fpga-ip-base 0x43C60000
```

The matching `.hwh` file should live beside the `.bit` file with the same basename so PYNQ can discover the overlay IP names.

## Pixel Format

The current Python path sends one full left image and one full right image as:

```text
shape: 1920 x 1080 x 3
type: uint8
order: row-major RGB
```

If the AXI DMA stream width is 24 bits, each stream beat can represent one RGB pixel. If the stream width is 32 bits, pack RGB into the lower 24 bits and ignore the unused byte in the detector.

