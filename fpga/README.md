# FPGA AXI/DMA Integration

This folder documents the hardware contract between the SoC Python code and the FPGA fast image processing path.

## Target Data Flow

```text
MATLAB / PC cache
    sends stereo frame over TCP
SoC Python
    receives {left_image, right_image}
    extracts the red channel and copies those images into PYNQ DDR buffers
    keeps frame 0 red-channel data as the persistent stereo base image in PYNQ DDR buffers
    starts current left/right and base left/right AXI DMA transfers
FPGA fabric
    left AXI DMA MM2S stream -> detector left stream input
    right AXI DMA MM2S stream -> detector right stream input
    left base AXI DMA MM2S stream -> detector left base stream input
    right base AXI DMA MM2S stream -> detector right base stream input
    detector does red-channel subtract, threshold, candidate extraction
    detector writes stereo pixel centroid result registers
SoC Python
    reads stereo pixel centroids through AXI-Lite register window
    converts stereo centroids into x/y/z
    returns command 21 to MATLAB
```

## Vivado Block Design

Minimum blocks needed:

- `Zynq Processing System`: owns DDR and runs Python/PYNQ.
- `Processor System Reset`: reset synchronization.
- `AXI SmartConnect` or `AXI Interconnect`: connects PS AXI master to control registers.
- `axi_dma_left`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `axi_dma_right`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `axi_dma_left_base`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `axi_dma_right_base`: AXI DMA configured for MM2S only, simple DMA mode, scatter gather disabled.
- `stereo_ball_detector`: custom IP with four AXI4-Stream slave inputs and AXI-Lite control/result registers.

The MVP hardware path intentionally requires four DMA IPs. A single shared DMA path is not supported in the SoC Python code because it would force the detector to decode current/base and left/right image boundaries from one serialized stream.

Important notes on the connections:

- PS `M_AXI_GP0` -> AXI-Lite control bus for both DMA IPs and the detector registers.
- All DMA `M_AXI_MM2S` ports -> PS high-performance DDR access port through SmartConnect.
- `axi_dma_left/M_AXIS_MM2S` -> detector left AXI stream input.
- `axi_dma_right/M_AXIS_MM2S` -> detector right AXI stream input.
- `axi_dma_left_base/M_AXIS_MM2S` -> detector left base AXI stream input.
- `axi_dma_right_base/M_AXIS_MM2S` -> detector right base AXI stream input.
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
  --left-base-dma-name axi_dma_left_base \
  --right-base-dma-name axi_dma_right_base \
  --fpga-ip-base 0x43C60000
```

The matching `.hwh` file should live beside the `.bit` file with the same basename so PYNQ can discover the overlay IP names.

Frame `0` is reserved for base-image calibration. When frame `0` arrives, Python updates persistent FPGA-accessible left/right base buffers and writes their addresses into the detector AXI-Lite registers. Normal nonzero frames stream both current buffers and base buffers into the detector for hardware subtraction.

## Pixel Format

The TCP/MATLAB side still sends one full left image and one full right image as:

```text
shape: 1920 x 1080 x 3
type: uint8
order: row-major RGB
```

Before DMA, Python extracts the red channel to match the OpenCV reference algorithm:

```text
shape: 1920 x 1080
type: uint8
order: row-major red channel only
```

The custom detector consumes the red current/base streams, performs saturating `raw - base`, applies threshold `25`, ignores the top 100 rows, expands threshold hits by a 5-pixel radius to cover the 10x10 close-style gap bridging, and returns left x, left y, and right x image-space centroid values. Python then computes world-space XYZ from those stereo centroids.
