# Sparrow Systems

Hawkeye, a system previously used by the United States Tennis Association [USTA], has been determined to produce errors under windy conditions. We need a system that guarantees accuracy in all conditions to keep the sport's integrity unquestioned. Eagle Eye, our FPGA-based ball tracker, is engineered to solve this challenge.

## Overview

- `PC / MATLAB`: GUI, timestamps, replay history, replay control
- `SoC CPU`: command handling, mode switching, final position output
- `FPGA`: fast image processing path
- `Camera module`: stereo or RGB image source

## Repo

- `snick_python/`: SoC side Python protocol and TCP scaffolding
- `pc_image_cache/`: PC/MATLAB-side replay cache scaffolding
- `shared_protocol/`: shared TCP command/image protocol helpers
- `fpga/`: AXI/DMA integration contract and detector IP stub notes

## Current Defaults

- image format: `1920 x 1080 x 3`
- data type: `uint8`

