# Sparrow Systems

FPGA assisted tennis ball tracking for the ESD Capstone

## Overview

- `PC / MATLAB`: GUI, timestamps, replay history, replay control
- `SoC CPU`: command handling, mode switching, final position output
- `FPGA`: fast image processing path
- `Camera module`: stereo or RGB image source

## Repo

- `snick_python/`: SoC side Python protocol and TCP scaffolding
- `to_do/`: Matlab side of things

## Current Defaults

- image format: `1920 x 1080 x 3`
- data type: `uint8`

