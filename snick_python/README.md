# snick_python

SoC side Python TCP and command scaffolding

## Files

- `soc_protocol.py`: command IDs, and master/slave mode logic
- `matlab_server_adapter.py`: TCP server on the SoC
- `numpysocket.py`: lightweight socket setup

## Protocol

- `[1]`, `[10]`, `[30]`, `[98]`, `[99]`: command byte only
- `[11,n]`, `[12,n]`, `[15,frame]`: command byte + int32
- `[22,in_out]`: command byte + uint8
- `[21,frame,x,y,z]`: command byte + int32 + 3 float32 values
- `[50,frame,image]`: command byte + int32 + raw image bytes

Default image format:

- `1920 x 1080 x 3`
- `uint8`

## Run

```bash
python matlab_server_adapter.py --host 0.0.0.0 --port 9999
```

## Test

1. Start the server on the Snickerdoodle.
2. Set `BOARD_IP` in `matlab_client_demo.py`.
3. Run:

```bash
python matlab_client_demo.py
```

Note: Make sure to send the pixel arrays, not the raw `.png` file bytes.
