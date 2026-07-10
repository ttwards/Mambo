# AresPlot In Mambo

Use AresPlot only on a trusted debug target. The protocol accepts raw MCU addresses and the firmware dereferences them. Read-only host tooling does not make an untrusted image or wrong address harmless.

## Repository Behavior

- `include/ares/protocol/plotter/aresplot_protocol.h` defines SOP `0xA5`, EOP `0x5A`, command IDs, types, and defaults.
- `lib/ares/protocol/plotter/aresplot_protocol.c` frames as `SOP, CMD, LEN_LE16, payload, XOR checksum, EOP`. The XOR covers CMD, both length bytes, and payload.
- `START_MONITOR` payload is a count byte followed by `address_le32, type_u8` for each variable. A zero count stops monitoring. `SET_SAMPLE_RATE` carries `rate_hz_le32`. ACK payload is `acknowledged_cmd, status`.
- Monitor data is `timestamp_ms_le32` followed by one `float32` per requested variable. The firmware converts supported source types to float32 before sending.
- `CONFIG_PLOTTER=y` creates an automatic UART binding using `DT_ALIAS(plot)`. Define a `plot` alias in the final devicetree or use the manual binding path demonstrated by `plotter_demo`.
- `CONFIG_PLOTTER` selects the protocol, UART interface, and ARES communication library. `CONFIG_ARESPLOT_FREQ` has range 1..1000 and affects the automatic default. `CONFIG_ARESPLOT_MAX_VARS_TO_MONITOR` defaults to 10; `CONFIG_ARESPLOT_SHARED_BUFFER_SIZE` defaults to 256.
- `samples/communication/plotter_demo` shows explicit binding and variable registration. `plotter_auto` shows automatic binding but its board setup must still supply the selected UART and `plot` alias.

## UART And ELF Workflow

Use 921600 baud, 8N1 on the dedicated AresPlot UART when the application/overlay configures that rate. Preserve `build/zephyr/zephyr.elf`; import it into [Web Serial Plotter](https://captainkaz.github.io/web-serial-plotter/) to select symbols by name. Treat ELF addresses as valid only for that exact build and trusted board image. Rebuild, re-import, and review the address list after layout-changing code or configuration changes.

## Types And Limits

The header defines `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `float32`, `float64`, and `bool`. The capture CLI accepts every implemented monitoring type except `float64`: it rejects `float64` with a compatibility error because the current sender has no `FLOAT64` case and emits `0.0` through `default`. `bool` is sent as `0.0` or `1.0`.

For `N` variables, each monitor payload is `4 + 4N` bytes and the full outgoing frame is `10 + 4N` bytes. At 921600 baud, 8N1 carries at most about 92,160 bytes/s before protocol, scheduling, and logging overhead. Approximate required wire bytes/s as `(10 + 4N) * sample_rate_hz`; leave margin and start low. Sampling faster than the variable update rate only repeats values. The firmware's timer drives sends and `send_raw()` calls `uart_tx()` directly without queue/backpressure, so do not claim a lossless 1000 Hz capture. Detect timestamp gaps in the CSV and lower rate or channel count if the UART cannot keep up. Rates above 1000 collapse to a 1 ms period even though the host command accepts a 32-bit rate.

## Debug Loop

Build a baseline. Add one or two non-actuating observables. Build again. Obtain flash authorization. Connect the dedicated serial link. Capture/plot a short interval. Inspect one interval, state one hypothesis, change one thing, and repeat. Never use plotter control features to write a variable during this workflow.
