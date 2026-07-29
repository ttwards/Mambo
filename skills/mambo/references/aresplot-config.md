# AresPlot Capture Config

Run only on a trusted debug target. The capture CLI sends `SET_SAMPLE_RATE`, then `START_MONITOR`, and finally a zero-variable `START_MONITOR` to stop; it never implements or sends `SET_VARIABLE`.

## JSON Fields

Required fields are `serial_port`, `baud_rate`, exactly one of `sample_rate_hz` or `sample_period_ms`, `duration_seconds`, `output_csv`, and `variables`. Use `aresplot-capture-schema.json` for the machine-readable schema and `aresplot-capture-example.json` as the starting configuration.

Each variable requires `name`, `address`, and `type`. Addresses accept JSON integers or strings such as `"0x24000100"`. Names and addresses must be unique, and each address must be naturally aligned for its type. Accepted CLI types are `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`, `float32`, and `bool`. The header also names `float64`, but this CLI rejects it because the current firmware emits a placeholder `0.0`, not a float64 observation.

## Choose Addresses From Evidence

Never invent or reuse an address from another build. Build first, preserve `build/zephyr/zephyr.elf`, and resolve each symbol from that exact ELF. The Web Serial Plotter can search the imported ELF. For a command-line check, read `CMAKE_NM:FILEPATH` from the build directory's `CMakeCache.txt`, then run that `arm-zephyr-eabi-nm` executable with `-S -n` on `zephyr.elf` and match the complete symbol name.

Monitor only stable, naturally aligned global or `static` scalar observables. Prefer a dedicated `static volatile` debug mirror updated by the thread that owns the source state; `volatile` keeps the observation in memory but is not a synchronization primitive. Do not monitor a stack local, a heap object that may be freed, MMIO, a pointer value when the intended data is its target, a multiword snapshot, or an actuator command variable. Rebuild and re-resolve every address after code, Kconfig, devicetree, linker, or toolchain changes.

Optional fields: `ack_timeout_seconds` (default 2), `max_frame_bytes` (default 256, range 64..512), and `address_ranges`. An omitted range list uses the DM-MC02 Zephyr memory regions: DTCM `0x20000000..0x2001ffff`, SRAM0 `0x24000000..0x2404ffff`, SRAM1/2 `0x30000000..0x30007fff`, and SRAM4 `0x38000000..0x38003fff`. A supplied range list can only narrow the permitted target addresses. The CLI checks that every typed value fits inside one permitted range.

Start from `aresplot-capture-example.json`. Override config values with `--port`, `--baud`, `--sample-rate-hz`, `--sample-period-ms`, `--duration`, and `--output`. Exactly one rate form is accepted: `sample_period_ms` is converted to a whole `sample_rate_hz` before transmission; the wire payload is always the firmware protocol's `uint32 rate_hz` little-endian value. Pass `--trusted-debug-target` deliberately. Inspect timestamp gaps after every capture and reduce rate or series count when gaps occur.

```sh
python3 scripts/aresplot_capture.py \
  --config references/aresplot-capture-example.json \
  --trusted-debug-target --duration 3

python3 scripts/render_aresplot_csv.py capture.csv \
  --series loop_time_us --start 0 --end 3 --output loop.png
```

`--start` and `--end` are seconds in the renderer's unwrapped MCU timeline. The capture summary counts MCU timestamp gaps larger than two expected periods; treat a nonzero count as a reason to reduce rate or channel count. The renderer reports malformed rows and timestamp wraps/resets instead of silently treating them as normal samples.
