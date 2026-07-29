# Troubleshooting And Safety

## Required Gates

- Confirm the board, application, build directory, probe, serial adapter, and target before any action that can alter hardware state.
- Obtain explicit user authorization before `west flash`, reset, probe attach, OpenOCD, pyOCD, J-Link, STM32CubeProgrammer, or motor/XT30 power enablement.
- Keep a debug console and AresPlot on separate UART peripherals when possible. On DM-MC02 use USART10 for the console topology and USART1 for the dedicated plotter topology only after wiring verification.
- Do not claim hardware validation unless it was actually observed on the named hardware.

## Diagnose In Order

1. Build failure: capture the exact command, first diagnostic, target, overlay list, and generated config.
2. Flash uncertainty: check authorization, runner/probe identity, board power, and target. A completed command proves only its reported result.
3. Silent console: verify selected UART, baud, TX/RX crossing, GND, voltage level, and that no other process owns the host port.
4. Silent AresPlot: verify trusted ELF/build match, USART1 ownership, 921600 setting, `plot` alias or explicit binding, async UART/DMA requirements, and frame/baud budget.
5. Bad plot: first reject a timestamp reset/wrap, timestamp gaps from UART overload, noisy serial data, sample-rate mismatch, and observables updated slower than capture. Lower rate/channel count before assuming a control bug. Then test one source-level hypothesis.

## Stop Conditions

Stop and request evidence when the board or probe identity is unknown, voltage/pinout is undocumented, a physical connection conflicts with DTS/manual facts, a serial port cannot be attributed to the intended device, or motor power would be required to continue. Do not bridge the gap with a guess.
