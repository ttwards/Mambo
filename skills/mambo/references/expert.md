# Expert Mode

Use only after an explicit expert-mode request. Keep facts, commands, logs, and unverified assumptions separate. Retain every safety gate.

## Batch Intake

Request or collect: `west topdir`; `west --version`; Python interpreter; exact board target from `west boards`; application path; `prj.conf`; overlays; generated `build/zephyr/zephyr.dts`; generated `.config`; build command and tail of output; probe/serial identity; power and motor state.

## Focused Routes

- Build/config: compare source DTS/Kconfig with generated artifacts; clean/pristine only when configuration cache evidence supports it.
- DM-MC02 UART: console is USART10 in board DTS at 115200. The repository's USART1 overlay uses PA9/PA10 at 921600 with async UART DMA. Do not multiplex USART1 between AresPlot and another peripheral.
- AresPlot: verify `DT_ALIAS(plot)` exists for `CONFIG_PLOTTER`, or use the manual protocol binding route. Verify UART async API, DMA, and non-cache memory requirements for the STM32H7 path before blaming the protocol.
- Runtime: baseline, minimal observables, one interval, one hypothesis, one change. Calculate wire budget before raising rate or variable count.

## Evidence Conflict

Drop back to beginner mode when a claimed board, UART, flash state, power state, or observation conflicts with the checked artifact. State the conflict, request the smallest discriminating check, then resume only after it resolves.
