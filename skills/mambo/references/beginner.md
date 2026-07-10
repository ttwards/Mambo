# Beginner Workflow

Use this route by default. Say what is being checked, why it matters, and what result would change the next step. Do not claim that a cable, flash, or sensor works without evidence.

## One Step At A Time

1. Ask for the goal in one sentence: for example, "blink the status LED", "read the IMU", or "drive one motor only after a dry-run." Define **board** as the physical controller and **application** as the source directory that becomes its firmware.
2. Identify the board with the user. Recommend DM-MC02 for a new Mambo robot unless a different board is already in use. Confirm its printed label, supply arrangement, and whether motors or controlled XT30 outputs are disconnected.
3. Run the environment checks in `environment.md`. Explain that `west build` creates firmware files; it does not put them on the board.
4. Choose an existing sample or the repository template. Read its `CMakeLists.txt`, `prj.conf`, source, board overlay, and the target board DTS before changing anything. Explain **devicetree** as the hardware map and **Kconfig** as the feature switch list.
5. Implement one visible behavior with all unsafe outputs left off. Use a small state machine or timer, not a busy loop. Add one log line only after its UART route is known.
6. Build the exact application for the exact board. Inspect the build result and generated `zephyr.dts`/`.config` when configuration is in doubt.
7. Ask separately for permission to flash. Before flashing, reconfirm board, runner/probe, target, and power state. A successful flash command is not proof that the program started.
8. Observe one physical or serial result. When it differs from expectation, preserve the result, form one hypothesis, change one thing, and rebuild.

## Explain Early Terms

- **Overlay**: an application-local devicetree patch that selects or configures hardware without changing the board definition.
- **UART**: two signal wires for serial data plus a shared ground. TX means data leaving that device; it normally connects to the other device's RX.
- **ELF**: the build artifact that retains symbol names and addresses. AresPlot's web workflow uses it to map selected variables to addresses.
- **Probe**: a debugger such as CMSIS-DAP, DAPLink, J-Link, or ST-Link. It can reset or program a board, so it is never used implicitly.

## First AresPlot Observation

First prove the normal program builds. Then add one or two non-actuating `static` observables, preserve the ELF, and use USART1 only after the dedicated UART wiring is confirmed. Capture a short baseline before changing control parameters. See `aresplot.md` and `aresplot-config.md`.
