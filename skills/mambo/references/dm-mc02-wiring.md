# DM-MC02 Wiring Facts

Use the DM-MC02 for new Mambo work unless another board is already selected. This reference separates board-side facts from probe-side assumptions.

## Verified Board-Side Facts

| Function | Board connector pin / MCU pin | Evidence |
| --- | --- | --- |
| USART1 GND | pin 1 / GND | Manual PDF p. 8, printed manual p. 6 |
| USART1 TX | pin 2 / `USART1_TX`, PA9 | Manual PDF p. 8, printed manual p. 6; repository USART1 overlay |
| USART1 RX | pin 3 / `USART1_RX`, PA10 | Manual PDF p. 8, printed manual p. 6; repository USART1 overlay |
| UART10 GND | pin 1 / GND | Manual PDF p. 9, printed manual p. 7 |
| UART10 TX | pin 2 / `UART10_TX`, PE3 | Manual PDF p. 9, printed manual p. 7; board DTS |
| UART10 RX | pin 3 / `UART10_RX`, PE2 | Manual PDF p. 9, printed manual p. 7; board DTS |
| SWD VCC | pin 1 / 3.3 V target reference | Manual PDF p. 9, printed manual p. 7 |
| SWD GND | pin 2 / GND | Manual PDF p. 9, printed manual p. 7 |
| SWCLK | pin 3 / PA14 | Manual PDF p. 9, printed manual p. 7 |
| SWDIO | pin 4 / PA13 | Manual PDF p. 9, printed manual p. 7 |

`boards/damiao/dm_mc02/dm_mc02.dts` selects USART10 as the console and maps RX PE2, TX PE3 at 115200. `samples/communication/ares_communication/boards/dm_mc02_usart1_921600.overlay` enables USART1 at PA9/PA10, 921600 baud, with DMA.

## Intended Debug Topology, Pending Probe Confirmation

Use a CMSIS-DAP/DAPLink-style probe that provides both SWD and a serial bridge. The desired separation is SWD for authorized programming/debug and its serial bridge on USART10 for the 115200 console. Before connecting, inspect the specific probe manual/label and confirm every probe-side pin, voltage tolerance, and whether VTref is a sense input.

Only after that confirmation, map signal names rather than connector positions: probe SWDIO to board SWDIO, probe SWCLK to board SWCLK, probe GND to board GND, probe UART TX to board UART10 RX, and probe UART RX to board UART10 TX. Treat SWD VCC as VTref/sense by default, not permission to power the target from the probe. The UART10 schematic also depicts a 5 V supply pin: normally connect only common GND and crossed TX/RX, and never tie a probe UART power output to that header unless the complete power scheme is intentionally verified.

The provided board manual does **not** establish that an arbitrary CMSIS-DAP/DAPLink has a CDC/UART bridge, identify its serial bridge pins, define its VTref direction, or provide a board-to-probe cable. Those facts remain unresolved until the exact probe model's documentation and host device enumeration are checked.

## Dedicated AresPlot UART

Use a Damiao debugging tool or USB-to-UART adapter on USART1 as the dedicated 921600 AresPlot link only after confirming the exact adapter model, its 3.3 V TTL compatibility, pinout, and host device enumeration. Cross TX/RX by signal name and share GND. The board manual confirms USART1's board-side pins, but does not document the debugging tool/adapter connector or voltage; treat those as unresolved.

USART1 is one hardware peripheral. Do not assign it to AresPlot and another UART peripheral simultaneously. Disable or move the competing node first, then verify the final generated `zephyr.dts`.

## Manual Limits

The original manual is bundled as `dm-mc02-manual-v1.1.pdf`. Its PDF p. 17 (printed p. 15) contains the board connector map for physical location. It documents the board and its connectors, not the host probe. Never infer a connector's mechanical order, VTref behavior, power direction, or adapter logic level from the MCU pin name alone.
