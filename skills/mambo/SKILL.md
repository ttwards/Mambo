---
name: mambo
description: "Configure, build, and debug small robots with the Mambo Zephyr embedded framework, especially the Damiao DM-MC02 board and AresPlot. Use for Mambo/Zephyr environment setup, devicetree and Kconfig work, safe board bring-up, small robot applications, UART diagnostics, and trusted AresPlot capture."
---

# Mambo

Guide a user from evidence to one small, working Mambo robot increment at a time. This is the Agentic Embedded contract: inspect the repository and generated artifacts, teach the reason for each step, make the smallest testable change, and preserve a reproducible record of commands and observations. Prefer the DM-MC02 unless the user names another supported board. Keep all actuator power disabled until the user has confirmed the electrical setup and explicitly asks to enable it.

## Select Mode

- Start in beginner mode. Read [beginner.md](references/beginner.md). Use simple Chinese, define a term the first time it matters, state cause and effect, and request or run one objective check before moving on.
- Enter expert mode only when the user explicitly says `专家模式`, `老手模式`, or an unambiguous equivalent. Read [expert.md](references/expert.md). Be concise and batch related diagnostics.
- Return to beginner mode when the user says `新手模式` or equivalent. Also fall back, and say why, when wiring, board identity, build output, flash state, or observed behavior conflicts with an assumption.
- Treat beginner and expert as conversation branches, not Git branches. A mode change alters teaching depth and diagnostic batching; it never weakens evidence or safety requirements.
- Keep the safety gates in [troubleshooting-safety.md](references/troubleshooting-safety.md) in both modes. Never flash, reset through a probe, open a debug probe, or enable motor power unless the user explicitly authorizes it and the board, probe, target, and connection are confirmed.

## Route The Work

1. Establish facts first: repository root, `west` workspace, board target, application, toolchain, and the exact physical connection. Read [environment.md](references/environment.md) for host setup and checks.
2. For DM-MC02, read [dm-mc02-wiring.md](references/dm-mc02-wiring.md) before proposing a cable. Treat its unresolved items as blockers, not guesses. The original manual is [dm-mc02-manual-v1.1.pdf](references/dm-mc02-manual-v1.1.pdf).
3. Make the smallest app change that demonstrates one behavior. Inspect the existing sample, board DTS, overlays, Kconfig, and generated build artifacts before editing. Keep motor outputs and controlled XT30 power off during bring-up unless authorized.
4. Build before diagnosing runtime behavior. Report the exact command and result; distinguish a successful build from a flashed image and from an observed board behavior.
5. Use AresPlot only on a trusted debug target. Read [aresplot.md](references/aresplot.md) and [aresplot-config.md](references/aresplot-config.md) before configuring it.

## AresPlot Loop

Use this fixed loop: baseline build; add minimal observables; build; obtain flash authorization; connect the dedicated serial port; capture or plot; inspect one selected interval; form one hypothesis; change one thing; repeat. Do not use `SET_VARIABLE`; the bundled capture tool cannot and must not send it.

- Use the browser workflow at [Web Serial Plotter](https://captainkaz.github.io/web-serial-plotter/) with the matching ELF only on a trusted debug build.
- Use `scripts/aresplot_capture.py` for a read-only, reproducible CSV capture. Require `--trusted-debug-target` and a reviewed JSON config. Start from [aresplot-capture-example.json](references/aresplot-capture-example.json).
- Use `scripts/render_aresplot_csv.py` to inspect a CSV interval offline. Run both scripts with the same Python environment that supplies their optional dependencies.

## Load References Deliberately

- Read [beginner.md](references/beginner.md) for guided setup and a first program.
- Read [expert.md](references/expert.md) for concise diagnostics and evidence batching.
- Read [environment.md](references/environment.md) for Linux, macOS, Windows, west, Python, and serial permissions.
- Read [dm-mc02-wiring.md](references/dm-mc02-wiring.md) for the board-side UART and SWD facts and their limits.
- Read [aresplot.md](references/aresplot.md) for protocol, devicetree, Kconfig, ELF, sampling, and bandwidth details.
- Read [aresplot-config.md](references/aresplot-config.md) for capture JSON fields, address windows, types, and CLI use.
- Read [troubleshooting-safety.md](references/troubleshooting-safety.md) when a build, serial link, probe, or observed behavior fails.
