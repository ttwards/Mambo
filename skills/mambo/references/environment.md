# Host And Workspace Checks

Run checks before installing or building. Use the active Python environment for `west`; do not mix a system `west` with a different virtual environment by accident.

## Choose The Workspace Path

First run `west topdir`. If it succeeds, preserve that workspace and inspect it with `west list zephyr`; do not initialize over it. If a Mambo checkout exists but no west workspace does, initialize from the local manifest after confirming the intended parent directory:

```sh
cd <workspace-parent>
west init -l mambo
west update
west zephyr-export
python3 -m pip install -r zephyr/scripts/requirements.txt
```

For a new remote checkout, follow the repository `README.md`. The current documented sequence is below; `west update` and SDK installation download substantial content, so explain that and obtain approval before running them:

```sh
west init -m https://github.com/ttwards/mambo --mr master my-workspace
cd my-workspace
west update
west zephyr-export
python3 -m pip install -r zephyr/scripts/requirements.txt
west sdk install -t arm-zephyr-eabi
```

Run these workspace commands only after creating and activating the host-specific virtual environment below. On Windows, use `py -m pip` where the examples use `python3 -m pip`. Install the capture dependencies in that same environment when AresPlot CSV tools are needed:

```sh
python3 -m pip install pyserial matplotlib
```

## Linux

Install the repository-documented build prerequisites, then create an isolated Python environment:

```sh
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install --upgrade pip wheel
python3 -m pip install west
west --version
west topdir
west list zephyr
```

Check the board identifier instead of assuming it:

```sh
west boards | rg 'dm_mc02|DM_MC02'
west build -b dm_mc02 samples/motor/dm_demo --pristine
```

Only run the build after checking that the sample and target exist. The target name may vary with the installed Zephyr version.

For a serial device, inspect ownership first:

```sh
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
id -nG
udevadm info -q property -n /dev/ttyACM0
```

If the device group is `dialout`, add the current user only after explaining that a new login is needed: `sudo usermod -aG dialout "$USER"`. Do not use `chmod 666`.

Generate a udev rule only after detecting the specific device VID/PID with `udevadm`. Use a narrow template, substitute the observed lowercase hexadecimal values, and ask before writing under `/etc/udev/rules.d/`:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="vvvv", ATTRS{idProduct}=="pppp", GROUP="dialout", MODE="0660", TAG+="uaccess"
```

Reload and reconnect only after approval: `sudo udevadm control --reload-rules && sudo udevadm trigger`. Do not create a wildcard rule or a `MODE="0666"` rule.

## macOS

Use `/dev/cu.*` for an outbound serial connection and inspect devices after reconnecting:

```sh
ls /dev/cu.*
python3 --version
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install --upgrade pip wheel
python3 -m pip install west
west --version
west topdir
```

macOS does not use Linux `dialout` or udev rules. Resolve serial access through the device driver, user session, and the selected `/dev/cu.*` node.

## Windows

Use PowerShell and a Python virtual environment. Confirm the COM port in Device Manager and the board/probe driver before opening it:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install --upgrade pip wheel
py -m pip install west
west --version
west topdir
```

Use `COM3`-style port names in the capture config. Do not use Linux group or udev commands on Windows.

## Build Facts

Run `west build` from the Mambo repository or pass explicit paths. Inspect `build/zephyr/zephyr.dts`, `build/zephyr/.config`, and `build/zephyr/zephyr.elf` after a successful build. `west flash` is a separate action requiring explicit approval and confirmed hardware.
