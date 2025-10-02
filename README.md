# CAN OTA Firmware for RP2040

This project implements a CAN-enabled firmware for an RP2040-based controller using the Raspberry Pi Pico SDK together with a companion Python host utility to perform over-the-air (OTA) firmware updates via an MCP2515 CAN transceiver. The runtime keeps the device responsive while new images are streamed over CAN, verifies integrity with CRC16, and automatically reboots into the freshly written application once the transfer succeeds.

## Repository Layout

- `CMakeLists.txt`, `pico_sdk_import.cmake` – build system for the Pico SDK application.
- `src/` – command-handler implementation, MCP2515 driver, WS2812 PIO program, and application entry point.
- `include/` – public headers for the firmware modules.
- `can_ota_host.py` – Python 3 script that fragments a compiled `.bin` file into CAN frames, manages retries, and tracks device status codes.
- `example_can_ota.ino` – legacy Arduino sketch kept as a reference for the original implementation.

## Firmware Highlights

- Targets RP2040 boards using the Raspberry Pi Pico SDK, together with an external MCP2515 CAN controller.
- Implements a command pattern dispatcher for OTA opcodes (`Begin`, `Data`, `End`, `Abort`) carried on CAN ID `0x381` and returns status on CAN ID `0x382`.
- Buffers incoming firmware in RAM via a `FlashWriter` abstraction so production images can be handed off to a bootloader or flash routine.
- Validates each transfer with CRC16-CCITT and aborts safely on mismatch or timeout.
- Drives a WS2812 status LED through PIO with a default 500 ms white blink to indicate the application loop is alive.

## Hardware Requirements

- RP2040 board (e.g., Raspberry Pi Pico, RP2040-Zero or compatible) wired for use with the Pico SDK build outputs.
- MCP2515 CAN controller module wired to RP2040 SPI pins with CS on GPIO5, SCK on GPIO2, MOSI on GPIO4, MISO on GPIO3.
- One WS2812/NeoPixel LED connected to GPIO16 (pin configurable in `src/main.cpp`).
- CAN transceiver and a 500 kbps CAN bus shared with the host tool.
- Stable 3.3 V power supply; ensure the MCP2515 module level shifts correctly for 3.3 V logic.

## Building and Flashing the Firmware

1. Install the Raspberry Pi Pico SDK and export `PICO_SDK_PATH`:

   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git --branch master --depth 1
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

   Add the export to your shell profile for convenience.

2. Configure and build the firmware with CMake:

   ```bash
   mkdir -p build
   cd build
   cmake ..
   cmake --build .
   ```

   The build generates UF2, ELF, and BIN artifacts under `build/`.

3. Flash the device by copying the produced UF2 to the RP2040 boot ROM drive or by using `openocd`/`picotool` from the build tree:

   ```bash
   picotool load can_ota.uf2 --family rp2040
   ```

   Once flashed, the device boots into the CAN OTA runtime and is ready to accept OTA transfers.

4. Generate OTA payloads by using the `can_ota.bin` output from the build directory (or another application binary produced with the same OTA interface) and feed it to the host tool described below.

## CAN OTA Protocol Summary

- **CAN IDs**
  - `0x381` – Host-to-device command frames.
  - `0x382` – Device-to-host status frames (8-byte payload).
- **Command opcodes**
  - `0x01 (BEGIN)` – Payload: firmware size (4 bytes, little-endian) + CRC16 (2 bytes). Initializes a new session.
  - `0x02 (DATA)` – Payload: chunk length (1 byte) + up to 6 data bytes. Chunks may not exceed the declared firmware size.
  - `0x03 (END)` – No payload. Triggers CRC validation and flash commit.
  - `0x04 (ABORT)` – No payload. Cancels the current session and rolls back.
- **Status codes**
  - `0x10` begin OK, `0x11` data OK, `0x12` complete.
  - `0x90` begin error, `0x91` data error, `0x92` verify fail, `0x93` aborted.
  - Status frames also echo received and expected byte counts to aid host-side monitoring.

## Host Tool Usage (`can_ota_host.py`)

### Prerequisites

- Python 3.8 or newer.
- `python-can` installed:

  ```bash
  pip install python-can
  ```

- Working CAN interface configured on the host (e.g., SocketCAN `can0`, USB-CAN adapter, PCAN).

### Command-line Options

```bash
python can_ota_host.py firmware.bin \
  --bustype socketcan \
  --channel can0 \
  --bitrate 500000 \
  --timeout 1.0 \
  --retries 5 \
  --log-level INFO
```

- `firmware.bin` – path to the compiled binary produced by the Pico SDK build (not UF2).
- `--bustype` – backend for `python-can` (socketcan, slcan, pcantype, etc.).
- `--channel` – CAN interface name.
- `--bitrate` – CAN bus speed (must match the device).
- `--timeout` – seconds to wait for each status frame before retrying.
- `--retries` – maximum number of retries per chunk before giving up.
- `--log-level` – adjust verbosity (`DEBUG` reveals per-chunk progress).

The tool automatically sends `BEGIN`, streams the firmware in 6-byte payloads, waits for positive acknowledgements, and issues `END`. Any error or manual interruption triggers an `ABORT` so the device can reset its session state.

## OTA Workflow

1. Flash the base firmware via USB so the device is running the CAN OTA-ready sketch.
2. Connect the RP2040/MCP2515 node and the host machine to the same 500 kbps CAN bus.
3. Generate the `.bin` to be delivered and run `can_ota_host.py` with the desired interface options.
4. Observe log output and the device serial console; the device keeps blinking during the transfer.
5. After `OTA_STATUS_COMPLETE`, the firmware triggers a watchdog reboot (RP2040) to boot the newly written image.
6. Optional: reconnect via Serial or monitor the CAN bus for an `OTA_STATUS_IDLE` frame confirming the rebooted firmware is ready.

## Troubleshooting

- **No status frames received** – Verify wiring to the MCP2515, CAN termination, and that the host tool is listening on the correct interface/bus speed.
- **Repeated `DATA_ERROR` or `ABORTED` statuses** – Check for bus congestion or dropped frames; increase `--timeout`/`--retries` and ensure the chunk generator is using the correct firmware size.
- **CRC verification failure** – Ensure the host binary is unchanged between `BEGIN` and `END`; avoid using UF2 images as OTA payloads.
- **Device does not reboot** – Confirm the watchdog is enabled in your hardware build; otherwise, trigger a manual reset after a successful OTA.
- **LED not blinking** – The LED is driven through the PIO WS2812 helper; verify GPIO16 is free and that the PIO program was loaded (check build output for errors).

## Extending the Project

- Add encryption/authentication to the OTA protocol for secure deployment.
- Increase chunk size by migrating to CAN-FD, or implement dynamic chunk sizing with sequence numbers.
- Integrate the host tool into CI/CD pipelines by scripting Arduino CLI compilation and automatic CAN deployment.
- Report detailed progress to a UI dashboard by consuming the status counters.
