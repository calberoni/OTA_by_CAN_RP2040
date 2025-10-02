# CAN OTA Firmware for RP2040

This project pairs a CAN-enabled firmware for an RP2040-based controller with a companion Python host utility to perform over-the-air (OTA) firmware updates via an MCP2515 CAN transceiver. The firmware keeps the device responsive while new images are streamed over CAN, verifies integrity with CRC16, and automatically reboots into the freshly written sketch once the transfer succeeds.

## Repository Layout

- `example_can_ota.ino` – minimal example sketch demonstrating the OTA logic without the legacy switch handling.
- `can_ota_host.py` – Python 3 script that fragments a compiled `.bin` file into CAN frames, manages retries, and tracks device status codes.

## Firmware Highlights

- Targets RP2040 boards using the Arduino core, together with an external MCP2515 CAN controller.
- Handles OTA commands on CAN ID `0x381` and returns status on CAN ID `0x382`.
- Streams firmware directly to flash with the Arduino `Update` API while the current firmware keeps running.
- Validates each transfer with CRC16-CCITT and aborts safely on mismatch or timeout.
- Provides a default 500 ms white blink on the NeoPixel (WS2812) to indicate the application loop is alive.

## Hardware Requirements

- RP2040 board (e.g., RP2040-Zero, Raspberry Pi Pico or compatible) running the Arduino core.
- MCP2515 CAN controller module wired to the RP2040 SPI pins and CS on GPIO5, SCK on GPIO2, MOSI on GPIO4, MISO on GPIO3.
- One WS2812/NeoPixel LED connected to GPIO16 (pin configurable in the sketch).
- CAN transceiver and a 500 kbps CAN bus for data exchange with the host tool.
- Stable 3.3 V power supply; ensure the MCP2515 module level shifts correctly for 3.3 V logic.

## Building and Flashing the Firmware

1. Install the Arduino core for RP2040 and the required libraries (`NeoPixelBus`, `Adafruit_MCP2515`).
2. Open `example_can_ota`  in the Arduino IDE.
3. Select the correct RP2040 board and serial port.
4. Compile and upload the sketch normally to seed the device with a baseline firmware.
5. To generate a binary for OTA updates, use the Arduino IDE "Export Compiled Binary" option or run Arduino CLI:

   ```bash
   arduino-cli compile --fqbn rp2040:rp2040:pico example_can_ota.ino --output-dir build
   ```

   The generated `.bin` file (not the `.uf2`) is the artifact consumed by the host tool.

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

- `firmware.bin` – path to the compiled Arduino binary (not UF2).
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
- **CRC verification failure** – Ensure the host binary is unchanged between `BEGIN` and `END`; avoid mixing `.uf2` files, which include metadata, with raw `.bin` images.
- **Device does not reboot** – Confirm the board defines `ARDUINO_ARCH_RP2040`; otherwise, a manual power cycle may be required.
- **LED not blinking** – The LED is managed by `NeoPixelBus`; verify the WS2812 wiring and that GPIO16 is free from conflicts.

## Extending the Project

- Add encryption/authentication to the OTA protocol for secure deployment.
- Increase chunk size by migrating to CAN-FD, or implement dynamic chunk sizing with sequence numbers.
- Integrate the host tool into CI/CD pipelines by scripting Arduino CLI compilation and automatic CAN deployment.
- Report detailed progress to a UI dashboard by consuming the status counters.

## License

No explicit license is provided in this repository. Consult the project owner before redistributing or reusing the code.

