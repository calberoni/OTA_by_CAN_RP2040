#!/usr/bin/env python3
"""Host-side CAN OTA helper for pulsadores_can_v1_ota firmware."""

import argparse
import logging
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import can


OTA_COMMAND_CAN_ID = 0x381
OTA_STATUS_CAN_ID = 0x382

OTA_CMD_BEGIN = 0x01
OTA_CMD_DATA = 0x02
OTA_CMD_END = 0x03
OTA_CMD_ABORT = 0x04

OTA_STATUS_IDLE = 0x00
OTA_STATUS_BEGIN_OK = 0x10
OTA_STATUS_BEGIN_ERROR = 0x90
OTA_STATUS_DATA_OK = 0x11
OTA_STATUS_DATA_ERROR = 0x91
OTA_STATUS_COMPLETE = 0x12
OTA_STATUS_VERIFY_FAIL = 0x92
OTA_STATUS_ABORTED = 0x93

CHUNK_DATA_BYTES = 6


@dataclass
class OtaStatus:
    status: int
    detail: int
    received: int
    expected: int

    @classmethod
    def from_can_message(cls, msg: can.Message) -> "OtaStatus":
        data = msg.data
        received = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
        expected = data[6] | (data[7] << 8)
        return cls(status=data[0], detail=data[1], received=received, expected=expected)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def read_status(bus: can.BusABC, timeout: float) -> Optional[OtaStatus]:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            return None
        if msg.arbitration_id != OTA_STATUS_CAN_ID:
            continue
        if len(msg.data) < 8:
            logging.warning("Ignoring status frame with invalid length: %s", msg)
            continue
        status = OtaStatus.from_can_message(msg)
        logging.debug(
            "Status 0x%02X detail 0x%02X received=%d expected=%d",
            status.status,
            status.detail,
            status.received,
            status.expected,
        )
        return status


def send_frame(bus: can.BusABC, payload: bytes) -> None:
    message = can.Message(
        arbitration_id=OTA_COMMAND_CAN_ID,
        is_extended_id=False,
        data=payload,
    )
    bus.send(message)


def expect_status(bus: can.BusABC, timeout: float, *expected_codes: int) -> OtaStatus:
    status = read_status(bus, timeout)
    if status is None:
        raise TimeoutError("Timed out waiting for OTA status")
    if expected_codes and status.status not in expected_codes:
        raise RuntimeError(
            f"Unexpected status 0x{status.status:02X} (detail 0x{status.detail:02X})"
        )
    return status


def send_begin(bus: can.BusABC, firmware: bytes) -> None:
    size = len(firmware)
    crc = crc16_ccitt(firmware)
    payload = struct.pack("<BIH", OTA_CMD_BEGIN, size, crc)
    send_frame(bus, payload)


def send_chunk(bus: can.BusABC, chunk: bytes) -> None:
    payload = bytes([OTA_CMD_DATA, len(chunk)]) + chunk
    send_frame(bus, payload)


def send_end(bus: can.BusABC) -> None:
    send_frame(bus, bytes([OTA_CMD_END]))


def send_abort(bus: can.BusABC) -> None:
    send_frame(bus, bytes([OTA_CMD_ABORT]))


def host_ota(
    bus: can.BusABC,
    firmware: bytes,
    timeout: float,
    max_retries: int,
) -> None:
    logging.info("Starting OTA session: %d bytes", len(firmware))

    send_begin(bus, firmware)
    status = expect_status(bus, timeout, OTA_STATUS_BEGIN_OK)
    logging.info("Node ready (detail 0x%02X)", status.detail)

    offset = 0
    while offset < len(firmware):
        chunk = firmware[offset : offset + CHUNK_DATA_BYTES]
        retries = 0
        while True:
            logging.debug("Enviando chunk offset=%d len=%d", offset, len(chunk))
            send_chunk(bus, chunk)
            try:
                status = expect_status(
                    bus,
                    timeout,
                    OTA_STATUS_DATA_OK,
                    OTA_STATUS_DATA_ERROR,
                    OTA_STATUS_ABORTED,
                )
            except TimeoutError:
                status = None

            if status is None:
                retries += 1
                if retries > max_retries:
                    raise TimeoutError("Sin respuesta tras varios reintentos en DATA")
                logging.warning("Sin respuesta del nodo; reintentando chunk")
                continue

            if status.status == OTA_STATUS_DATA_OK:
                offset += len(chunk)
                break

            if status.status in (OTA_STATUS_DATA_ERROR, OTA_STATUS_ABORTED):
                retries += 1
                if retries > max_retries:
                    raise RuntimeError(
                        f"Node kept returning status 0x{status.status:02X}"
                    )
            logging.warning(
                "Node reported error 0x%02X detail 0x%02X; retrying chunk",
                status.status,
                status.detail,
            )
            continue

            raise RuntimeError(f"Unexpected status during DATA: 0x{status.status:02X}")

    send_end(bus)
    final_status = expect_status(
        bus,
        timeout,
        OTA_STATUS_COMPLETE,
        OTA_STATUS_VERIFY_FAIL,
        OTA_STATUS_ABORTED,
    )

    if final_status.status == OTA_STATUS_COMPLETE:
        logging.info("OTA completed successfully; node will reboot")
        return

    if final_status.status == OTA_STATUS_VERIFY_FAIL:
        raise RuntimeError(
            f"CRC verification failed (detail 0x{final_status.detail:02X})"
        )

    if final_status.status == OTA_STATUS_ABORTED:
        raise RuntimeError(f"Node aborted OTA (detail 0x{final_status.detail:02X})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send firmware over CAN to pulsadores_can_v1_ota.ino",
    )
    parser.add_argument("firmware", type=Path, help="Path to the .bin firmware file")
    parser.add_argument(
        "--bustype",
        default="socketcan",
        help="CAN interface type for python-can (e.g. socketcan, slcan, pcantype)",
    )
    parser.add_argument(
        "--channel",
        default="can0",
        help="CAN channel/interface (e.g. can0, slcan0, PCAN_USBBUS1)",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=500000,
        help="CAN bus bitrate in bit/s",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Timeout while waiting for OTA status frames (seconds)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=5,
        help="Maximum number of retries per chunk",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ...)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    firmware_path: Path = args.firmware
    if not firmware_path.exists():
        logging.error("Firmware file not found: %s", firmware_path)
        return 1

    firmware = firmware_path.read_bytes()
    if not firmware:
        logging.error("Firmware file is empty")
        return 1

    logging.debug("Firmware CRC16=0x%04X", crc16_ccitt(firmware))

    try:
        bus = can.Bus(
            bustype=args.bustype,
            channel=args.channel,
            bitrate=args.bitrate,
        )
    except Exception as exc:  # pragma: no cover - backend dependent
        logging.error("Failed to open CAN interface: %s", exc)
        return 1

    try:
        # Drain pending status frames before starting
        while True:
            msg = bus.recv(timeout=0.05)
            if msg is None:
                break
        host_ota(bus, firmware, timeout=args.timeout, max_retries=args.retries)
    except KeyboardInterrupt:
        logging.warning("Manual interruption, sending ABORT")
        try:
            send_abort(bus)
        except Exception:  # pragma: no cover - best effort
            pass
        return 1
    except Exception as exc:
        logging.error("Error during OTA: %s", exc)
        try:
            send_abort(bus)
        except Exception:  # pragma: no cover - best effort
            pass
        return 1
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
