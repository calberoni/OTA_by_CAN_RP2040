#include "ota_session.hpp"

#include <array>

namespace {
constexpr std::uint16_t CRC_POLY = 0x1021;

std::uint16_t crc16_ccitt_update(std::uint16_t crc, std::uint8_t data) {
    crc ^= static_cast<std::uint16_t>(data) << 8;
    for (std::uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ CRC_POLY;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

} // namespace

void OtaSession::reset() {
    active = false;
    pending_reboot = false;
    expected_size = 0;
    received_size = 0;
    expected_crc = 0;
    running_crc = 0xFFFF;
}

void OtaSession::reset_crc() {
    running_crc = 0xFFFF;
}

void OtaSession::update_crc(const uint8_t *data, std::size_t length) {
    for (std::size_t i = 0; i < length; ++i) {
        running_crc = crc16_ccitt_update(running_crc, data[i]);
    }
}

void OtaContext::send_status(std::uint8_t status, std::uint8_t detail) {
    if (!can.begin_packet(0x382)) {
        return;
    }

    can.write(status);
    can.write(detail);
    can.write(static_cast<uint8_t>((session.received_size >> 0) & 0xFF));
    can.write(static_cast<uint8_t>((session.received_size >> 8) & 0xFF));
    can.write(static_cast<uint8_t>((session.received_size >> 16) & 0xFF));
    can.write(static_cast<uint8_t>((session.received_size >> 24) & 0xFF));
    can.write(static_cast<uint8_t>((session.expected_size >> 0) & 0xFF));
    can.write(static_cast<uint8_t>((session.expected_size >> 8) & 0xFF));
    can.end_packet();
}

