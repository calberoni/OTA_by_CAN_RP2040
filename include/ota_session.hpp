#ifndef OTA_SESSION_HPP
#define OTA_SESSION_HPP

#include <cstddef>
#include <cstdint>

#include "flash_writer.hpp"
#include "mcp2515.hpp"

struct OtaSession {
    bool active = false;
    bool pending_reboot = false;
    std::uint32_t expected_size = 0;
    std::uint32_t received_size = 0;
    std::uint16_t expected_crc = 0;
    std::uint16_t running_crc = 0xFFFF;

    void reset();
    void reset_crc();
    void update_crc(const uint8_t *data, std::size_t length);
};

struct OtaContext {
    Mcp2515 &can;
    FlashWriter &flash;
    OtaSession &session;

    void send_status(std::uint8_t status, std::uint8_t detail = 0x00);
};

#endif // OTA_SESSION_HPP
