#include <array>
#include <cstdint>
#include <cstdio>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "flash_writer.hpp"
#include "mcp2515.hpp"
#include "ota_commands.hpp"
#include "ota_session.hpp"
#include "ws2812.hpp"

int main() {
    stdio_init_all();

    constexpr uint CAN_CS_PIN = 5;
    constexpr uint CAN_SCK_PIN = 2;
    constexpr uint CAN_MOSI_PIN = 4;
    constexpr uint CAN_MISO_PIN = 3;
    constexpr uint32_t CAN_BAUDRATE = 500'000;

    constexpr uint PIXEL_PIN = 16;

    Mcp2515 can(spi0, CAN_CS_PIN, CAN_SCK_PIN, CAN_MOSI_PIN, CAN_MISO_PIN);
    if (!can.initialize(CAN_BAUDRATE)) {
        printf("MCP2515 init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    FlashWriter flash_writer;
    OtaSession session;
    session.reset();
    OtaContext ctx{can, flash_writer, session};

    ota::CommandDispatcher dispatcher;

    Ws2812 pixel(pio0, 0, PIXEL_PIN);
    if (!pixel.initialize()) {
        printf("WS2812 init failed\n");
    }

    ctx.send_status(ota::OTA_STATUS_IDLE);

    absolute_time_t last_blink = get_absolute_time();
    bool led_on = false;

    while (true) {
        int packet_size = can.parse_packet();
        while (packet_size > 0) {
            uint16_t packet_id = can.packet_id();
            std::array<uint8_t, 8> buffer{};
            std::size_t index = 0;

            while (can.available() && index < buffer.size()) {
                buffer[index++] = can.read();
            }

            if (packet_id == ota::OTA_COMMAND_CAN_ID && index > 0) {
                std::uint8_t opcode = buffer[0];
                const std::uint8_t *payload = buffer.data() + 1;
                std::size_t payload_length = index - 1;
                dispatcher.dispatch(opcode, payload, payload_length, ctx);
            }

            packet_size = can.parse_packet();
        }

        if (session.pending_reboot) {
            printf("Rebooting to apply OTA firmware...\n");
            sleep_ms(50);
            watchdog_reboot(0, 0, 0);
        }

        const uint32_t blink_interval_ms = 500;
        if (absolute_time_diff_us(last_blink, get_absolute_time()) >= blink_interval_ms * 1000) {
            last_blink = get_absolute_time();
            led_on = !led_on;
            if (led_on) {
                pixel.set_color(128, 128, 128);
            } else {
                pixel.set_color(0, 0, 0);
            }
        }

        tight_loop_contents();
    }

    return 0;
}

