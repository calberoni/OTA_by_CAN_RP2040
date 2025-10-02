#ifndef MCP2515_HPP
#define MCP2515_HPP

#include <cstdint>
#include <cstddef>
#include <array>

#include "hardware/spi.h"
#include "pico/stdlib.h"

class Mcp2515 {
public:
    struct Frame {
        uint16_t id = 0;
        uint8_t dlc = 0;
        std::array<uint8_t, 8> data{};
    };

    Mcp2515(spi_inst_t *spi_instance, uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin);

    bool initialize(uint32_t desired_bitrate_hz);

    // Transmission helpers matching Arduino-like API
    bool begin_packet(uint16_t std_id);
    void write(uint8_t byte);
    bool end_packet();

    // Reception helpers matching Arduino-like API
    int parse_packet();
    uint16_t packet_id() const { return rx_frame_.id; }
    bool available() const { return rx_index_ < rx_frame_.dlc; }
    uint8_t read();

private:
    bool configure_bitrate(uint32_t bitrate_hz);
    void reset();
    void write_register(uint8_t address, uint8_t value);
    uint8_t read_register(uint8_t address);
    void bit_modify(uint8_t address, uint8_t mask, uint8_t value);
    bool set_mode(uint8_t mode);
    bool read_frame(Frame &frame);
    bool transmit_frame(const Frame &frame);

    void cs_select();
    void cs_deselect();

    spi_inst_t *spi_;
    uint cs_pin_;
    uint sck_pin_;
    uint mosi_pin_;
    uint miso_pin_;

    Frame tx_frame_{};
    Frame rx_frame_{};
    uint8_t tx_index_ = 0;
    uint8_t rx_index_ = 0;
};

#endif // MCP2515_HPP
