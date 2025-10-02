#include "mcp2515.hpp"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

namespace {
constexpr uint8_t CMD_RESET = 0xC0;
constexpr uint8_t CMD_READ = 0x03;
constexpr uint8_t CMD_WRITE = 0x02;
constexpr uint8_t CMD_BIT_MODIFY = 0x05;
constexpr uint8_t CMD_READ_STATUS = 0xA0;
constexpr uint8_t CMD_RX_STATUS = 0xB0;
constexpr uint8_t CMD_READ_RX_BUFFER = 0x90;  // Start from RXB0SIDH
constexpr uint8_t CMD_LOAD_TX_BUFFER = 0x40;  // Start with TXB0SIDH
constexpr uint8_t CMD_RTS_TXB0 = 0x81;

constexpr uint8_t REG_CANCTRL = 0x0F;
constexpr uint8_t REG_CANSTAT = 0x0E;
constexpr uint8_t REG_CNF1 = 0x2A;
constexpr uint8_t REG_CNF2 = 0x29;
constexpr uint8_t REG_CNF3 = 0x28;
constexpr uint8_t REG_RXB0CTRL = 0x60;
constexpr uint8_t REG_RXB1CTRL = 0x70;

constexpr uint8_t CANCTRL_MODE_BITS = 0xE0;
constexpr uint8_t CANCTRL_MODE_CONFIG = 0x80;
constexpr uint8_t CANCTRL_MODE_NORMAL = 0x00;

constexpr uint8_t RX_STATUS_RX0IF = 0x01;
constexpr uint8_t RX_STATUS_RX1IF = 0x02;

uint8_t calculate_tq_count(uint32_t bitrate_hz, uint32_t clock_hz, uint8_t &brp_out) {
    for (uint8_t brp = 0; brp < 64; ++brp) {
        uint32_t tq = 2 * (brp + 1);
        uint32_t target_tq = clock_hz / (bitrate_hz * tq);
        if (target_tq >= 5 && target_tq <= 25) {
            brp_out = brp;
            return target_tq;
        }
    }
    brp_out = 0;
    return 0;
}

} // namespace

Mcp2515::Mcp2515(spi_inst_t *spi_instance, uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin)
    : spi_(spi_instance), cs_pin_(cs_pin), sck_pin_(sck_pin), mosi_pin_(mosi_pin), miso_pin_(miso_pin) {
}

bool Mcp2515::initialize(uint32_t desired_bitrate_hz) {
    spi_init(spi_, 1'000'000);
    gpio_set_function(sck_pin_, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin_, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin_, GPIO_FUNC_SPI);

    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    cs_deselect();

    reset();
    sleep_ms(5);

    if (!set_mode(CANCTRL_MODE_CONFIG)) {
        return false;
    }

    if (!configure_bitrate(desired_bitrate_hz)) {
        return false;
    }

    // Configure receive buffers to accept any standard frame
    write_register(REG_RXB0CTRL, 0x00);
    write_register(REG_RXB1CTRL, 0x00);

    if (!set_mode(CANCTRL_MODE_NORMAL)) {
        return false;
    }

    return true;
}

bool Mcp2515::configure_bitrate(uint32_t bitrate_hz) {
    constexpr uint32_t oscillator_hz = 16'000'000; // common on MCP2515 breakout
    uint8_t brp = 0;
    uint8_t tq_count = calculate_tq_count(bitrate_hz, oscillator_hz, brp);
    if (tq_count == 0) {
        return false;
    }

    // We aim for: SyncSeg = 1 TQ, PropSeg = 2 TQ, PS1 = (tq_count / 2), PS2 = tq_count - (PropSeg + PS1 + SyncSeg)
    uint8_t prop_seg = 2;
    uint8_t ps1 = (tq_count / 2);
    if (ps1 < 1) {
        ps1 = 1;
    }
    uint8_t ps2 = tq_count - (1 + prop_seg + ps1);
    if (ps2 < 2) {
        ps2 = 2;
    }

    uint8_t cnf1 = (brp & 0x3F);
    uint8_t cnf2 = 0x80 | ((ps1 - 1) << 3) | (prop_seg - 1);
    uint8_t cnf3 = (ps2 - 1);

    write_register(REG_CNF1, cnf1);
    write_register(REG_CNF2, cnf2);
    write_register(REG_CNF3, cnf3);
    return true;
}

bool Mcp2515::begin_packet(uint16_t std_id) {
    if (std_id > 0x7FF) {
        return false;
    }
    tx_frame_.id = std_id;
    tx_frame_.dlc = 0;
    tx_index_ = 0;
    return true;
}

void Mcp2515::write(uint8_t byte) {
    if (tx_frame_.dlc < tx_frame_.data.size()) {
        tx_frame_.data[tx_frame_.dlc++] = byte;
    }
}

bool Mcp2515::end_packet() {
    tx_frame_.dlc = static_cast<uint8_t>(tx_frame_.dlc > 8 ? 8 : tx_frame_.dlc);
    return transmit_frame(tx_frame_);
}

int Mcp2515::parse_packet() {
    Frame candidate;
    if (!read_frame(candidate)) {
        return 0;
    }
    rx_frame_ = candidate;
    rx_index_ = 0;
    return rx_frame_.dlc;
}

uint8_t Mcp2515::read() {
    if (rx_index_ >= rx_frame_.dlc) {
        return 0;
    }
    return rx_frame_.data[rx_index_++];
}

bool Mcp2515::read_frame(Frame &frame) {
    cs_select();
    uint8_t command = CMD_RX_STATUS;
    spi_write_blocking(spi_, &command, 1);
    uint8_t status = 0;
    spi_read_blocking(spi_, 0x00, &status, 1);
    cs_deselect();

    if ((status & (RX_STATUS_RX0IF | RX_STATUS_RX1IF)) == 0) {
        return false;
    }

    uint8_t buffer_cmd = (status & RX_STATUS_RX0IF) ? CMD_READ_RX_BUFFER : (CMD_READ_RX_BUFFER | 0x04);

    cs_select();
    spi_write_blocking(spi_, &buffer_cmd, 1);

    uint8_t header[5] = {};
    spi_read_blocking(spi_, 0x00, header, 5);

    frame.id = static_cast<uint16_t>(header[0] << 3 | (header[1] >> 5));
    frame.dlc = header[4] & 0x0F;
    if (frame.dlc > 8) {
        frame.dlc = 8;
    }

    for (uint8_t i = 0; i < frame.dlc; ++i) {
        uint8_t byte = 0;
        spi_read_blocking(spi_, 0x00, &byte, 1);
        frame.data[i] = byte;
    }
    cs_deselect();
    return true;
}

bool Mcp2515::transmit_frame(const Frame &frame) {
    uint8_t buffer_cmd = CMD_LOAD_TX_BUFFER;
    cs_select();
    spi_write_blocking(spi_, &buffer_cmd, 1);

    uint8_t sid_high = static_cast<uint8_t>(frame.id >> 3);
    uint8_t sid_low = static_cast<uint8_t>((frame.id & 0x07) << 5);

    uint8_t header[5] = {sid_high, sid_low, 0x00, 0x00, frame.dlc & 0x0F};
    spi_write_blocking(spi_, header, sizeof(header));
    spi_write_blocking(spi_, frame.data.data(), frame.dlc);
    cs_deselect();

    cs_select();
    uint8_t rts_cmd = CMD_RTS_TXB0;
    spi_write_blocking(spi_, &rts_cmd, 1);
    cs_deselect();

    return true;
}

void Mcp2515::reset() {
    cs_select();
    uint8_t cmd = CMD_RESET;
    spi_write_blocking(spi_, &cmd, 1);
    cs_deselect();
}

void Mcp2515::write_register(uint8_t address, uint8_t value) {
    cs_select();
    uint8_t buffer[3] = {CMD_WRITE, address, value};
    spi_write_blocking(spi_, buffer, sizeof(buffer));
    cs_deselect();
}

uint8_t Mcp2515::read_register(uint8_t address) {
    cs_select();
    uint8_t buffer[2] = {CMD_READ, address};
    spi_write_blocking(spi_, buffer, 2);
    uint8_t value = 0;
    spi_read_blocking(spi_, 0x00, &value, 1);
    cs_deselect();
    return value;
}

void Mcp2515::bit_modify(uint8_t address, uint8_t mask, uint8_t value) {
    cs_select();
    uint8_t buffer[4] = {CMD_BIT_MODIFY, address, mask, value};
    spi_write_blocking(spi_, buffer, sizeof(buffer));
    cs_deselect();
}

bool Mcp2515::set_mode(uint8_t mode) {
    bit_modify(REG_CANCTRL, CANCTRL_MODE_BITS, mode);
    const absolute_time_t deadline = make_timeout_time_ms(10);
    while (!time_reached(deadline)) {
        uint8_t status = read_register(REG_CANSTAT);
        if ((status & CANCTRL_MODE_BITS) == mode) {
            return true;
        }
        sleep_us(50);
    }
    return false;
}

void Mcp2515::cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin_, 0);
    asm volatile("nop \n nop \n nop");
}

void Mcp2515::cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin_, 1);
    asm volatile("nop \n nop \n nop");
}
