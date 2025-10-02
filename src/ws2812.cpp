#include "ws2812.hpp"

#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "ws2812.pio.h"

Ws2812::Ws2812(PIO pio, uint sm, uint pin) : pio_(pio), sm_(sm), pin_(pin) {}

bool Ws2812::initialize() {
    program_offset_ = pio_add_program(pio_, &ws2812_program);
    if (program_offset_ < 0) {
        return false;
    }

    pio_gpio_init(pio_, pin_);
    pio_sm_set_consecutive_pindirs(pio_, sm_, pin_, 1, true);

    pio_sm_config config = ws2812_program_get_default_config(program_offset_);
    sm_config_set_sideset_pins(&config, pin_);
    sm_config_set_out_shift(&config, false, true, 24);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    // Target 800kHz bitstream (assuming system clock at 125MHz)
    pio_sm_set_clkdiv(pio_, sm_, 1.25f);

    pio_sm_init(pio_, sm_, program_offset_, &config);
    pio_sm_set_enabled(pio_, sm_, true);

    set_color(0, 0, 0);
    return true;
}

void Ws2812::set_color(uint8_t r, uint8_t g, uint8_t b) {
    const uint32_t packed = (static_cast<uint32_t>(g) << 16) |
                            (static_cast<uint32_t>(r) << 8) |
                            static_cast<uint32_t>(b);
    put_pixel(packed);
}

void Ws2812::put_pixel(uint32_t rgb) {
    while (pio_sm_is_tx_fifo_full(pio_, sm_)) {
        tight_loop_contents();
    }
    pio_sm_put_blocking(pio_, sm_, rgb << 8u);
}
