#ifndef WS2812_HPP
#define WS2812_HPP

#include <cstdint>

#include "hardware/pio.h"

class Ws2812 {
public:
    Ws2812(PIO pio, uint sm, uint pin);

    bool initialize();
    void set_color(uint8_t r, uint8_t g, uint8_t b);

private:
    void put_pixel(uint32_t rgb);

    PIO pio_;
    uint sm_;
    uint pin_;
    int program_offset_ = -1;
};

#endif // WS2812_HPP
