#include "flash_writer.hpp"

#include <algorithm>

#include "hardware/flash.h"
#include "pico/stdlib.h"

bool FlashWriter::begin(std::size_t image_size_bytes) {
    buffer_.clear();
    buffer_.reserve(image_size_bytes);
    return true;
}

bool FlashWriter::write_chunk(const uint8_t *data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }
    buffer_.insert(buffer_.end(), data, data + length);
    return true;
}

bool FlashWriter::finalize(bool validate_only) {
    if (validate_only) {
        return true;
    }

    // NOTE: Programming user flash while executing from it requires a two-stage
    // bootloader and careful placement of the new image. This example keeps the
    // image in RAM to demonstrate the OTA pipeline and leaves the actual flash
    // handoff for the production bootloader.
    // Implementors can use flash_range_program / flash_range_erase here when the
    // appropriate memory layout is available.
    return true;
}

void FlashWriter::abort() {
    buffer_.clear();
}

