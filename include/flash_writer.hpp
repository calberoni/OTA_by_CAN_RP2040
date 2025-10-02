#ifndef FLASH_WRITER_HPP
#define FLASH_WRITER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

class FlashWriter {
public:
    bool begin(std::size_t image_size_bytes);
    bool write_chunk(const uint8_t *data, std::size_t length);
    bool finalize(bool validate_only);
    void abort();

    std::size_t bytes_written() const { return buffer_.size(); }

private:
    std::vector<uint8_t> buffer_;
};

#endif // FLASH_WRITER_HPP
