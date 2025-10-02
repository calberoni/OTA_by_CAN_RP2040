#include "ota_commands.hpp"

#include <cstring>
#include <utility>

namespace ota {

namespace {
constexpr std::size_t kMaxImageSize = 512 * 1024;

void abort_session(OtaContext &ctx, std::uint8_t detail) {
    ctx.flash.abort();
    ctx.session.reset();
    ctx.send_status(OTA_STATUS_ABORTED, detail);
}

} // namespace

void BeginCommand::execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) {
    if (length < 6) {
        ctx.send_status(OTA_STATUS_BEGIN_ERROR, 0x01);
        return;
    }

    auto &session = ctx.session;
    if (session.active) {
        ctx.flash.abort();
        session.reset();
    }

    std::uint32_t image_size = static_cast<std::uint32_t>(payload[0]) |
                               (static_cast<std::uint32_t>(payload[1]) << 8) |
                               (static_cast<std::uint32_t>(payload[2]) << 16) |
                               (static_cast<std::uint32_t>(payload[3]) << 24);

    std::uint16_t image_crc = static_cast<std::uint16_t>(payload[4]) |
                              (static_cast<std::uint16_t>(payload[5]) << 8);

    if (image_size == 0 || image_size > kMaxImageSize) {
        ctx.send_status(OTA_STATUS_BEGIN_ERROR, 0x02);
        return;
    }

    if (!ctx.flash.begin(image_size)) {
        ctx.send_status(OTA_STATUS_BEGIN_ERROR, 0x03);
        return;
    }

    session.active = true;
    session.expected_size = image_size;
    session.expected_crc = image_crc;
    session.received_size = 0;
    session.reset_crc();

    ctx.send_status(OTA_STATUS_BEGIN_OK);
}

void DataCommand::execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) {
    auto &session = ctx.session;
    if (!session.active) {
        ctx.send_status(OTA_STATUS_DATA_ERROR, 0x10);
        return;
    }

    if (length < 1) {
        ctx.send_status(OTA_STATUS_DATA_ERROR, 0x11);
        return;
    }

    std::uint8_t chunk_length = payload[0];
    if (chunk_length == 0 || chunk_length > 6 || length < (1 + chunk_length)) {
        ctx.send_status(OTA_STATUS_DATA_ERROR, 0x12);
        return;
    }

    if (session.received_size + chunk_length > session.expected_size) {
        abort_session(ctx, 0x13);
        return;
    }

    const std::uint8_t *chunk = payload + 1;
    if (!ctx.flash.write_chunk(chunk, chunk_length)) {
        abort_session(ctx, 0x14);
        return;
    }

    session.received_size += chunk_length;
    session.update_crc(chunk, chunk_length);
    ctx.send_status(OTA_STATUS_DATA_OK);
}

void EndCommand::execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) {
    (void)payload;
    (void)length;

    auto &session = ctx.session;
    if (!session.active) {
        ctx.send_status(OTA_STATUS_BEGIN_ERROR, 0x20);
        return;
    }

    if (session.received_size != session.expected_size) {
        abort_session(ctx, 0x21);
        return;
    }

    if (session.running_crc != session.expected_crc) {
        ctx.flash.abort();
        ctx.send_status(OTA_STATUS_VERIFY_FAIL, 0x22);
        session.reset();
        return;
    }

    if (!ctx.flash.finalize(false)) {
        abort_session(ctx, 0x23);
        return;
    }

    ctx.send_status(OTA_STATUS_COMPLETE);
    session.active = false;
    session.pending_reboot = true;
}

void AbortCommand::execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) {
    (void)payload;
    (void)length;

    if (!ctx.session.active) {
        ctx.send_status(OTA_STATUS_ABORTED, 0x30);
        return;
    }

    abort_session(ctx, 0x30);
}

CommandDispatcher::CommandDispatcher() {
    commands_.emplace(OTA_CMD_BEGIN, std::make_unique<BeginCommand>());
    commands_.emplace(OTA_CMD_DATA, std::make_unique<DataCommand>());
    commands_.emplace(OTA_CMD_END, std::make_unique<EndCommand>());
    commands_.emplace(OTA_CMD_ABORT, std::make_unique<AbortCommand>());
}

void CommandDispatcher::dispatch(std::uint8_t opcode, const std::uint8_t *payload, std::size_t length, OtaContext &ctx) {
    auto it = commands_.find(opcode);
    if (it == commands_.end()) {
        ctx.send_status(OTA_STATUS_BEGIN_ERROR, 0xFF);
        return;
    }
    it->second->execute(payload, length, ctx);
}

} // namespace ota

