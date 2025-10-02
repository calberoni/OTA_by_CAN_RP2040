#ifndef OTA_COMMANDS_HPP
#define OTA_COMMANDS_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "ota_session.hpp"

namespace ota {

constexpr std::uint16_t OTA_COMMAND_CAN_ID = 0x381;
constexpr std::uint8_t OTA_CMD_BEGIN = 0x01;
constexpr std::uint8_t OTA_CMD_DATA = 0x02;
constexpr std::uint8_t OTA_CMD_END = 0x03;
constexpr std::uint8_t OTA_CMD_ABORT = 0x04;

constexpr std::uint8_t OTA_STATUS_IDLE = 0x00;
constexpr std::uint8_t OTA_STATUS_BEGIN_OK = 0x10;
constexpr std::uint8_t OTA_STATUS_BEGIN_ERROR = 0x90;
constexpr std::uint8_t OTA_STATUS_DATA_OK = 0x11;
constexpr std::uint8_t OTA_STATUS_DATA_ERROR = 0x91;
constexpr std::uint8_t OTA_STATUS_COMPLETE = 0x12;
constexpr std::uint8_t OTA_STATUS_VERIFY_FAIL = 0x92;
constexpr std::uint8_t OTA_STATUS_ABORTED = 0x93;

class Command {
public:
    virtual ~Command() = default;
    virtual void execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) = 0;
};

class BeginCommand final : public Command {
public:
    void execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) override;
};

class DataCommand final : public Command {
public:
    void execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) override;
};

class EndCommand final : public Command {
public:
    void execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) override;
};

class AbortCommand final : public Command {
public:
    void execute(const std::uint8_t *payload, std::size_t length, OtaContext &ctx) override;
};

class CommandDispatcher {
public:
    CommandDispatcher();
    void dispatch(std::uint8_t opcode, const std::uint8_t *payload, std::size_t length, OtaContext &ctx);

private:
    std::unordered_map<std::uint8_t, std::unique_ptr<Command>> commands_;
};

} // namespace ota

#endif // OTA_COMMANDS_HPP
