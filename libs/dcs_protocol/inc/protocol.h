#ifndef PROTOCOL_PROTOCOL_H
#define PROTOCOL_PROTOCOL_H
#include <cstddef>
#include <cstdint>
#include <optional>
#include <array>

// Namespace for use protection in larger scopes
namespace protocol {
    // Start byte for the protocol, constant
    constexpr std::byte START_BYTE{0xAA};
    // Max payload size for keeping memory allocation static
    constexpr std::size_t MAX_PL_SIZE = 13;
    // Max frame size (start byte + type + payload + checksum) for keeping memory allocation static
    constexpr  std::size_t MAX_FR_SIZE = MAX_PL_SIZE + 3;

    // Enum class for message type bytes
    enum class MsgType : uint8_t {
        // Guard for unset message type in ParseResult
        UNSET = 0x00,

        ARM_CMD = 0x01, // GCS -> DRONE
        GOTO_CMD = 0x02, // GCS -> DRONE
        LAND_CMD = 0x03, // GCS -> DRONE
        TELEMETRY = 0x04, // DRONE -> GCS
        LOG = 0x05, // DRONE -> GCS
    };

    // Enum class for log codes
    enum class LogCode : uint8_t {
        // Guard for unset log code type in payload
        UNSET = 0,

        // State codes
        ST_DISARMED,
        ST_ARMING, // When climbing to alt 20m
        ST_IDLE,
        ST_GOTO,
        ST_LANDING,

        // Command codes
        CMD_ARM,
        CMD_GOTO,
        CMD_LAND,

        // Error codes
        ERR_CHECKSUM,
        ERR_INVALID_CMD,
        ERR_ARM_REJECTED,
        ERR_GOTO_REJECTED,
        ERR_LAND_REJECTED,
        ERR_GEOFENCE,
        ERR_SYS_UART,
        ERR_SYS_RTOS,
    };

    // Payload for GOTO messages
    struct GotoPL {
        float X = 0.0f;
        float Y = 0.0f;
    };
    // Payload for Telemetry messages
    struct TelemetryPL {
        float X = 0.0f;
        float Y = 0.0f;
        float alt = 0.0f;
        uint8_t armed = 0;
    };
    // Payload for log messages
    struct LogPL {
        LogCode code = LogCode::UNSET;
        uint8_t detail = 0; // Extra detail for system errors
    };
    // Fixed size frame
    struct Frame {
        std::array<std::byte, MAX_FR_SIZE> frame{};
        size_t size = 0;
    };
    // Payload raw data to be used in parsing result
    struct PayloadRaw {
        std::array<std::byte, MAX_PL_SIZE> payload{};
        size_t size = 0;
    };
    // Parsing result
    struct ParseResult {
        MsgType type = MsgType::UNSET;
        PayloadRaw payload;
        std::size_t bytes_read = 0;
    };
    // GCS encoders

    // Arm and Land encoders
    Frame encodeArm();
    Frame encodeLand();
    // GoTo message encoder
    Frame encodeGoTo(const GotoPL& pl);

    // DRONE encoders

    // Telemetry message encoder
    Frame encodeTelemetry(const TelemetryPL& pl);
    // Log message encoder
    Frame encodeLog(const LogPL& pl);

    //Parser
    std::optional<ParseResult> parseFrame(const std::byte* buffer, std::size_t bufferLen);

    // GCS decoders
    TelemetryPL decodeTelemetry(const PayloadRaw& payload);
    LogPL decodeLog(const PayloadRaw& payload);

    // Drone decoders
    GotoPL decodeGoto(const PayloadRaw& payload);

    // Checksum calculation function
    uint8_t checkSum(MsgType type, const std::byte* payload, std::size_t size);
}
#endif //PROTOCOL_PROTOCOL_H
