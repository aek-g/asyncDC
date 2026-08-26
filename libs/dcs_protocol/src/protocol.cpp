#include "protocol.h"
#include <cstring>
// Namespace for use protection in larger scopes
namespace protocol {
    // Arm encoder
    Frame encodeArm() {
        // START BYTE + Type byte + Checksum
        Frame fr;
        fr.frame[0] = START_BYTE;
        fr.frame[1] = static_cast<std::byte>(MsgType::ARM_CMD);
        fr.frame[2] = static_cast<std::byte>(checkSum(MsgType::ARM_CMD, nullptr, 0));
        fr.size = 3;

        return fr;
    }
    // Land encoder
    Frame encodeLand() {
        // START BYTE + Type byte + Checksum
        Frame fr;
        fr.frame[0] = START_BYTE;
        fr.frame[1] = static_cast<std::byte>(MsgType::LAND_CMD);
        fr.frame[2] = static_cast<std::byte>(checkSum(MsgType::LAND_CMD, nullptr, 0));
        fr.size = 3;

        return fr;
    }
    // GoTo message encoder
    Frame encodeGoTo(const GotoPL& pl) {
        Frame fr;
        fr.frame[0] = START_BYTE;
        fr.frame[1] = static_cast<std::byte>(MsgType::GOTO_CMD);
        // Payload copied from GotoPL, 4 + 4 bytes
        std::memcpy(&fr.frame[2], &pl.X, 4);
        std::memcpy(&fr.frame[6], &pl.Y, 4);
        fr.frame[10] = static_cast<std::byte>(checkSum(MsgType::GOTO_CMD, &fr.frame[2], 8));
        fr.size = 11;

        return fr;
    }

    // DRONE encoders

    // Telemetry message encoder
    Frame encodeTelemetry(const TelemetryPL& pl) {
        Frame fr;
        fr.frame[0] = START_BYTE;
        fr.frame[1] = static_cast<std::byte>(MsgType::TELEMETRY);
        // Payload copied from TelemetryPL, 4 + 4 + 4 + 1 bytes
        std::memcpy(&fr.frame[2], &pl.X, 4);
        std::memcpy(&fr.frame[6], &pl.Y, 4);
        std::memcpy(&fr.frame[10], &pl.alt, 4);
        fr.frame[14] = static_cast<std::byte>(pl.armed);
        fr.frame[15] = static_cast<std::byte>(checkSum(MsgType::TELEMETRY, &fr.frame[2], 13));
        fr.size = 16;

        return fr;
    }
    // Log message encoder
    Frame encodeLog(const LogPL& pl) {
        Frame fr;
        fr.frame[0] = START_BYTE;
        fr.frame[1] = static_cast<std::byte>(MsgType::LOG);
        // Code and detail bytes as payload
        fr.frame[2] = static_cast<std::byte>(pl.code);
        fr.frame[3] = static_cast<std::byte>(pl.detail);
        fr.frame[4] = static_cast<std::byte>(checkSum(MsgType::LOG, &fr.frame[2], 2));
        fr.size = 5;

        return fr;
    }

    //Parser
    std::optional<ParseResult> parseFrame(const std::byte* buffer, std::size_t bufferLen) {
        // Check to see if buffer is too small or does not start with START BYTE
        // Start byte recognition done in GCS/DRONE code
        if (bufferLen < 3 || buffer[0] != START_BYTE) {
            return std::nullopt;
        }
        // Extract byte and initialize ParseResult
        auto type = static_cast<MsgType>(buffer[1]);
        ParseResult result;

        // Switch for every message type (except UNSET)
        switch (type) {
            case MsgType::ARM_CMD:
                // Check third byte for checksum of ARM and null
                if ( buffer[2] == static_cast<std::byte>(checkSum(MsgType::ARM_CMD, nullptr, 0))) {
                    result.type = MsgType::ARM_CMD;
                    result.bytes_read = 3;
                    return result;
                }
                break;
            case MsgType::LAND_CMD:
                // Check third byte for checksum of LAND and null
                if ( buffer[2] == static_cast<std::byte>(checkSum(MsgType::LAND_CMD, nullptr, 0))) {
                    result.type = MsgType::LAND_CMD;
                    result.bytes_read = 3;
                    return result;
                }
                break;
            case MsgType::GOTO_CMD:
                // New length check for GOTO type
                if (bufferLen < 11) {
                    return std::nullopt;
                }
                // Check last (presumed correct) byte for checksum of type and payload
                if ( buffer[10] == static_cast<std::byte>(checkSum(MsgType::GOTO_CMD, &buffer[2], 8))) {
                    result.type = MsgType::GOTO_CMD;
                    // Copy payload to result
                    std::memcpy(&result.payload.payload[0], &buffer[2], 8);
                    result.payload.size = 8;
                    result.bytes_read = 11;
                    return result;
                }
                break;
            case MsgType::TELEMETRY:
                // New length check for Telemetry type
                if (bufferLen < 16) {
                    return std::nullopt;
                }
                // Check last (presumed correct) byte for checksum of type and payload
                if ( buffer[15] == static_cast<std::byte>(checkSum(MsgType::TELEMETRY, &buffer[2], 13))) {
                    result.type = MsgType::TELEMETRY;
                    // Copy payload to result
                    std::memcpy(&result.payload.payload[0], &buffer[2], 13);
                    result.payload.size = 13;
                    result.bytes_read = 16;
                    return result;
                }
                break;
            case MsgType::LOG:
                if ( bufferLen < 5) {
                    return std::nullopt;
                }
                // Check last (presumed correct) byte for checksum of type and payload
                if ( buffer[4] == static_cast<std::byte>(checkSum(MsgType::LOG, &buffer[2], 2))) {
                    result.type = MsgType::LOG;
                    // Copy payload to result
                    std::memcpy(&result.payload.payload[0], &buffer[2], 2);
                    result.payload.size = 2;
                    result.bytes_read = 5;
                    return result;
                }
                break;
            default:
                return std::nullopt;
        }
        return std::nullopt;
    }

    // GCS decoders
    TelemetryPL decodeTelemetry(const PayloadRaw& payload) {
        // Create payload object and populate values
        TelemetryPL pl;
        std::memcpy(&pl.X, &payload.payload[0], 4);
        std::memcpy(&pl.Y, &payload.payload[4], 4);
        std::memcpy(&pl.alt, &payload.payload[8], 4);
        std::memcpy(&pl.armed, &payload.payload[12], 1);

        return pl;
    }
    LogPL decodeLog(const PayloadRaw& payload) {
        // Create log object and populate values
        LogPL pl;
        pl.code = static_cast<LogCode>(payload.payload[0]);
        pl.detail = static_cast<std::uint8_t>(payload.payload[1]);

        return pl;
    }

    // Drone decoders
    GotoPL decodeGoto(const PayloadRaw& payload) {
        // Create GoTo object and populate values
        GotoPL pl;
        std::memcpy(&pl.X, &payload.payload[0], 4);
        std::memcpy(&pl.Y, &payload.payload[4], 4);

        return pl;
    }

    // Checksum calculation function
    uint8_t checkSum(MsgType type, const std::byte* payload, std::size_t size) {
        // Start with type and do XOR on sum and every next byte
        auto sum = static_cast<uint8_t>(type);
        for (std::size_t i = 0; i < size; ++i) {
            // Simple XOR value for the checksum
            sum ^= static_cast<uint8_t>(payload[i]);
        }

        return sum;
    }
}
