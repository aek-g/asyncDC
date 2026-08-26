#include <gtest/gtest.h>
#include "protocol.h"
#include <cstring>

using namespace protocol;

// CHECKSUM

// Test for empty payload ARM/LAND = type byte
TEST(CheckSum_Test, EmptyToType) {
    uint8_t cs1 = checkSum(MsgType::ARM_CMD, nullptr, 0);
    EXPECT_EQ(cs1, static_cast<uint8_t>(MsgType::ARM_CMD));

    uint8_t cs2 = checkSum(MsgType::LAND_CMD, nullptr, 0);
    EXPECT_EQ(cs2, static_cast<uint8_t>(MsgType::LAND_CMD));
}
// ARM VS LAND - If the checksum for these empty payloads is different
TEST(CheckSum_Test, ArmVsLand) {
    uint8_t arm = checkSum(MsgType::ARM_CMD, nullptr, 0);
    uint8_t land = checkSum(MsgType::LAND_CMD, nullptr, 0);
    EXPECT_NE(arm, land);
}
// Compare checksum calculated on known byte values with preknown XOR value
TEST(CheckSum_Test, PayloadComparison) {
    // type = 0x02 (GOTO_CMD), payload = {0x0F, 0xF0}
    // 0x02 ^ 0x0F = 0x0D ; 0x0D ^ 0xF0 = 0xDD
    std::byte payload[2] = { std::byte{0x0F}, std::byte{0xF0} };
    uint8_t result = checkSum(MsgType::GOTO_CMD, payload, 2);
    EXPECT_EQ(result, 0xFD);
}

TEST(ChecksumTest, DifferentPayloads) {
    std::byte payloadA[2] = { std::byte{0x11}, std::byte{0x22} };
    std::byte payloadB[2] = { std::byte{0x33}, std::byte{0x44} };
    uint8_t resultA = checkSum(MsgType::TELEMETRY, payloadA, 2);
    uint8_t resultB = checkSum(MsgType::TELEMETRY, payloadB, 2);
    EXPECT_NE(resultA, resultB);
}

// ARM

TEST(EncodeArmTest, ProducesCorrectFrame) {
    Frame fr = encodeArm();
    ASSERT_EQ(fr.size, 3u);
    EXPECT_EQ(fr.frame[0], START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(MsgType::ARM_CMD));
    EXPECT_EQ(fr.frame[2], static_cast<std::byte>(checkSum(MsgType::ARM_CMD, nullptr, 0)));
}
// LAND

TEST(EncodeLandTest, ProducesCorrectFrame) {
    Frame fr = encodeLand();
    ASSERT_EQ(fr.size, 3u);
    EXPECT_EQ(fr.frame[0], START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(MsgType::LAND_CMD));
    EXPECT_EQ(fr.frame[2], static_cast<std::byte>(checkSum(MsgType::LAND_CMD, nullptr, 0)));
}

// Encode GoTo

TEST(EncodeGoToTest, ProducesCorrectFrameForPositiveValues) {
    GotoPL pl{12.5f, 7.25f};
    Frame fr = encodeGoTo(pl);

    ASSERT_EQ(fr.size, 11u);
    EXPECT_EQ(fr.frame[0], START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(MsgType::GOTO_CMD));

    float decodedX, decodedY;
    std::memcpy(&decodedX, &fr.frame[2], 4);
    std::memcpy(&decodedY, &fr.frame[6], 4);
    EXPECT_FLOAT_EQ(decodedX, 12.5f);
    EXPECT_FLOAT_EQ(decodedY, 7.25f);

    EXPECT_EQ(fr.frame[10], static_cast<std::byte>(checkSum(MsgType::GOTO_CMD, &fr.frame[2], 8)));
}

TEST(EncodeGoToTest, HandlesNegativeValues) {
    GotoPL pl{-42.75f, -0.5f};
    Frame fr = encodeGoTo(pl);

    float decodedX, decodedY;
    std::memcpy(&decodedX, &fr.frame[2], 4);
    std::memcpy(&decodedY, &fr.frame[6], 4);
    EXPECT_FLOAT_EQ(decodedX, -42.75f);
    EXPECT_FLOAT_EQ(decodedY, -0.5f);
}

TEST(EncodeGoToTest, HandlesZeroValues) {
    GotoPL pl{0.0f, 0.0f};
    Frame fr = encodeGoTo(pl);

    float decodedX, decodedY;
    std::memcpy(&decodedX, &fr.frame[2], 4);
    std::memcpy(&decodedY, &fr.frame[6], 4);
    EXPECT_FLOAT_EQ(decodedX, 0.0f);
    EXPECT_FLOAT_EQ(decodedY, 0.0f);
}

// Encode telemetry

TEST(EncodeTelemetryTest, ProducesCorrectFrameWhenArmed) {
    TelemetryPL pl{100.0f, -25.5f, 20.0f, 1};
    Frame fr = encodeTelemetry(pl);

    ASSERT_EQ(fr.size, 16u);
    EXPECT_EQ(fr.frame[0], START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(MsgType::TELEMETRY));

    float x, y, alt;
    std::memcpy(&x, &fr.frame[2], 4);
    std::memcpy(&y, &fr.frame[6], 4);
    std::memcpy(&alt, &fr.frame[10], 4);
    EXPECT_FLOAT_EQ(x, 100.0f);
    EXPECT_FLOAT_EQ(y, -25.5f);
    EXPECT_FLOAT_EQ(alt, 20.0f);
    EXPECT_EQ(fr.frame[14], static_cast<std::byte>(1));

    EXPECT_EQ(fr.frame[15], static_cast<std::byte>(checkSum(MsgType::TELEMETRY, &fr.frame[2], 13)));
}

TEST(EncodeTelemetryTest, ProducesCorrectFrameWhenDisarmed) {
    TelemetryPL pl{0.0f, 0.0f, 0.0f, 0};
    Frame fr = encodeTelemetry(pl);
    EXPECT_EQ(fr.frame[14], static_cast<std::byte>(0));
}

// Encode Log

TEST(EncodeLogTest, StateChangeLogHasZeroDetail) {
    LogPL pl{LogCode::ST_IDLE, 0};
    Frame fr = encodeLog(pl);

    ASSERT_EQ(fr.size, 5u);
    EXPECT_EQ(fr.frame[0], START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(MsgType::LOG));
    EXPECT_EQ(fr.frame[2], static_cast<std::byte>(LogCode::ST_IDLE));
    EXPECT_EQ(fr.frame[3], static_cast<std::byte>(0));
    EXPECT_EQ(fr.frame[4], static_cast<std::byte>(checkSum(MsgType::LOG, &fr.frame[2], 2)));
}

TEST(EncodeLogTest, SystemErrorLogCarriesDetail) {
    LogPL pl{LogCode::ERR_SYS_UART, 3};
    Frame fr = encodeLog(pl);

    EXPECT_EQ(fr.frame[2], static_cast<std::byte>(LogCode::ERR_SYS_UART));
    EXPECT_EQ(fr.frame[3], static_cast<std::byte>(3));
}

// Parse frame tests

TEST(ParseFrameTest, RoundTripsArmCmd) {
    Frame fr = encodeArm();
    auto result = parseFrame(fr.frame.data(), fr.size);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::ARM_CMD);
    EXPECT_EQ(result->bytes_read, 3u);
    EXPECT_EQ(result->payload.size, 0u);
}

TEST(ParseFrameTest, RoundTripsLandCmd) {
    Frame fr = encodeLand();
    auto result = parseFrame(fr.frame.data(), fr.size);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::LAND_CMD);
    EXPECT_EQ(result->bytes_read, 3u);
}

TEST(ParseFrameTest, RoundTripsGoToCmd) {
    GotoPL original{55.5f, -10.25f};
    Frame fr = encodeGoTo(original);
    auto result = parseFrame(fr.frame.data(), fr.size);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::GOTO_CMD);
    EXPECT_EQ(result->bytes_read, 11u);
    ASSERT_EQ(result->payload.size, 8u);

    GotoPL decoded = decodeGoto(result->payload);
    EXPECT_FLOAT_EQ(decoded.X, original.X);
    EXPECT_FLOAT_EQ(decoded.Y, original.Y);
}

TEST(ParseFrameTest, RoundTripsTelemetry) {
    TelemetryPL original{1.0f, 2.0f, 20.0f, 1};
    Frame fr = encodeTelemetry(original);
    auto result = parseFrame(fr.frame.data(), fr.size);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::TELEMETRY);
    EXPECT_EQ(result->bytes_read, 16u);
    ASSERT_EQ(result->payload.size, 13u);

    TelemetryPL decoded = decodeTelemetry(result->payload);
    EXPECT_FLOAT_EQ(decoded.X, original.X);
    EXPECT_FLOAT_EQ(decoded.Y, original.Y);
    EXPECT_FLOAT_EQ(decoded.alt, original.alt);
    EXPECT_EQ(decoded.armed, original.armed);
}

TEST(ParseFrameTest, RoundTripsLog) {
    LogPL original{LogCode::ERR_GEOFENCE, 0};
    Frame fr = encodeLog(original);
    auto result = parseFrame(fr.frame.data(), fr.size);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::LOG);
    ASSERT_EQ(result->payload.size, 2u);

    LogPL decoded = decodeLog(result->payload);
    EXPECT_EQ(decoded.code, original.code);
    EXPECT_EQ(decoded.detail, original.detail);
}

TEST(ParseFrameTest, RejectsBufferShorterThanMinimumFrame) {
    std::byte buffer[2] = { START_BYTE, static_cast<std::byte>(MsgType::ARM_CMD) };
    auto result = parseFrame(buffer, 2);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, RejectsEmptyBuffer) {
    auto result = parseFrame(nullptr, 0);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, RejectsTruncatedPayloadMessage) {
    GotoPL pl{1.0f, 2.0f};
    Frame fr = encodeGoTo(pl);
    auto result = parseFrame(fr.frame.data(), 6);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, RejectsWrongStartByte) {
    Frame fr = encodeArm();
    fr.frame[0] = std::byte{0x00};
    auto result = parseFrame(fr.frame.data(), fr.size);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, RejectsCorruptedChecksum) {
    GotoPL pl{10.0f, 20.0f};
    Frame fr = encodeGoTo(pl);
    fr.frame[2] ^= std::byte{0x01};
    auto result = parseFrame(fr.frame.data(), fr.size);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, RejectsUnknownMessageType) {
    std::byte buffer[3] = {
        START_BYTE,
        std::byte{0xFE},
        std::byte{0x00}
    };
    auto result = parseFrame(buffer, 3);
    EXPECT_FALSE(result.has_value());
}

TEST(ParseFrameTest, IgnoresTrailingBytesAndReportsCorrectBytesRead) {
    Frame fr = encodeArm();
    std::byte buffer[6];
    std::memcpy(buffer, fr.frame.data(), fr.size);
    buffer[3] = START_BYTE;
    buffer[4] = static_cast<std::byte>(MsgType::LAND_CMD);
    buffer[5] = std::byte{0xFF};

    auto result = parseFrame(buffer, 6);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, MsgType::ARM_CMD);
    EXPECT_EQ(result->bytes_read, 3u);
}

// Decode
TEST(DecodeGotoTest, DecodesHandBuiltPayload) {
    PayloadRaw raw;
    float x = -3.5f, y = 99.0f;
    std::memcpy(&raw.payload[0], &x, 4);
    std::memcpy(&raw.payload[4], &y, 4);
    raw.size = 8;

    GotoPL pl = decodeGoto(raw);
    EXPECT_FLOAT_EQ(pl.X, -3.5f);
    EXPECT_FLOAT_EQ(pl.Y, 99.0f);
}

TEST(DecodeTelemetryTest, DecodesHandBuiltPayload) {
    PayloadRaw raw;
    float x = 1.5f, y = -2.5f, alt = 20.0f;
    std::memcpy(&raw.payload[0], &x, 4);
    std::memcpy(&raw.payload[4], &y, 4);
    std::memcpy(&raw.payload[8], &alt, 4);
    raw.payload[12] = std::byte{1};
    raw.size = 13;

    TelemetryPL pl = decodeTelemetry(raw);
    EXPECT_FLOAT_EQ(pl.X, 1.5f);
    EXPECT_FLOAT_EQ(pl.Y, -2.5f);
    EXPECT_FLOAT_EQ(pl.alt, 20.0f);
    EXPECT_EQ(pl.armed, 1);
}

TEST(DecodeLogTest, DecodesHandBuiltPayload) {
    PayloadRaw raw;
    raw.payload[0] = static_cast<std::byte>(LogCode::CMD_GOTO);
    raw.payload[1] = std::byte{0};
    raw.size = 2;

    LogPL pl = decodeLog(raw);
    EXPECT_EQ(pl.code, LogCode::CMD_GOTO);
    EXPECT_EQ(pl.detail, 0);
}

// Encode to Decode

TEST(EncodeDecodeConsistencyTest, GoToSurvivesRoundTrip) {
    GotoPL original{123.456f, -78.9f};
    Frame fr = encodeGoTo(original);

    PayloadRaw raw;
    std::memcpy(&raw.payload[0], &fr.frame[2], 8);
    raw.size = 8;

    GotoPL decoded = decodeGoto(raw);
    EXPECT_FLOAT_EQ(decoded.X, original.X);
    EXPECT_FLOAT_EQ(decoded.Y, original.Y);
}

TEST(EncodeDecodeConsistencyTest, TelemetrySurvivesRoundTrip) {
    TelemetryPL original{-5.0f, 5.0f, 20.0f, 1};
    Frame fr = encodeTelemetry(original);

    PayloadRaw raw;
    std::memcpy(&raw.payload[0], &fr.frame[2], 13);
    raw.size = 13;

    TelemetryPL decoded = decodeTelemetry(raw);
    EXPECT_FLOAT_EQ(decoded.X, original.X);
    EXPECT_FLOAT_EQ(decoded.Y, original.Y);
    EXPECT_FLOAT_EQ(decoded.alt, original.alt);
    EXPECT_EQ(decoded.armed, original.armed);
}

TEST(EncodeDecodeConsistencyTest, LogSurvivesRoundTrip) {
    LogPL original{LogCode::ERR_CHECKSUM, 0};
    Frame fr = encodeLog(original);

    PayloadRaw raw;
    std::memcpy(&raw.payload[0], &fr.frame[2], 2);
    raw.size = 2;

    LogPL decoded = decodeLog(raw);
    EXPECT_EQ(decoded.code, original.code);
    EXPECT_EQ(decoded.detail, original.detail);
}