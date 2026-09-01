#include <gtest/gtest.h>
#include "gcs_comm.h"
#include <cstring>

using namespace gcs_comm;

// Test connection init
namespace {
    Connection makeTestConnection() {
        return Connection("COM_NONEXISTENT_TEST", false);
    }
}

// Connection state

// Test connection .isOpen() with nonexistant port
TEST(ConnectionTest, IsOpenFalse) {
    auto conn = makeTestConnection();
    EXPECT_FALSE(conn.isOpen());
}

// Check default state of nonexistent port test connection
TEST(ConnectionTest, DefaultState) {
    auto conn = makeTestConnection();
    auto state = conn.getState();
    EXPECT_FLOAT_EQ(state.X, 0.0f);
    EXPECT_FLOAT_EQ(state.Y, 0.0f);
    EXPECT_FLOAT_EQ(state.alt, 0.0f);
    EXPECT_FALSE(state.armed);
    EXPECT_FALSE(state.connected);
}

// Logs empty on nonexistent port test connection
TEST(ConnectionTest, GetLogsEmpty) {
    auto conn = makeTestConnection();
    EXPECT_TRUE(conn.getLogs().empty());
}

// Change state values, test getter match
TEST(ConnectionTest, GetStateChanged) {
    auto conn = makeTestConnection();
    DStateGCS s{5.0f, -3.0f, 20.0f, true, true};
    conn.testSetState(s);
    auto state = conn.getState();
    EXPECT_FLOAT_EQ(state.X, 5.0f);
    EXPECT_FLOAT_EQ(state.Y, -3.0f);
    EXPECT_FLOAT_EQ(state.alt, 20.0f);
    EXPECT_TRUE(state.armed);
    EXPECT_TRUE(state.connected);
}

// Test that getstate returns copy not refrence
TEST(ConnectionTest, GetStateCopy) {
    auto conn = makeTestConnection();
    DStateGCS s{1.0f, 1.0f, 1.0f, true, true};
    conn.testSetState(s);

    auto stateCopy = conn.getState();
    stateCopy.alt = 999.0f;

    auto stateAgain = conn.getState();
    EXPECT_NE(stateAgain.alt, 999.0f);
}

// Send command tests

// Test sendArm
TEST(ConnectionSendTest, SendArm) {
    auto conn = makeTestConnection();
    conn.sendArm();
    ASSERT_EQ(conn.testOutFrameCount(), 1u);
    auto fr = conn.testPopOutFrame();
    EXPECT_EQ(fr.frame[0], protocol::START_BYTE);
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(protocol::MsgType::ARM_CMD));
    EXPECT_EQ(fr.size, 3u);
}

// Test sendLand
TEST(ConnectionSendTest, SendLand) {
    auto conn = makeTestConnection();
    conn.sendLand();
    ASSERT_EQ(conn.testOutFrameCount(), 1u);
    auto fr = conn.testPopOutFrame();
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(protocol::MsgType::LAND_CMD));
    EXPECT_EQ(fr.size, 3u);
}

// Test sendGoTo
TEST(ConnectionSendTest, SendGoTo) {
    auto conn = makeTestConnection();
    conn.sendGoTo(12.5f, -7.25f);
    ASSERT_EQ(conn.testOutFrameCount(), 1u);
    auto fr = conn.testPopOutFrame();
    EXPECT_EQ(fr.frame[1], static_cast<std::byte>(protocol::MsgType::GOTO_CMD));
    EXPECT_EQ(fr.size, 11u);

    float x, y;
    std::memcpy(&x, &fr.frame[2], 4);
    std::memcpy(&y, &fr.frame[6], 4);
    EXPECT_FLOAT_EQ(x, 12.5f);
    EXPECT_FLOAT_EQ(y, -7.25f);
}

// Test sendGoTo with negative values
TEST(ConnectionSendTest, SendGoToNegative) {
    auto conn = makeTestConnection();
    conn.sendGoTo(-15.0f, -5.0f);
    auto fr = conn.testPopOutFrame();
    float x, y;
    std::memcpy(&x, &fr.frame[2], 4);
    std::memcpy(&y, &fr.frame[6], 4);
    EXPECT_FLOAT_EQ(x, -15.0f);
    EXPECT_FLOAT_EQ(y, -5.0f);
}

// Test arm, goto and land in sequence
TEST(ConnectionSendTest, MultipleSends) {
    auto conn = makeTestConnection();
    conn.sendArm();
    conn.sendGoTo(1.0f, 2.0f);
    conn.sendLand();
    ASSERT_EQ(conn.testOutFrameCount(), 3u);
    EXPECT_EQ(conn.testPopOutFrame().frame[1], static_cast<std::byte>(protocol::MsgType::ARM_CMD));
    EXPECT_EQ(conn.testPopOutFrame().frame[1], static_cast<std::byte>(protocol::MsgType::GOTO_CMD));
    EXPECT_EQ(conn.testPopOutFrame().frame[1], static_cast<std::byte>(protocol::MsgType::LAND_CMD));
}

// pushLog

// Test pushlog append
TEST(ConnectionLogTest, AppendsMessage) {
    auto conn = makeTestConnection();
    conn.testPushLog("Test string");
    ASSERT_EQ(conn.testLogCount(), 1u);
    EXPECT_EQ(conn.testLogAt(0), "Test string");
}

// Test pushlog order
TEST(ConnectionLogTest, Order) {
    auto conn = makeTestConnection();
    conn.testPushLog("first");
    conn.testPushLog("second");
    conn.testPushLog("third");
    ASSERT_EQ(conn.testLogCount(), 3u);
    EXPECT_EQ(conn.testLogAt(0), "first");
    EXPECT_EQ(conn.testLogAt(1), "second");
    EXPECT_EQ(conn.testLogAt(2), "third");
}

// Test log overflow for removing oldest and reordering
TEST(ConnectionLogTest, Overflow) {
    auto conn = makeTestConnection();
    for (int i = 0; i < 50; ++i) conn.testPushLog("log " + std::to_string(i));
    ASSERT_EQ(conn.testLogCount(), 50u);
    EXPECT_EQ(conn.testLogAt(0), "log 0");

    conn.testPushLog("log 50");
    ASSERT_EQ(conn.testLogCount(), 50u);
    EXPECT_EQ(conn.testLogAt(0), "log 1");
    EXPECT_EQ(conn.testLogAt(49), "log 50");
}

// Test repeated log overflow
TEST(ConnectionLogTest, RepeatedOverflow) {
    auto conn = makeTestConnection();
    for (int i = 0; i < 100; ++i) conn.testPushLog("log " + std::to_string(i));
    ASSERT_EQ(conn.testLogCount(), 50u);
    EXPECT_EQ(conn.testLogAt(0), "log 50");
    EXPECT_EQ(conn.testLogAt(49), "log 99");
}

// outFrame queue commands overflow
TEST(ConnectionSendTest, QueueOverflow) {
    auto conn = makeTestConnection();
    for (int i = 0; i < 50; ++i) conn.sendArm();
    ASSERT_EQ(conn.testOutFrameCount(), 50u);

    conn.sendArm();
    EXPECT_EQ(conn.testOutFrameCount(), 50u);

    ASSERT_EQ(conn.testLogCount(), 1u);
    EXPECT_EQ(conn.testLogAt(0), "[GCS] Command queue full - dropping ARM command");
}

// Overflow with land command, oldest should be dropped in favour of LAND
TEST(ConnectionSendTest, LandQueueOverflow) {
    auto conn = makeTestConnection();
    for (int i = 0; i < 50; ++i) conn.sendArm();
    ASSERT_EQ(conn.testOutFrameCount(), 50u);

    conn.sendLand();
    ASSERT_EQ(conn.testOutFrameCount(), 50u);

    protocol::Frame last;
    for (std::size_t i = 0; i < 50; ++i) last = conn.testPopOutFrame();
    EXPECT_EQ(last.frame[1], static_cast<std::byte>(protocol::MsgType::LAND_CMD));
}

// Test invalid port on io_thread
TEST(ConnectionThreadTest, StartStop) {
    Connection conn("COM_NONEXISTENT_TEST", true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_FALSE(conn.isOpen());
    EXPECT_FALSE(conn.getState().connected);
}

// Test destructor non hanging/blocking
TEST(ConnectionThreadTest, Destructor) {
    auto start = std::chrono::steady_clock::now();
    {
        Connection conn("COM_NONEXISTENT_TEST", true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 1000);
}