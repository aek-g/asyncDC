#include <gtest/gtest.h>
#include "gcs_comm.h"
#include <chrono>
#include <numeric>
#include <thread>

using namespace gcs_comm;

// NB: These tests are currently order dependant and meant to run in sequence as a full flight mission test.

namespace {
    // Hardware port - REPLACE WITH YOUR HARDWARE PORT
    constexpr const char* DRONE_PORT = "COM4";

    // Polls condition on drone until true or timeout
    template <typename Pred>
    bool waitFor(Connection& conn, Pred pred, int timeoutMs = 3000) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeoutMs)) {
            if (pred(conn.getState())) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

    // Check log for substring
    bool logContains(Connection& conn, const std::string& substr) {
        for (auto& l : conn.getLogs()) {
            if (l.msg.find(substr) != std::string::npos) return true;
        }
        return false;
    }

    // Polls logContains until true or timeout
    bool waitForLog(Connection& conn, const std::string& substr, int timeoutMs = 2000) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeoutMs)) {
            if (logContains(conn, substr)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }
}

// Test connection and telemetry heartbeat
TEST(IntegrationTest, ConnectsAndReceives) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(conn.isOpen());
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected; }));
}

// Test arm command ascent and command log
TEST(IntegrationTest, ArmCommand) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected; }));

    conn.sendArm();
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.armed; }, 2000));
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.alt >= 19.9f; }, 5000));
    EXPECT_TRUE(waitForLog(conn, "Arm command OK"));
}

// Test geofence rejection (from armed state)
TEST(IntegrationTest, GeofenceReject) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));

    conn.sendGoTo(9999.0f, 9999.0f);
    ASSERT_TRUE(waitForLog(conn, "GEOFENCE"));
}

// Goto moves to target
TEST(IntegrationTest, GoTo) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));

    conn.sendGoTo(3.0f, 4.0f);
    ASSERT_TRUE(waitFor(conn, [](auto s) {
        return std::abs(s.X - 3.0f) < 0.5f && std::abs(s.Y - 4.0f) < 0.5f;
    }, 5000));
}


// Retarget mid flight
TEST(IntegrationTest, Retarget) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));

    conn.sendGoTo(10.0f, 10.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    conn.sendGoTo(2.0f, 2.0f);
    ASSERT_TRUE(waitFor(conn, [](auto s) {
        return std::abs(s.X - 2.0f) < 0.5f && std::abs(s.Y - 2.0f) < 0.5f;
    }, 5000));
}

// Invalid retarget preserves original target
TEST(IntegrationTest, RejectedRetarget) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));

    conn.sendGoTo(3.0f, 0.0f);
    conn.sendGoTo(9999.0f, 9999.0f);
    ASSERT_TRUE(waitForLog(conn, "GEOFENCE"));
    ASSERT_TRUE(waitFor(conn, [](auto s) {
        return std::abs(s.X - 3.0f) < 0.5f && std::abs(s.Y - 0.0f) < 0.5f;
    }, 5000));
}

// Test if telemetry is ~10 Hz
TEST(IntegrationTest, TelemetryTenHz) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));
    conn.sendGoTo(-100.0f, -100.0f);

    int updateCount = 0;
    float lastX = conn.getState().X;
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1000)) {
        float x = conn.getState().X;
        if (x != lastX) {
            updateCount++;
            lastX = x;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    EXPECT_GE(updateCount, 8);
    EXPECT_LE(updateCount, 11);
}

// Test if telemetry survives log burst
TEST(IntegrationTest, TelemetryLogBurst) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));
    conn.sendGoTo(0.0f, 0.0f);

    for (int i = 0; i < 10; ++i) {
        conn.sendGoTo(9999.0f, 9999.0f);
    }

    std::vector<float> samples;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1000)) {
        samples.push_back(conn.getState().X);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    int distinctChanges = 0;
    for (size_t i = 1; i < samples.size(); ++i) {
        if (samples[i] != samples[i - 1]) distinctChanges++;
    }
    EXPECT_GT(distinctChanges, 8);
}

// Land command descends and disarms
TEST(IntegrationTest, Land) {
    Connection conn(DRONE_PORT);
    ASSERT_TRUE(waitFor(conn, [](auto s) { return s.connected && s.armed; }, 8000));

    conn.sendLand();
    ASSERT_TRUE(waitFor(conn, [](auto s) { return !s.armed; }, 10000));
    EXPECT_TRUE(waitForLog(conn, "STATE: DISARMED"));
}
