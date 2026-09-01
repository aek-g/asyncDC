#ifndef ASYNCDC_GCS_COMM_H
#define ASYNCDC_GCS_COMM_H
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <windows.h>
#include <vector>
#include <protocol.h>
#include <queue>

namespace gcs_comm {

    // Struct for holding drone telemetry/connection data
    struct DStateGCS {
        // Telemetry
        float X = 0.0f;
        float Y = 0.0f;
        float alt = 0.0f;
        bool armed = false;

        // Telemetry heartbeat bool, used to display gcs/drone connection status
        bool connected = false;
    };

    // Struct for GCS and Drone logs into written out strings
    struct LogStr {
        std::string msg;
        std::chrono::steady_clock::time_point time;
    };

    // Helper function to match LogCode values to string
    std::string logCodeToStr(protocol::LogCode code);

    // Class for managing connection resources
    class Connection {
        // Mutex for state, to prevent race conditions
        mutable std::mutex state_mutex;
        DStateGCS state;

        // Stop signal and thread variable
        std::atomic<bool> running{false};
        std::thread io_thread;

        // Windows port handle
        HANDLE port_handle = INVALID_HANDLE_VALUE;

        // Update loop function
        void update();

        // Function to transmit frame to port
        void transmitFrame(const protocol::Frame& frame);

        // Helper function for log vector
        void pushLog(const std::string& msg);

        // Function to parse the command queue
        void parseCmds();

        // Max size of log vector
        static constexpr std::size_t MAX_LOG_SIZE = 50;
        std::vector<LogStr> logVec;

        // Command mutex and queue to avoid blocking while waiting transmission
        mutable std::mutex cmd_mutex;
        // Max outFrames queue size
        static constexpr std::size_t MAX_OUT_FRAMES = 50;
        std::queue<protocol::Frame> outFrames;

    public:
        // Functions for sending commands
        void sendGoTo(float x, float y);
        void sendArm();
        void sendLand();

        // Getters for UI
        [[nodiscard]] DStateGCS getState() const;
        [[nodiscard]] std::vector<LogStr> getLogs() const;

        // Check if port is open
        [[nodiscard]] bool isOpen() const;

        // Constructor, destructor
        explicit Connection(const std::string& port);
        ~Connection();

        // No copy
        Connection(const Connection&) = delete;
        Connection& operator=(const Connection&) = delete;

#ifdef GCS_UNIT_TEST
        // UNIT TEST LOGIC
        // Without io_thread
        Connection(const std::string& port, bool startIoThread);

        std::size_t testOutFrameCount() const {
            std::lock_guard lock(cmd_mutex);
            return outFrames.size();
        }
        protocol::Frame testPopOutFrame() {
            std::lock_guard lock(cmd_mutex);
            protocol::Frame fr = outFrames.front();
            outFrames.pop();
            return fr;
        }
        void testPushLog(const std::string& msg) { pushLog(msg); }
        std::size_t testLogCount() const {
            std::lock_guard lock(state_mutex);
            return logVec.size();
        }
        std::string testLogAt(std::size_t i) const {
            std::lock_guard lock(state_mutex);
            return logVec.at(i).msg;
        }

        void testSetState(const DStateGCS& s) {
            std::lock_guard lock(state_mutex);
            state = s;
        }
#endif
    };

}

#endif
