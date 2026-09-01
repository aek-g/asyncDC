
#include "protocol.h"
#include "gcs_comm.h"
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>

namespace gcs_comm {

    // Function to transmit frame to port
    void Connection::transmitFrame(const protocol::Frame& frame) {
        // Port connection check
        if (port_handle == INVALID_HANDLE_VALUE) {
            return;
        }

        // Write to port, keep as bool
        DWORD bytesWritten = 0;
        BOOL result = WriteFile(port_handle, frame.frame.data(), static_cast<DWORD>(frame.size), &bytesWritten,nullptr);

        // If no result or less bytes written than frame size push write error into logs
        if (!result || bytesWritten != frame.size) {
            DWORD err = GetLastError();
            pushLog("[GCS] Write failed: " + std::to_string(err));
        }
    }

    // Send GoTo to transmit
    void Connection::sendGoTo(float x, float y) {
        // Lock cmd queue mutex, encode GoTo and push
        std::lock_guard lock(cmd_mutex);
        if (outFrames.size() >= MAX_OUT_FRAMES) {
            pushLog("[GCS] Command queue full - dropping GoTo command");
            return;
        }
        outFrames.push(protocol::encodeGoTo({x, y}));
    }

    // Send ARM to transmit
    void Connection::sendArm() {
        // Lock cmd queue mutex, encode Arm and push
        std::lock_guard lock(cmd_mutex);
        if (outFrames.size() >= MAX_OUT_FRAMES) {
            pushLog("[GCS] Command queue full - dropping ARM command");
            return;
        }
        outFrames.push(protocol::encodeArm());
    }

    // Send LAND to transmit
    void Connection::sendLand() {
        // Lock cmd queue mutex, encode Land and push
        std::lock_guard lock(cmd_mutex);
        if (outFrames.size() >= MAX_OUT_FRAMES) {
            // Safety critical for LAND to be queued correctly
            pushLog("[GCS] (LAND) Command queue full - dropping oldest");
            outFrames.pop();
        }
        outFrames.push(protocol::encodeLand());
    }

    // Getters for UI
    DStateGCS Connection::getState() const {
        std::lock_guard lock(state_mutex);
        return state;
    }

    std::vector<LogStr> Connection::getLogs() const {
        std::lock_guard lock(state_mutex);
        return logVec;
    }

    // Helper function for log vector
    void Connection::pushLog(const std::string& msg) {
        // Mutex lock, push new log
        std::lock_guard lock(state_mutex);
        logVec.push_back({msg, std::chrono::steady_clock::now()});

        // If log vector full then drop oldest
        if (logVec.size() > MAX_LOG_SIZE) {
            logVec.erase(logVec.begin());
        }
    }

    // Helper function to turn LogCode into string
    std::string logCodeToStr(protocol::LogCode code) {
        switch (code) {
            // State change cases
            case protocol::LogCode::ST_DISARMED: return "[DRONE] STATE: DISARMED";
            case protocol::LogCode::ST_ARMING: return "[DRONE] STATE: ARMING";
            case protocol::LogCode::ST_IDLE: return "[DRONE] STATE: IDLE";
            case protocol::LogCode::ST_GOTO: return "[DRONE] STATE: GOTO";
            case protocol::LogCode::ST_LANDING: return "[DRONE] STATE: LANDING";
            // Command OK
            case protocol::LogCode::CMD_ARM: return "[DRONE] Arm command OK";
            case protocol::LogCode::CMD_GOTO: return "[DRONE] GOTO command OK";
            case protocol::LogCode::CMD_LAND: return "[DRONE] Land command OK";
            // ERROR codes
            case protocol::LogCode::ERR_CHECKSUM: return "[DRONE] ERROR: Checksum error";
            case protocol::LogCode::ERR_INVALID_CMD: return "[DRONE] ERROR: Invalid command";
            case protocol::LogCode::ERR_ARM_REJECTED: return "[DRONE] ERROR: ARM rejected";
            case protocol::LogCode::ERR_GOTO_REJECTED: return "[DRONE] ERROR: GOTO rejected";
            case protocol::LogCode::ERR_LAND_REJECTED: return "[DRONE] ERROR: LAND rejected";
            case protocol::LogCode::ERR_GEOFENCE: return "[DRONE] ERROR: Target outside GEOFENCE";
            case protocol::LogCode::ERR_SYS_UART: return "[DRONE] ERROR: System UART error";
            // UNSET and default
            case protocol::LogCode::UNSET: return "[DRONE] UNSET log code";
            default: return "[DRONE] UNKNOWN code";
        }
    }

    // Check if port is open
    bool Connection::isOpen() const {
        return port_handle != INVALID_HANDLE_VALUE;
    }

    // Parse command queue
    void Connection::parseCmds() {
        while (true) {
            protocol::Frame fr;
            // Scope for the mutex lock
            {
                std::lock_guard lock(cmd_mutex);
                if (outFrames.empty()) break;

                // Read and pop frame from the queue
                fr = outFrames.front();
                outFrames.pop();
            }
            // Transmit frame
            transmitFrame(fr);
        }
    }

    // Update thread function
    void Connection::update() {
        // Dynamic RX buffer for parsing
        static constexpr std::size_t MAX_PARSE_BUFFER = 512;
        std::vector<std::byte> parseBuffer;
        // Last telemetry time (for connection status)
        auto lastTelemetry = std::chrono::steady_clock::now();

        // While thread is running
        while (running) {
            parseCmds();
            std::byte readBuffer[64];
            DWORD bytesRead = 0;
            BOOL result = ReadFile(port_handle, readBuffer, sizeof(readBuffer), &bytesRead, nullptr);

            if (result && bytesRead > 0) {
                parseBuffer.insert(parseBuffer.end(), readBuffer, readBuffer + bytesRead);
            }

            // Check for START_BYTE, if not found skip forward
            if (!parseBuffer.empty() && parseBuffer[0] != protocol::START_BYTE) {
                auto it = std::find(parseBuffer.begin() + 1, parseBuffer.end(), protocol::START_BYTE);
                parseBuffer.erase(parseBuffer.begin(), it);
            }

            // Parse available frames
            while (true) {
                // Parse frame
                auto pResult = protocol::parseFrame(parseBuffer.data(), parseBuffer.size());
                // Break if no result
                if (!pResult) {
                    break;
                }
                // Checksum error handling
                if (pResult->type == protocol::MsgType::UNSET) {
                    pushLog("[GCS] ERROR: Checksum error");
                    parseBuffer.erase(parseBuffer.begin(), parseBuffer.begin() + static_cast<std::ptrdiff_t>(pResult->bytes_read));
                    continue;
                }

                // Telemetry and Log handling
                if (pResult->type == protocol::MsgType::TELEMETRY) {
                    //Decode, lock mutex, and update telemetry fields
                    protocol::TelemetryPL tel = protocol::decodeTelemetry(pResult->payload);
                    std::lock_guard lock(state_mutex);
                    state.X = tel.X;
                    state.Y = tel.Y;
                    state.alt = tel.alt;
                    state.armed = (tel.armed != 0);
                    state.connected = true;
                    // Update last telemetry time
                    lastTelemetry = std::chrono::steady_clock::now();
                } else if (pResult->type == protocol::MsgType::LOG) {
                    // Log payload construction
                    protocol::LogPL log = protocol::decodeLog(pResult->payload);
                    // Push log into log vector
                    pushLog(logCodeToStr(log.code) + (log.detail ? " , detail: " + std::to_string(log.detail) + ")" : ""));
                }
                // Erase bytes read from buffer
                parseBuffer.erase(parseBuffer.begin(), parseBuffer.begin() + static_cast<std::ptrdiff_t>(pResult->bytes_read));
            }

            // Check parsebuffer for overflow
            if (parseBuffer.size() > MAX_PARSE_BUFFER) {
                pushLog("[GCS] Parse buffer overflow");
                parseBuffer.clear();
            }
            // Telemetry heartbeat check for connection status (<500ms between telemetry)
            auto now = std::chrono::steady_clock::now();
            auto sinceLastTelemetry = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTelemetry).count();
            if (sinceLastTelemetry > 500) {
                std::lock_guard lock(state_mutex);
                state.connected = false;
            }
        }



    }

    // Constructor, destructor
    Connection::Connection(const std::string& port) {
        // Full port string
        std::string comport = R"(\\.\)" + port;

        // Opening the port with Win32 API
        port_handle = CreateFileA(comport.c_str(),GENERIC_READ | GENERIC_WRITE,0,
            nullptr, OPEN_EXISTING, 0, nullptr);
        if (port_handle == INVALID_HANDLE_VALUE) {
            return;
        }

        // Default DCB struct
        DCB dcb_struct = {0};
        dcb_struct.DCBlength = sizeof(dcb_struct);
        GetCommState(port_handle, &dcb_struct);

        // Baud rate to 115200 (default)
        dcb_struct.BaudRate = CBR_115200;

        // Byte size to 8 bits
        dcb_struct.ByteSize = 8;

        // One stop bit, but no parity bit (checksum integrated in protocol)
        dcb_struct.StopBits = ONESTOPBIT;
        dcb_struct.Parity = NOPARITY;

        SetCommState(port_handle, &dcb_struct);

        // Timeouts
        COMMTIMEOUTS timeouts = {0};
        // Read timeouts 50 ms
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 0;

        // Write timeout (10*bytes) + 50 ms
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(port_handle, &timeouts);

        // Clear port RX/TX
        PurgeComm(port_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

        // Start update thread
        running = true;
        io_thread = std::thread(&Connection::update, this);
    }

#ifdef GCS_UNIT_TEST
    // Constructor for unit testing
    Connection::Connection(const std::string& port, bool startIoThread) {
        std::string comport = R"(\\.\)" + port;
        port_handle = CreateFileA(comport.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
            nullptr, OPEN_EXISTING, 0, nullptr);
        if (port_handle != INVALID_HANDLE_VALUE) {
            DCB dcb_struct = {0};
            dcb_struct.DCBlength = sizeof(dcb_struct);
            GetCommState(port_handle, &dcb_struct);
            dcb_struct.BaudRate = CBR_115200;
            dcb_struct.ByteSize = 8;
            dcb_struct.StopBits = ONESTOPBIT;
            dcb_struct.Parity = NOPARITY;
            SetCommState(port_handle, &dcb_struct);

            COMMTIMEOUTS timeouts = {0};
            timeouts.ReadIntervalTimeout = 50;
            timeouts.ReadTotalTimeoutConstant = 50;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 50;
            timeouts.WriteTotalTimeoutMultiplier = 10;
            SetCommTimeouts(port_handle, &timeouts);

            PurgeComm(port_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
        }

        if (startIoThread && port_handle != INVALID_HANDLE_VALUE) {
            running = true;
            io_thread = std::thread(&Connection::update, this);
        }
    }
#endif

    // Destructor
    Connection::~Connection() {
        // Thread running to false
        running = false;
        // Check if thread joinable
        if (io_thread.joinable()) {
            io_thread.join();
        }
        // Close port handle
        if (port_handle != INVALID_HANDLE_VALUE) {
            CloseHandle(port_handle);
        }
    }
}
