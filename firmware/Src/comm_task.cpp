#include "protocol.h"
#include "drone.h"
#include "tasks.h"
#include "dcs_type.h"
#include "main.h"
#include <cstring>

// UART handle (defined elsewhere)
extern UART_HandleTypeDef huart2;

namespace {
    // Ring buffer for UART ISR
    constexpr std::size_t RING_BUF_SIZE = 128;
    volatile std::byte ringBuffer[RING_BUF_SIZE];
    // Write and read indexes for populating and reading from the buffer
    volatile std::size_t writeIndex = 0;
    volatile std::size_t readIndex = 0;
    // Byte variable that the HAL UART handler will write into
    volatile uint8_t uartByte = 0;

    // Buffer size for parsing
    constexpr std::size_t PARSE_BUF_SIZE = 128;
    // Buffer for bytes to be parsed
    std::byte parseBuffer[PARSE_BUF_SIZE];
    std::size_t parseLen = 0;

    // Drain the ISR ring buffer into the parse buffer
    void drainRing() {
        // Process while buffer not empty
        while (readIndex != writeIndex) {
            // Parse buffer overflow guard
            if (parseLen < PARSE_BUF_SIZE) {
                // Copy next byte from ring buffer to parse buffer
                parseBuffer[parseLen++] = ringBuffer[readIndex];
            }
            // Increase ring buffer read index
            readIndex = (readIndex + 1) % RING_BUF_SIZE;
        }
    }

    // Parse the bytes in the parse buffer
    void parseBuf() {
        // If START_BYTE not found as first, try finding next START_BYTE
        if (parseLen > 0 && parseBuffer[0] != protocol::START_BYTE) {
            std::size_t skip = 1;
            while (skip < parseLen && parseBuffer[skip] != protocol::START_BYTE) {
                ++skip;
            }
            std::memmove(parseBuffer, parseBuffer + skip, parseLen - skip);
            parseLen -= skip;
            // If no START_BYTE found
            if (parseLen == 0) return;
        }
        // Parse frame into result variable
        auto result = protocol::parseFrame(parseBuffer, parseLen);

        if (!result) {
            // Reset buffer if full without a valid frame
            if (parseLen >= PARSE_BUF_SIZE) {
                parseLen = 0;
            }
            return;
        }
        // Checksum error handling
        if (result->type == protocol::MsgType::UNSET) {
            LogMsg msg{static_cast<uint8_t>(protocol::LogCode::ERR_CHECKSUM), 0};
            osMessageQueuePut(logQueue, &msg, 0, 0);
            std::memmove(parseBuffer, parseBuffer + result->bytes_read, parseLen - result->bytes_read);
            parseLen -= result->bytes_read;
            return;
        }

        // Initialize command
        Cmd cmd{};
        bool validCmd = true;

        // Switch case for command type, validCmd = false if not ARM/LAND/GOTO
        switch (result->type) {
            case protocol::MsgType::ARM_CMD:
                cmd.type = CMD_ARM;
                break;
            case protocol::MsgType::LAND_CMD:
                cmd.type = CMD_LAND;
                break;
            case protocol::MsgType::GOTO_CMD: {
                protocol::GotoPL pl = protocol::decodeGoto(result->payload);
                cmd.type = CMD_GOTO;
                cmd.targetX = pl.X;
                cmd.targetY = pl.Y;
                break;
            }
            default:
                validCmd = false;
                break;
        }

        // Queue command if valid
        if (validCmd) {
            osMessageQueuePut(cmdQueue, &cmd, 0, 0);
        }

        // Move unparsed bytes to front of buffer
        std::memmove(parseBuffer, parseBuffer + result->bytes_read, parseLen - result->bytes_read);
        // Update buffer length
        parseLen -= result->bytes_read;
    }

    // Transmit frame over UART, timeout 100ms
    void transmitFrame(const protocol::Frame& frame) {
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t*>(const_cast<std::byte*>(frame.frame.data())),
                           frame.size, 100);
        // Check for UART transmit errors
        if (status != HAL_OK) {
            LogMsg msg{static_cast<uint8_t>(protocol::LogCode::ERR_SYS_UART), static_cast<uint8_t>(status)};
            osMessageQueuePut(logQueue, &msg, 0, 0);
        }
    }

    // Construct and transmit telemetry payload
    void sendTelemetry() {
        // Get position and state
        drone::Position pos = ControlTask_GetPosition();
        drone::DroneState state = ControlTask_GetState();

        // Populate payload
        protocol::TelemetryPL tel{};
        tel.X = pos.X;
        tel.Y = pos.Y;
        tel.alt = pos.alt;
        tel.armed = (state != drone::DroneState::DISARMED) ? 1 : 0;

        // Encode and transmit
        transmitFrame(protocol::encodeTelemetry(tel));
    }

    // Transmit pending log messages
    void sendLogs() {
        LogMsg msg;
        // Get log message from queue
        while (osMessageQueueGet(logQueue, &msg, nullptr, 0) == osOK) {
            // Convert to payload, encode and transmit
            protocol::LogPL log{static_cast<protocol::LogCode>(msg.code), msg.detail};
            transmitFrame(protocol::encodeLog(log));
        }
    }
}

// CommTask run function
extern "C" void CommTask_Run(void) {
    // Start UART RX with interrupt for received byte
    HAL_UART_Receive_IT(&huart2, const_cast<uint8_t*>(&uartByte), 1);

    // Main loop
    for (;;) {
        drainRing();
        parseBuf();
        sendTelemetry();
        sendLogs();
        osDelay(100);
    }
}

// Handle received byte from UART interrupt
extern "C" void CommTask_OnByte(void) {
    // Read received UART byte
    auto byte = static_cast<std::byte>(uartByte);
    // Calculate next write index
    std::size_t nextWrite = (writeIndex + 1) % RING_BUF_SIZE;
    // Guard for full ring buffer
    if (nextWrite != readIndex) {
        // Store byte in ring buffer
        ringBuffer[writeIndex] = byte;
        writeIndex = nextWrite;
    }
    // Start UART RX for next byte
    HAL_UART_Receive_IT(&huart2, const_cast<uint8_t*>(&uartByte), 1);
}

// Handle UART error (log and start UART RX again for next byte)
extern "C" void CommTask_OnUartError(void) {
    // 32 bit error code from HAL
    uint32_t errorCode = HAL_UART_GetError(&huart2);
    // 32 bit error code truncated to fit into 1 detail byte (lossless), LogMsg created
    LogMsg msg{static_cast<uint8_t>(protocol::LogCode::ERR_SYS_UART), static_cast<uint8_t>(errorCode)};
    // LogMsg to queue
    osMessageQueuePut(logQueue, &msg, 0, 0);
    // Start UART RX for next byte
    HAL_UART_Receive_IT(&huart2, const_cast<uint8_t*>(&uartByte), 1);
}