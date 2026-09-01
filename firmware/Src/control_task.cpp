#include "tasks.h"
#include "dcs_type.h"
#include "drone.h"
#include "protocol.h"
#include "main.h"

namespace {
    // DroneController initiation (default constructor)
    // Only touched directly in this namespace
    drone::DroneController droneInstance;
    drone::DroneState prevState = droneInstance.getState();

    // Blink counter for ARMING/LANDING/GOTO LED
    uint32_t blinkCounter = 0;

    // Log code for command error results
    protocol::LogCode cmdERROR(drone::RCode result, CmdType cmdType) {
        if (result == drone::RCode::ERR_GEOFENCE) {
            return protocol::LogCode::ERR_GEOFENCE;
        }
        switch (cmdType) {
            case CMD_ARM:  return protocol::LogCode::ERR_ARM_REJECTED;
            case CMD_GOTO: return protocol::LogCode::ERR_GOTO_REJECTED;
            case CMD_LAND: return protocol::LogCode::ERR_LAND_REJECTED;
        }
        return protocol::LogCode::UNSET;
    }
    // Log code for successful command
    protocol::LogCode cmdOK(CmdType type) {
        switch (type) {
            case CMD_ARM:  return protocol::LogCode::CMD_ARM;
            case CMD_GOTO: return protocol::LogCode::CMD_GOTO;
            case CMD_LAND: return protocol::LogCode::CMD_LAND;
        }
        return protocol::LogCode::UNSET;
    }

    // Push response log to queue
    void pushLog(protocol::LogCode code) {
        LogMsg msg{static_cast<uint8_t>(code), 0};
        osMessageQueuePut(logQueue, &msg, 0, 0);
    }
    // Command handle logic
    void handleCommand(const Cmd& cmd) {
        // Result code initialized
        drone::RCode result;

        // Mutex accessed, command passed, result assigned
        osMutexAcquire(droneStateMutex, osWaitForever);
        switch (cmd.type) {
            case CMD_ARM:  result = droneInstance.arm(); break;
            case CMD_GOTO: result = droneInstance.goTo(cmd.targetX, cmd.targetY); break;
            case CMD_LAND: result = droneInstance.land(); break;
            default: result = drone::RCode::ERR_INVALID;
        }
        osMutexRelease(droneStateMutex);

        // Result logging
        if (result == drone::RCode::OK) {
            pushLog(cmdOK(cmd.type));
        } else {
            pushLog(cmdERROR(result, cmd.type));
        }
    }

    // Function for indicating state with LED
    void updateLed(drone::DroneState current) {

        GPIO_PinState ledState;
        // If ARMING/ GOTO/ LANDING then blink rapidly
        if (current == drone::DroneState::ARMING || current == drone::DroneState::GOTO || current == drone::DroneState::LANDING) {
            blinkCounter++;
            ledState = ((blinkCounter % 5) < 3) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        } else {
            // If DISARMED then LED off, if Idle then LED solid
            blinkCounter = 0;
            ledState = (current != drone::DroneState::DISARMED) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        }
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ledState);
    }

    // Check for state change, update LED
    void stateChanged(drone::DroneState current) {
        if (current != prevState) {

            // Update previous state
            prevState = current;

            protocol::LogCode changeCode;
            switch (current) {
                case drone::DroneState::DISARMED: changeCode = protocol::LogCode::ST_DISARMED; break;
                case drone::DroneState::ARMING:   changeCode = protocol::LogCode::ST_ARMING; break;
                case drone::DroneState::IDLE:     changeCode = protocol::LogCode::ST_IDLE; break;
                case drone::DroneState::GOTO:     changeCode = protocol::LogCode::ST_GOTO; break;
                case drone::DroneState::LANDING:  changeCode = protocol::LogCode::ST_LANDING; break;
                default: changeCode = protocol::LogCode::UNSET; break;
            }
            pushLog(changeCode);
        }
    }
}

// Getter functions for namespace/mutex locked DroneController (for telemetry)
drone::Position ControlTask_GetPosition() {
    osMutexAcquire(droneStateMutex, osWaitForever);
    drone::Position pos = droneInstance.getPosition();
    osMutexRelease(droneStateMutex);
    return pos;
}

drone::DroneState ControlTask_GetState() {
    osMutexAcquire(droneStateMutex, osWaitForever);
    drone::DroneState state = droneInstance.getState();
    osMutexRelease(droneStateMutex);
    return state;
}

// Task run function
extern "C" void ControlTask_Run(void) {
    // Delta seconds for update, 0.1 = 100 ticks
    const float deltaSeconds = 0.1f;

    // Task for loop
    for (;;) {
        // Cmd pulling
        Cmd cmd;
        if (osMessageQueueGet(cmdQueue, &cmd, nullptr, 0) == osOK) {
            handleCommand(cmd);
        }
        // Acquire drone state mutex, get state
        osMutexAcquire(droneStateMutex, osWaitForever);
        drone::DroneState currentState = droneInstance.getState();
        osMutexRelease(droneStateMutex);
        // State change check and logging (for edge case where goto target is less distance than one update iteration)
        stateChanged(currentState);

        // Acquire drone state mutex, update and get state again
        osMutexAcquire(droneStateMutex, osWaitForever);
        droneInstance.update(deltaSeconds);
        currentState = droneInstance.getState();
        osMutexRelease(droneStateMutex);

        // State change check and logging after update
        stateChanged(currentState);
        // Update LED based on state
        updateLed(currentState);

        osDelay(100);
    }
}

// Land function for button press, issues land command to cmdQueue
extern "C" void BtnLand(void) {
    Cmd cmd{CMD_LAND};
    osMessageQueuePut(cmdQueue, &cmd, 0, 0);
}