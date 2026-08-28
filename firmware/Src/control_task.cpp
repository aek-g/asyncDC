#include "tasks.h"
#include "dcs_type.h"
#include "drone.h"
#include "protocol.h"
#include "main.h"

namespace {
    // DroneController initiation (default constructor)
    // Only available in this namespace, only control task can interact with it
    drone::DroneController droneInstance;
    drone::DroneState prevState = droneInstance.getState();

    // Logcode for command error results
    protocol::LogCode cmdERROR(drone::RCode result, CmdType cmdType) {
        if (result == drone::RCode::ERR_GEOFENCE) {
            return protocol::LogCode::ERR_GEOFENCE;
        }
        // result == RCode::ERR_INVALID
        switch (cmdType) {
            case CMD_ARM:  return protocol::LogCode::ERR_ARM_REJECTED;
            case CMD_GOTO: return protocol::LogCode::ERR_GOTO_REJECTED;
            case CMD_LAND: return protocol::LogCode::ERR_LAND_REJECTED;
        }
        return protocol::LogCode::UNSET; // unreachable, but keeps compiler happy
    }
    // Logcode for successful command
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
        // Result code initalized
        drone::RCode result;

        // Mutex accessed, command passed, result assigned
        osMutexAcquire(droneStateMutex, osWaitForever);
        switch (cmd.type) {
            case CMD_ARM:  result = droneInstance.arm(); break;
            case CMD_GOTO: result = droneInstance.goTo(cmd.targetX, cmd.targetY); break;
            case CMD_LAND: result = droneInstance.land(); break;
        }
        osMutexRelease(droneStateMutex);

        // Result logging
        if (result == drone::RCode::OK) {
            pushLog(cmdOK(cmd.type));
        } else {
            pushLog(cmdERROR(result, cmd.type));
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

// LED toggle on ARMED/DISARMED state change
static void updateLed(drone::DroneState state) {
    if (state == prevState) return;
    GPIO_PinState ledState = (state != drone::DroneState::DISARMED) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ledState);
    prevState = state;
}

// Task run function
extern "C" void ControlTask_Run(void) {
    // Deltaseconds for update, 0.1 = 100 ticks
    const float deltaSeconds = 0.1f;

    // Task for loop
    for (;;) {
        // Cmd pulling
        Cmd cmd;
        if (osMessageQueueGet(cmdQueue, &cmd, nullptr, 0) == osOK) {
            handleCommand(cmd);
        }

        // Acquire dronestate mutex, call update(), release mutex
        osMutexAcquire(droneStateMutex, osWaitForever);
        droneInstance.update(deltaSeconds);
        drone::DroneState currentState = droneInstance.getState();
        osMutexRelease(droneStateMutex);

        // LED check for state
        updateLed(currentState);

        osDelay(100);
    }
}

// Land function for button press, issues land command to cmdQueue
extern "C" void BtnLand(void) {
    Cmd cmd{CMD_LAND};
    osMessageQueuePut(cmdQueue, &cmd, 0, 0);
}