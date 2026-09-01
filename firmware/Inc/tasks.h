#ifndef TASKS_H
#define TASKS_H
#include "cmsis_os.h"
extern osMutexId_t droneStateMutex;
extern osMessageQueueId_t cmdQueue;
extern osMessageQueueId_t logQueue;

#ifdef __cplusplus
extern "C" {
#endif
void ControlTask_Run(void);
void CommTask_Run(void);
void BtnLand(void);
void CommTask_OnByte(void);
void CommTask_OnUartError(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "drone.h"
drone::Position ControlTask_GetPosition();
drone::DroneState ControlTask_GetState();
#endif
#endif