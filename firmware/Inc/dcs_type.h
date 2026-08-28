#ifndef FIRMWARE_DCS_TYPE_H
#define FIRMWARE_DCS_TYPE_H
#include <stdint.h>
typedef enum {
    CMD_ARM,
    CMD_GOTO,
    CMD_LAND,
} CmdType;

typedef struct {
    CmdType type;
    float targetX;
    float targetY;
} Cmd;

typedef struct {
    uint8_t code;
    uint8_t detail;
} LogMsg;

#endif //FIRMWARE_DCS_TYPE_H
