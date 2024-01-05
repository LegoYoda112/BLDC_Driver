#ifndef CAN_H
#define CAN_H

#include "main.h"
#include "bootloader.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern uint8_t TxData[8];
extern FDCAN_TxHeaderTypeDef TxHeader;

enum canBitrate {
    _500Kbs = 6,
    _1000Kbs = 8
};

void init_and_start_can();
void can_set_500kbs();
void can_set_1000kbs();

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif