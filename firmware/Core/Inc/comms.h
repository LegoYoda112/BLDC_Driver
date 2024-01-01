#ifndef COMMS_H
#define COMMS_H

#include "main.h"
#include "drive.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData[8];

#define SET_TARGET_POSITION 12

/**
 * @brief Called when a new line of USB data is received
 * 
 * @param Buf Buffer containing bytes
 * @param Len Length of buffer
 * 
 * Buffer will fill until a newline character ('\\n') is sent through the virtual comm port
 */
void process_USB_rx(uint8_t* Buf, uint32_t *Len);

int CAN_Transmit_Safe(FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData);

/**
 * @brief Starts hfdcan1 and activates RX interrupt
 * 
 */
void start_can();

void can_send();

#endif