#ifndef COMMS_H
#define COMMS_H

#include "main.h"
#include "app.h"
#include "stdbool.h"

// extern enum appState;
// extern enum appState app_state;


enum canBitrate {
    _500Kbs = 6,
    _1000Kbs = 8
};

extern enum canBitrate can_bitrate;

extern FDCAN_HandleTypeDef hfdcan1;

extern bool rx_msg_led;

/**
 * @brief Sets up TxHeader, activates RX callback and starts CAN peripheral
 * 
 */
void init_and_start_can();

/**
 * @brief Called when a new line of USB data is received
 * 
 * @param Buf Buffer containing bytes
 * @param Len Length of buffer
 * 
 * Buffer will fill until a newline character ('\\n') is sent through the virtual comm port
 */
void process_USB_rx(uint8_t* Buf, uint32_t buffer_length);

void process_cmd_rx(uint8_t RxData[]);

void can_set_500kbs();
void can_set_1000kbs();

void send_slcan_string(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *RxData);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif