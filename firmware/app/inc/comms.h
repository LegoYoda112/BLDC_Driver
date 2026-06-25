#pragma once

#include "stm32g4xx_hal.h"
#include "main.h"
#include "fdcan.h"
#include "usbd_cdc_if.h"

namespace comms
{

void initialize(FDCAN_HandleTypeDef*);

void set_device_id(uint16_t);
uint16_t get_device_id();

int can_transmit(FDCAN_TxHeaderTypeDef*, uint8_t*);
void can_receive(FDCAN_RxHeaderTypeDef*, uint8_t*);

void send_slcan_string(FDCAN_RxHeaderTypeDef, uint8_t*);
void slcan_usb_rx(uint8_t*, uint32_t);

FDCAN_RxHeaderTypeDef convert_tx_header(FDCAN_TxHeaderTypeDef*);

};