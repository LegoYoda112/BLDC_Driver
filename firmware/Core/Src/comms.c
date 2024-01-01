#include "comms.h"

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

void process_USB_rx(uint8_t* Buf, uint32_t *Len){
    // process_cmd_RX(Buf);

    // char intStr[12];
    // sprintf(intStr, "%d\r\n", target_encoder_value);
    // CDC_Transmit_FS((uint8_t*)intStr, strlen(intStr));
}

int CAN_Transmit_Safe(FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData)
{ 

  // If we have filled up the mailboxes, return early, CAN is not connected
  if(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0){
    // drive_state == drive_state_idle;
    return 0;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, TxHeader, TxData) != HAL_OK)
  {
    Error_Handler();
  }

  return 1;
}

// void process_cmd_RX(uint8_t RxData[]){
//     int cmd = RxData[0];

//     int a = RxData[2];
//     int b = RxData[3];
//     int c = RxData[4];

//     // Set 
//     if(cmd == SET_TARGET_POSITION){
//         volatile int target_position_encoder_counts = 
//             (RxData[1] << 0) |
//             (RxData[2] << 8) |
//             (RxData[3] << 16);

//         target_encoder_value = target_position_encoder_counts;

//         printf("123");
//     }
// }

void start_can(){
    // Set up CAN header
    TxHeader.Identifier = 10;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;

    // Activate the notification for new data in FIFO0 for FDCAN1
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

// void can_send(){
//     TxData[0] = 1;
//     if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
//     {
//       Error_Handler();
//     }
// }


// FDCAN1 Callback
// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }


    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}