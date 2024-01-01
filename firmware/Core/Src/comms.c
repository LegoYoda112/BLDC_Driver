#include "comms.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

// Tx and Rx headers
// + data buffers
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

enum canBitrate can_bitrate = _500Kbs;

bool slcan_open = false;
bool rx_msg_led = false;
int num_sent = 0;

unsigned char __attribute__((section(".bootBlockRAM"))) boot_bits[10];

// Placeholder CAN_ID
uint16_t device_can_id = 2000;

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

    rx_msg_led = true;
    num_sent += 1;
    return 1;
}

void init_and_start_can()
{
    // Set up CAN header
    TxHeader.Identifier = device_can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;

    // TODO: Filter doesn't seem to be working yet
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00;
    sFilterConfig.FilterID2 = 0x01;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    can_set_500kbs();
    
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    // Activate the notification for new data in FIFO0 for FDCAN1
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}

void process_USB_rx(uint8_t *Buf, uint32_t buffer_length)
{
    
    /////// Enter bootloader
    if(Buf[0] == 'B' && Buf[1] == 'T' && Buf[2] == 'L'){
        // Set ram blocks to request bootloader to run
        // boot_bits[0] = 'B';
        // boot_bits[1] = 'T';
        // boot_bits[2] = 'L';
        HAL_NVIC_SystemReset();
    }

    /////// Open interface
    if (Buf[0] == 'O')
    {
        // Check if CAN is already started, ignore if so
        if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_READY)
        {
            return;
        }

        // Else, attempt to start CAN interface
        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        {
            Error_Handler();
        }

        app_state = app_state_active;
        slcan_open = true;
    }

    /////// Close interface
    if (Buf[0] == 'C')
    {
        // Ensure that CAN has already been started
        if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_BUSY)
        {
            return;
        }

        // Else, attempt to close CAN interface
        if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
        {
            Error_Handler();
        }

        app_state = app_state_idle;
        slcan_open = false;
    }

    /////// Set bitrate
    // Only supports 500 and 1000 Kbit/s for now
    if (Buf[0] == 'S')
    {
        volatile int bitrate_code = Buf[1] - '0';

        if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_READY)
        {
            return;
        }

        if (bitrate_code == _500Kbs)
        {
            can_set_500kbs();
        }
        else if (bitrate_code == _1000Kbs)
        {
            can_set_1000kbs();
        }

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /////// Send message
    if (Buf[0] == 't' || Buf[0] == 'T')
    { // 11 bit data frame

        volatile int dlc;
        char id_substring[12];

        if (Buf[0] == 't')
        {
            strncpy(id_substring, Buf + 1, 3);
            TxHeader.IdType = FDCAN_STANDARD_ID;
            dlc = Buf[4] - '0';
        }
        else
        {
            strncpy(id_substring, Buf + 1, 11);
            TxHeader.IdType = FDCAN_EXTENDED_ID;
            dlc = Buf[9] - '0';
            // TODO: do this
        }

        TxHeader.DataLength = FDCAN_DLC_BYTES_1 * dlc;
        TxHeader.Identifier = strtol(id_substring, NULL, 16);

        for (int i = 0; i < dlc; i++)
        {
            char byte_substring[3];
            strncpy(byte_substring, Buf + 1 + 4 + i * 2, 2);
            TxData[i] = strtol(byte_substring, NULL, 16);
        }

        // Fake loopback
        // send_slcan_string(TxHeader, TxData);
        CAN_Transmit_Safe(&TxHeader, TxData);
    }
}

// Set registers for 500_000 kb/s timing
void can_set_500kbs()
{
    hfdcan1.Init.NominalPrescaler = 16;
    hfdcan1.Init.NominalTimeSeg1 = 15;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    can_bitrate = _500Kbs;
}

// Set registers for 1_000_000 kb/s (1Mbit) timing
void can_set_1000kbs()
{
    hfdcan1.Init.NominalPrescaler = 8;
    hfdcan1.Init.NominalTimeSeg1 = 15;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    can_bitrate = _1000Kbs;
}

void process_cmd_rx(uint8_t RxData[])
{
    volatile int cmd = RxData[0];
    printf("");
}

void send_slcan_string(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *RxData)
{
    volatile char tx_string[30];

    // Set message type and ID
    // sprintf is valid here as we will override the termination character later
    if (RxHeader.IdType == FDCAN_STANDARD_ID)
    {
        if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
        {
            tx_string[0] = 'r';
        }
        else if (RxHeader.RxFrameType == FDCAN_DATA_FRAME)
        {
            tx_string[0] = 't';
        }

        sprintf(tx_string + 1, "%03x", RxHeader.Identifier);
    }
    else if (RxHeader.IdType == FDCAN_EXTENDED_ID)
    {
        if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
        {
            tx_string[0] = 'R';
        }
        else if (RxHeader.RxFrameType == FDCAN_DATA_FRAME)
        {
            tx_string[0] = 'T';
        }
        sprintf(tx_string + 1, "%011x", RxHeader.Identifier);
    }

    // Set DLC
    uint32_t dlc = RxHeader.DataLength / FDCAN_DLC_BYTES_1;
    tx_string[4] = dlc + '0';

    for (int i = 0; i < dlc; i++)
    {
        // char byte_substring[3];
        sprintf(tx_string + 5 + i*2, "%02x", RxData[i]);
        // tx_string[5 + i*2] = byte_substring[0];
        // tx_string[6 + i*2] = byte_substring[0];
    }
    
    tx_string[5 + 2*dlc] = '\r';

    CDC_Transmit_FS(tx_string, 5 + 2*dlc + 1);
}

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

    if(slcan_open){
        rx_msg_led = true;
        send_slcan_string(RxHeader, RxData);
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}
