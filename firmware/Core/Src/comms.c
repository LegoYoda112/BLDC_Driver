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
uint16_t device_can_id = 420;


int CAN_Transmit_Safe(FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData)
{ 
  if(slcan_open){
    send_slcan_string(convert_tx_header(TxHeader), TxData);
  }

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

void CAN_Transmit_Array(uint8_t *TxData, uint8_t length)
{
  TxHeader.DataLength = FDCAN_DLC_BYTES_1 * length;
  TxHeader.Identifier = device_can_id;

  CAN_Transmit_Safe(&TxHeader, TxData);
}

void CAN_Transmit_Bool(bool value)
{
  uint8_t array[] = {value};
  CAN_Transmit_Array(array, 1);
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
        boot_bits[0] = 'B';
        boot_bits[1] = 'T';
        boot_bits[2] = 'L';
        HAL_NVIC_SystemReset();
    }

    /////// Open interface
    if (Buf[0] == 'O')
    {
        // Check if CAN is already started
        if (HAL_FDCAN_GetState(&hfdcan1) == HAL_FDCAN_STATE_READY)
        {
            // Else, attempt to start CAN interface
            if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
            {
                Error_Handler();
            }
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

        RxHeader = convert_tx_header(&TxHeader);
        handleRX(RxHeader, TxData);
        slcan_open = false;
        CAN_Transmit_Safe(&TxHeader, TxData);
        slcan_open = true;
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








// Handle CAN RX
void handleRX(FDCAN_RxHeaderTypeDef RxHeader, uint8_t RxData[])
{
  volatile int msg_type = RxData[0];

  if(RxHeader.Identifier == 0){ // Catch system messages
    // handle_system_RX(RxHeader, RxData);
  } else if(RxHeader.Identifier == device_can_id){ // Handle device messages
    int msg_type = RxData[0];

    if(msg_type < 20){
    //   handle_diagnostic_RX(RxHeader, RxData);
    } else if (msg_type < 100) {
        handle_action_RX(RxHeader, RxData);
    } else if (msg_type < 150) {
        handle_telemetry_RX(RxHeader, RxData);
    } else if (msg_type < 256) {
    //   handle_parameter_RX(RxHeader, RxData);
    }
  }
}

void handle_action_RX(FDCAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]){
    int msg_type = RxData[0];


    if(msg_type == ACTION_MOTOR_ENABLE_POSITION_CONTROL){
        position_control_enabled = RxData[1];
    }

    if(msg_type == ACTION_MOTOR_POSITION_SETPOINT){
        int setpoint = RxData[1] | RxData[2] << 8 | RxData[3] << 16;
        if(setpoint > 16777216/2){
            setpoint = setpoint - 16777216;
        }

        position_setpoint = setpoint;
    }
}

void handle_telemetry_RX(FDCAN_RxHeaderTypeDef RxHeader, uint8_t RxData[]){
    int msg_type = RxData[0];

    if(msg_type == TELEM_MOTOR_VOLTAGE){
        uint8_t array[] = {TELEM_MOTOR_VOLTAGE, 
                            (voltage_supply_mV) & 0xFF, 
                            (voltage_supply_mV >> 8) & 0xFF};
        CAN_Transmit_Array(array, 3);
    }

    if(msg_type == TELEM_MOTOR_POSITION){
        uint8_t array[] = {TELEM_MOTOR_POSITION, 
                            (enc_angle_int) & 0xFF, 
                            (enc_angle_int >> 8) & 0xFF,
                            (enc_angle_int >> 16) & 0xFF};
        CAN_Transmit_Array(array, 4);
    }

    if(msg_type == TELEM_MOTOR_CURRENTS){
        uint8_t array[] = {TELEM_MOTOR_CURRENTS, 
                            (current_A_mA_filtered) & 0xFF, 
                            (current_A_mA_filtered >> 8) & 0xFF,
                            (current_B_mA_filtered) & 0xFF, 
                            (current_B_mA_filtered >> 8) & 0xFF,
                            (current_C_mA_filtered) & 0xFF, 
                            (current_C_mA_filtered >> 8) & 0xFF};
        CAN_Transmit_Array(array, 7);
    }
}


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

    ////////// Jump to bootloader 
    if(RxData[0] == 'B' && RxData[1] == 'T' && RxData[2] == 'L'){
        // Set ram blocks to request app to run
        boot_bits[0] = 'B';
        boot_bits[1] = 'T';
        boot_bits[2] = 'L';
        HAL_NVIC_SystemReset();
    }

    // Handle device RX
    handleRX(RxHeader, RxData);

    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}



// There must be a better way to do this
FDCAN_RxHeaderTypeDef convert_tx_header(FDCAN_TxHeaderTypeDef *TxHeader){
    FDCAN_RxHeaderTypeDef RxHeader;
    RxHeader.BitRateSwitch = TxHeader->BitRateSwitch;
    RxHeader.DataLength = TxHeader->DataLength;
    RxHeader.Identifier = TxHeader->Identifier;
    RxHeader.IdType = TxHeader->IdType;

    RxHeader.RxFrameType = TxHeader->TxFrameType;

    return RxHeader;
}