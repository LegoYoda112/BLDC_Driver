#include "comms.h"

using namespace comms;

// Preallocate headers and data buffers
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[64];
uint8_t RxData[64];

// CAN handle to use
FDCAN_HandleTypeDef* hfdcan;

// Device ID, also used as CAN ID
uint16_t device_id = 0;

bool slcan_open = false;

void comms::set_device_id(uint16_t _device_id){
    device_id = _device_id;
}

uint16_t comms::get_device_id(){
    return device_id;
}

int comms::can_transmit(FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData)
{
    if(slcan_open){
        send_slcan_string(convert_tx_header(TxHeader), TxData);
    }

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0){
        return 0;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, TxHeader, TxData) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

void comms::can_receive(FDCAN_RxHeaderTypeDef *TxHeader, uint8_t *TxData)
{
    return;
}

void comms::initialize(FDCAN_HandleTypeDef* _hfdcan)
{
    hfdcan = _hfdcan;

    // Set up CAN header
    TxHeader.Identifier = device_id;
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


    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }
    
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    // Activate the notification for new data in FIFO0 for FDCAN1
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}

void comms::slcan_usb_rx(uint8_t *Buf, uint32_t buffer_length)
{
    /////// Open interface
    if (Buf[0] == 'O')
    {
        // Check if CAN is already started
        if (HAL_FDCAN_GetState(hfdcan) == HAL_FDCAN_STATE_READY)
        {
            // Else, attempt to start CAN interface
            if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
            {
                Error_Handler();
            }
        }

        // app_state = app_state_active;
        slcan_open = true;
    }

    /////// Close interface
    if (Buf[0] == 'C')
    {
        // Ensure that CAN has already been started
        if (HAL_FDCAN_GetState(hfdcan) != HAL_FDCAN_STATE_BUSY)
        {
            return;
        }

        // Else, attempt to close CAN interface
        if (HAL_FDCAN_Stop(hfdcan) != HAL_OK)
        {
            Error_Handler();
        }

        // app_state = app_state_idle;
        slcan_open = false;
    }

    /////// Set bitrate
    // Only supports 500 and 1000 Kbit/s for now
    // if (Buf[0] == 'S')
    // {
    //     volatile int bitrate_code = Buf[1] - '0';

    //     if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_READY)
    //     {
    //         return;
    //     }

    //     if (bitrate_code == _500Kbs)
    //     {
    //         can_set_500kbs();
    //     }
    //     else if (bitrate_code == _1000Kbs)
    //     {
    //         can_set_1000kbs();
    //     }

    //     if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    //     {
    //         Error_Handler();
    //     }
    // }

    /////// Send message
    if (Buf[0] == 't' || Buf[0] == 'T')
    { // 11 bit data frame

        volatile int dlc;
        char id_substring[12];

        if (Buf[0] == 't')
        {
            strncpy(id_substring, (char *) Buf + 1, 3);
            id_substring[3] = '\0';
            TxHeader.IdType = FDCAN_STANDARD_ID;
            dlc = Buf[4] - '0';
        }
        else
        {
            strncpy(id_substring, (char *) Buf + 1, 11);
            id_substring[11] = '\0';
            TxHeader.IdType = FDCAN_EXTENDED_ID;
            dlc = Buf[9] - '0';
            // TODO: do this
        }

        TxHeader.DataLength = FDCAN_DLC_BYTES_1 * dlc;
        TxHeader.Identifier = strtol(id_substring, NULL, 16);

        for (int i = 0; i < dlc; i++)
        {
            char byte_substring[3];
            byte_substring[2] = '\0';
            strncpy(byte_substring, (char *) Buf + 1 + 4 + i * 2, 2);
            TxData[i] = strtol(byte_substring, NULL, 16);
        }

        // Fake loopback
        send_slcan_string(comms::convert_tx_header(&TxHeader), TxData);

        slcan_open = false;
        can_transmit(&TxHeader, TxData);
        slcan_open = true;

        RxHeader = convert_tx_header(&TxHeader);
        can_receive(&RxHeader, TxData);
    }
}

void comms::send_slcan_string(FDCAN_RxHeaderTypeDef RxHeader, uint8_t *RxData)
{
    char tx_string[30];

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

    CDC_Transmit_FS((unsigned char*) tx_string, 5 + 2*dlc + 1);
}

// There must be a better way to do this
FDCAN_RxHeaderTypeDef comms::convert_tx_header(FDCAN_TxHeaderTypeDef *TxHeader){
    FDCAN_RxHeaderTypeDef RxHeader;
    RxHeader.BitRateSwitch = TxHeader->BitRateSwitch;
    RxHeader.DataLength = TxHeader->DataLength;
    RxHeader.Identifier = TxHeader->Identifier;
    RxHeader.IdType = TxHeader->IdType;

    RxHeader.RxFrameType = TxHeader->TxFrameType;

    return RxHeader;
}