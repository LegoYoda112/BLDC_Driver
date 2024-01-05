#include "bootloader.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "stdbool.h"

#include "can.h"

unsigned char __attribute__((section(".bootBlockRAM"))) boot_bits[10];

// Grab usb device pointer
extern USBD_HandleTypeDef hUsbDeviceFS;

enum bootloaderState bootloader_state;
typedef void (*pFunction)(void);

uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint64_t data64 = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

bool flashing_firmware = false;
uint8_t words_ready = 0;
uint64_t double_word = 0;
uint8_t null_counter = 0;

static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
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

void can_ack(){
    TxData[0] = 'O';
    TxData[1] = 'K';
    TxHeader.Identifier = 69;
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    CAN_Transmit_Safe(&TxHeader, TxData);
}

void process_CAN_rx(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[]){

    if(RxHeader->Identifier == 69){
        if(RxHeader->DataLength == FDCAN_DLC_BYTES_0){
            flashing_firmware = false;
        }

        if(flashing_firmware){
            __disable_irq();
            double_word = 0;
            for(int i = 0; i < 8; i++){
                double_word = double_word | ((uint64_t)RxData[i] << (i * 8));
            }

            // Calculate flash address
            uint32_t offset_address = (uint32_t)APP_START_ADDR + Address;

            // Flash
            HAL_FLASH_Unlock();
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, offset_address, double_word) != HAL_OK){
                Error_Handler();
            }
            HAL_FLASH_Lock();
            __enable_irq();

            Address += 8;
        }

        ////////// Clear memory
        if(RxData[0] == 'C' && RxData[1] == 'L' && RxData[2] == 'R'){
            erase_flash();
            Address = 0;
            flashing_firmware = true;
        }

        ////////// Jump to app 
        if(RxData[0] == 'A' && RxData[1] == 'P' && RxData[2] == 'P'){
            // Set ram blocks to request app to run
            boot_bits[0] = 'A';
            boot_bits[1] = 'P';
            boot_bits[2] = 'P';
            can_ack();
            HAL_NVIC_SystemReset();
        }

        
        can_ack();
    }
}

void process_USB_rx(uint8_t* Buf, uint32_t *Len){
    __disable_irq();

    ////////// Clear 
    if(Buf[0] == 'C' && Buf[1] == 'L' && Buf[2] == 'R'){
        erase_flash();
        Address = 0;
    }

    ////////// Jump to app 
    if(Buf[0] == 'A' && Buf[1] == 'P' && Buf[2] == 'P'){
        // Set ram blocks to request bootloader to run
        boot_bits[0] = 'A';
        boot_bits[1] = 'P';
        boot_bits[2] = 'P';
        HAL_NVIC_SystemReset();
        // bootloader_jump_to_user_app();
    }

    // Receive ihex data
    if(Buf[0] == ':'){
        char record_substring[3];
        strncpy(record_substring, Buf + (uint32_t)7, 2);
        volatile uint32_t record_type = strtol(record_substring, NULL, 16);

        // Process data record
        if(record_substring[0] == '0' && record_substring[1] == '0') {
            
            // Hardcode data length
            uint16_t data_length = 16;
            uint8_t data[16];
            // Record strings are 16 byte long = 2x 64bit double words
            uint64_t double_word_1 = 0;
            uint64_t double_word_2 = 0;

            // Extract data array from hex string and fill double words
            for(int i = 0; i < data_length; i++){

                // Convert hex byte to uint8
                char data_substring[3];
                strncpy(data_substring, Buf + 9 + 2 * i, 2);
                data_substring[2] = '\0';
                data[i] = strtol(data_substring, NULL, 16);

                // Fill double_word_1
                if(i < 8){
                    double_word_1 = double_word_1 | ((uint64_t)data[i] << i * 8);
                }else{ // Fill double_word_2
                    double_word_2 = double_word_2 | ((uint64_t)data[i] << (i - 8) * 8);
                }
            }

            // Calculate flash address
            uint32_t offset_address = (uint32_t)APP_START_ADDR + Address;

            // Flash two double words
            HAL_FLASH_Unlock();
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, offset_address, double_word_1) != HAL_OK){
                Error_Handler();
            }
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, offset_address + 0x08, double_word_2) != HAL_OK){
                Error_Handler();
            }
            HAL_FLASH_Lock();

            // Increment address
            Address += 16;
        }

    }

    uint8_t *data = "OK\r";
    CDC_Transmit_FS(data, strlen(data));
    __enable_irq();
}


void bootloader_jump_to_user_app(void)
{
    void (*app_reset_handler)(void);

    // DeInit all
    HAL_GPIO_DeInit(LED_R_GPIO_Port, LED_R_Pin);
    HAL_GPIO_DeInit(LED_G_GPIO_Port, LED_G_Pin);
    HAL_GPIO_DeInit(LED_B_GPIO_Port, LED_B_Pin);
    HAL_GPIO_DeInit(DIP_3_GPIO_Port, DIP_3_Pin);

    USBD_DeInit(&hUsbDeviceFS);
    USBD_LL_DeInit(&hUsbDeviceFS);

    HAL_FDCAN_DeInit(&hfdcan1);
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Disable interrupts
    __set_PRIMASK(1);
    __disable_irq();

    // SCB->VTOR = FLASH_SECTOR2_BASE_ADDRESS; //0x080080000

    // Set main stack pointer
    uint32_t msp_value = *(__IO uint32_t *)APP_START_ADDR;
    __set_MSP(msp_value);

    // Set up reset handler function pointer
    uint32_t resethandler_address = *(__IO uint32_t *) (APP_START_ADDR + 4);
    app_reset_handler = (void*) resethandler_address;

    // Call app reset handler to jump to app code
    app_reset_handler();

}

bool bootloader_requested(){
    // If DIP_3 is ON, skip boot bits check and run bootloader
    if(!HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin)){
        return true;
    }

    // If boot bits are set for bootloader (BTL), run bootloader
    // if they aren't set, run app
    if(boot_bits[0] == 'B' && boot_bits[1] == 'T' && boot_bits[2] == 'L'){
        return true;
    } else {
        return false;
    }
}

void bootloader_init(){
    if(bootloader_requested()){
        // Continue
    }else{
        bootloader_jump_to_user_app();
    }

    __enable_irq();
    init_and_start_can();
}

void bootloader_loop(){
    if(flashing_firmware){
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
        HAL_Delay(200);
    } else {
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
        HAL_Delay(900);
    }

}

void erase_flash(){
    
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); // Not quite sure this was included in example code

    // Set up erase pages
    FirstPage = GetPage(APP_START_ADDR);
    NbOfPages = GetPage(APP_END_ADDR) - FirstPage + 1;
    // Set up erase struct
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = FirstPage;
    EraseInitStruct.NbPages = NbOfPages;

    // Perform erase
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
        Error_Handler();
    }

    HAL_FLASH_Lock();
}