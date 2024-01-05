#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "main.h"

///////////// Externs in
extern FDCAN_HandleTypeDef hfdcan1;
extern uint8_t TxData[8];
extern FDCAN_TxHeaderTypeDef TxHeader;

///////////// Defines
#define APP_START_ADDR   0x8008000
#define APP_END_ADDR    (0x8008000 + 46 * FLASH_PAGE_SIZE - 1)
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0801F800)

///////////// Variables
// Bootloader state variable
enum bootloaderState{
    init,
    idle,
    flashing
};

extern enum bootloaderState bootloader_state;

///////////// Functions
/**
 * @brief Called after peripherals have been initted
 * branches into running app code, or running the rest
 * of the bootloader code. 
 * 
 */
void bootloader_init();

/**
 * @brief Called in main.c while loop FDCAN1 and LED GPIO pins will have been initted. 
 * 
 * @note
 * Test
 * 
 */
void bootloader_loop();

/**
 * Erases app flash (in preparation for flashing). 
 * 
 * @note
 * Erases pages between 0x8008000 (page 16) to 0x0801F800
 * (page 63)
 * 
 */
void erase_flash();

/**
 * @brief DeInits peripherals and jumps processor to
 * 0x8008000 
 * 
 */
void bootloader_jump_to_user_app(void);

/**
 * @brief Called when USB interface reaches a '\\r' character
 * 
 * @param Buf Array of received characters
 * @param Len Length of buffer
 */
void process_USB_rx(uint8_t* Buf, uint32_t *Len);


void process_CAN_rx(FDCAN_RxHeaderTypeDef *Rxheader, uint8_t RxData[]);

#endif