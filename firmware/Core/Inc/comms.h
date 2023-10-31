#ifndef COMMS_H
#define COMMS_H

#include "main.h"
#include "drive.h"

/**
 * @brief Called when a new line of USB data is received
 * 
 * @param Buf Buffer containing bytes
 * @param Len Length of buffer
 * 
 * Buffer will fill until a newline character ('\\n') is sent through the virtual comm port
 */
void process_USB_rx(uint8_t* Buf, uint32_t *Len);

#endif