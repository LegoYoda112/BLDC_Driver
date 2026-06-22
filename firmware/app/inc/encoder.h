#pragma once
#include "stm32g4xx_hal.h"

namespace encoder 
{

void initialize();
void interrupt(uint16_t);
void index_interrupt();

int get_raw();
bool get_index_read();
uint8_t get_enc_val();

} // encoder