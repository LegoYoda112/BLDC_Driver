#pragma once
#include "stm32g4xx_hal.h"

namespace encoder 
{

void initialize();
void interrupt();
void index_interrupt();

int get_raw();
bool get_index_read();

} // encoder