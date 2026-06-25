#pragma once

#include "main.h"

#include <stdio.h>
#include <string.h>

#include "stm32g4xx_ll_utils.h"

namespace uid_hash{

void initialize();
uint32_t get_32bit_uid();

}