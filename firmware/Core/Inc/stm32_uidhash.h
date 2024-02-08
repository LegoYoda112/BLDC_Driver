#ifndef _STM32_UIDHASH_
#define _STM32_UIDHASH_

#include "main.h"

#include <stdio.h>
#include <string.h>

#include "stm32g4xx_ll_utils.h"

extern uint32_t device_uid;

uint32_t Hash32Len5to12(const char *s, size_t len);
uint32_t set_32bit_uid();

#endif