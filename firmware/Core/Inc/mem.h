#ifndef MEM_H
#define MEM_H
#include "main.h"
#include "config.h"
#include "stdbool.h"

#define EEPROM_ADDRESS 0xA0
#define PAGE_SIZE 64
#define PAGE_NUM 250

#define PARAM_ENCODER_OFFSET_PAGE 2
#define PARAM_ENCODER_OFFSET_BYTE 0

extern I2C_HandleTypeDef hi2c1;
extern uint8_t electrical_angle_offset;

uint16_t bytestowrite(uint16_t size, uint16_t offset);
void mem_write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void mem_read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

void mem_write_uint8(uint8_t number, uint16_t page, uint16_t offset);
uint8_t mem_read_uint8(uint16_t page, uint16_t offset);
void mem_write_uint16(uint16_t number, uint16_t page, uint16_t offset);
uint16_t mem_read_uint16(uint16_t page, uint16_t offset);
void mem_write_uint32(uint32_t number, uint16_t page, uint16_t offset);
uint32_t mem_read_uint32(uint16_t page, uint16_t offset);


void write_encoder_params();
void read_encoder_params();

#endif