#include <stdint.h>

/* Minimal STM32G4 vector table */
extern void _start(void);
void Reset_Handler(void);
void Default_Handler(void) { while (1); }

__attribute__((section(".isr_vector")))
const void *vector_table[] = {
    (void *)0x20020000,   /* Initial SP (top of 128KB SRAM) */
    Reset_Handler,        /* Reset                           */
    Default_Handler,      /* NMI                             */
    Default_Handler,      /* HardFault                       */
};

/* Simple delay */
static void delay(volatile uint32_t n) {
    while (n--);
}

void Reset_Handler(void) {
    /* GPIOA clock enable (RCC_AHB2ENR bit 0) */
    volatile uint32_t *RCC_AHB2ENR = (uint32_t *)0x4002104C;
    *RCC_AHB2ENR |= (1 << 0);

    /* GPIOA MODER: set PA5 to output (bits 11:10 = 01) */
    volatile uint32_t *GPIOA_MODER = (uint32_t *)0x48000000;
    *GPIOA_MODER &= ~(3 << 10);
    *GPIOA_MODER |=  (1 << 10);

    /* GPIOA ODR: toggle PA5 forever */
    volatile uint32_t *GPIOA_ODR = (uint32_t *)0x48000014;
    while (1) {
        *GPIOA_ODR ^= (1 << 5);
        delay(500000);
    }
}