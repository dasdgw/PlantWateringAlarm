#ifndef __UART_H
#define __UART_H
#include <stdint.h>
#include <avr/io.h>
 
// Baud rate
#define UART_BPS 10000
#define UART_TX_DDR  DDRA
#define UART_TX_PORT PORTA
#define UART_TX_PIN  PA6
 
extern uint8_t uart_setup(void);
extern void uart_putc(char c);
extern void uart_puts(char* s);
 
#endif
