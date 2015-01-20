//http://www.mikrocontroller.net/topic/334426#3668118
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
 
#define UART_SET_BIT(port,bit)   (port |=  (1 << bit))
#define UART_CLEAR_BIT(port,bit) (port &= ~(1 << bit))
 
#define UART_BIT_WAIT()  _delay_us(1000000.0 / (float) UART_BPS)
#define UART_SEND_HIGH() {UART_CLEAR_BIT(UART_TX_PORT, UART_TX_PIN); UART_BIT_WAIT();}
#define UART_SEND_LOW()  {UART_SET_BIT(UART_TX_PORT, UART_TX_PIN); UART_BIT_WAIT();}
 
uint8_t uart_setup(void){
  UART_SET_BIT(UART_TX_DDR, UART_TX_PIN);
  UART_SET_BIT(UART_TX_PORT, UART_TX_PIN);
  return 0;
}
 
void uart_putc(char c){
  uint8_t i = 8;
  // start bit
  UART_SEND_HIGH();
  while(i){
    if(c & 1){
      UART_SEND_LOW();
    } else {
      UART_SEND_HIGH();
    }
    c = c >> 1;
    i--;
  }
  // stop bits
  UART_SEND_LOW();
}
 
void uart_puts(char* s){
  while (*s) {
    uart_putc(*s);
    s++;
  }
}
