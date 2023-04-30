#ifndef __UART_H__
#define __UART_H__

void uart_init(uint32_t baudrate);
uint16_t uart_rx_avail();
void uart_putch(char ch);
void uart_puts(char *ch);
uint8_t uart_getch();

void uart_print_hex_word(uint32_t outv);
void uart_print_hex_short(uint16_t outv);
void uart_print_hex_byte(uint8_t outv);

#endif
