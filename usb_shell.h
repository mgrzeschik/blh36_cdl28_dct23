// usb_shell.h
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width

/*
 * NOTE: almost this entire file is <avr/io.h> dependent!
 */

#ifndef _USB_SHELL_H
#define _USB_SHELL_H


#include "usb_interface.h"

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define RX_BUF_SIZE 50
#define TX_BUF_SIZE 50


extern uint8_t rx_buf[];
extern uint8_t rx_buf_len;

extern uint8_t tx_buf[];
extern uint8_t tx_buf_len;

extern uint8_t exec_cmd_flag;

void usart_putchar(uint8_t char_to_put);
void usart_putstr(char *str_to_put);
void initialize_usart();

#endif
