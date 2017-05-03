// usb_shell.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


#include "usb_shell.h"


/***********/
/* GLOBALS */
/***********/


uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_buf_len = 0;

uint8_t tx_buf[TX_BUF_SIZE];
uint8_t tx_buf_len = 0;

uint8_t exec_cmd_flag = 0;

// a string that will make backspaces actually look like backspaces in a terminal connection
const uint8_t backspace_str[] = "\b \b";


/**************/
/* INTERRUPTS */
/**************/


ISR(USART_RXC_vect)
// Code executed when USART receives a byte.
{
    uint8_t received_char;

    // Fetch the recieved byte value.
    received_char = UDR; 

    switch (received_char) {
        case '\b':
            rx_buf[rx_buf_len] = '\0';
            /* Need rx_buf_len to be non-zero before decrement to prevent unsigned integer 
             * underflow. */
            if (rx_buf_len) {
                rx_buf_len--;
                // Need the following cast to avoid the warning.
                usart_putstr((uint8_t *)backspace_str);
            }
            break;
        case '\r':
        case '\n':
            UDR = received_char; 
            rx_buf[rx_buf_len] = '\0';
            exec_cmd_flag = 1;
            break;
        default:
            UDR = received_char; 
            //sprintf(rx_buf, "0x%x ", received_char);
            //usart_putstr(rx_buf);
            rx_buf[rx_buf_len++] = received_char;
            // Prevent buffer overflows.
            if (rx_buf_len >= RX_BUF_SIZE) rx_buf_len = 0;
            break;
    }
}


/************************/
/* FUNCTION DEFINITIONS */
/************************/

void usart_putchar(uint8_t char_to_put)
/*
 * Waits for transmit flag to become clear, then sends a single byte.
 */
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = char_to_put;
}

void usart_putstr(char *str_to_put)
/*
 * Transmits the given string over USART.
 */
{
    uint8_t iter;
    uint8_t char_to_put;
    for (iter = 0; (char_to_put = str_to_put[iter]); iter++) {
        usart_putchar(char_to_put);
    }
}

void initialize_usart()
{
    // Enable usart for transmit and receive.
    UCSRB |= (1 << RXEN) | (1 << TXEN);   
    // 8-bit chars (uint8_t)
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 

    // load lower 8-bits of the baud rate value into the low byte of the UBRR register
    UBRRL = BAUD_PRESCALE; 
    // load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRRH = (BAUD_PRESCALE >> 8); 

    // enable 'recieve complete' interrupt (USART_RXC)
    UCSRB |= (1 << RXCIE); 

    // Enable interrupts globally
    sei();
}
