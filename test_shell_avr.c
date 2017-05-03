// test_shell_avr.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


#include "usb_shell.h"


uint16_t sum_chars_in_buf(uint8_t *buf)
{
    uint8_t iter;
    uint16_t sum;

    sum = 0;
    for (iter = 0; iter < 255 && buf[iter]; iter++) {
        sum += buf[iter];
    }

    return sum;
}

void cmd_handler(HC_T *hc, SIE_T *sie, uint16_t sum)
{
    uint8_t iter;
    uint8_t rx_byte_count;

    // transaction-specific variables
    TRANSACTION_T *setup;
    TRANSACTION_T **ins;
    TOKEN_PACKET_T *tp;
    DATA_PACKET_T *dp;

    switch (sum) {
        case 2096: // device get descriptor
        // hold reset
          sie_reset(sie);
		  for (iter = 0; iter < 100; iter++) _delay_ms(10);
		  sie_idle(sie);

		  // create a setup transaction
		  tp = TokenPacket(SETUP, 0, 0);
		  rx_packet_count = 0x12; //expecting 18 bytes in response
		  dp = make_GET_DEVICE_DESCRIPTOR(rx_byte_count);

		  // create the control transaction with the new token packet
		  setup = Transaction(CONTROL, tp, dp);
		  hc_push_transaction(hc, setup);

		  // we also need 3 INs to read the device descriptor
		  for (iter = 0; iter < (rx_packet_count/8 + 1); iter++) {
			tp = TokenPacket(IN, 0, 0);
			// this is a data in, so PID and size don't matter
			dp = DataPacket(DATA0, 0); 
			ins[iter] = Transaction(CONTROL, tp, dp);
			// push the transaction on the HC queue
			hc_push_transaction(hc, ins[iter]);
		}

	// do the transactions
	for (iter = 0; iter < 4; iter++) hc_do_transaction(hc);
            break;
        case 2408: // device get configuration
            break;
        case 1666: // device connected?
        case 1603: // device connected
            switch (sie_detect_device(sie)) {
                // Returns -1 if full or high speed, 1 if low speed, 0 if none connected.
                case -1:
                    sprintf(tx_buf, "Full speed device connected\n");
                    break;
                case 1:
                    sprintf(tx_buf, "Low speed device connected\n");
                    break;
                default:
                    sprintf(tx_buf, "No device connected\n");
                    break;
            }
            usart_putstr(tx_buf);
            break;
        default:
            sprintf(tx_buf, "sum: %d", sum);
            usart_putstr(tx_buf);
            break;
    }
}

int main() 
{
    HC_T *hc;
    SIE_T *sie;
    char prompt[] = "bcd# ";
    uint16_t sum;

    hc = HostController();
    sie = SerialInterfaceEngine();
    hc_bind_sie(hc, sie);

    sie_idle(sie);

    initialize_usart();
    usart_putstr(prompt);
    for (;;) {
        if (exec_cmd_flag) {
            exec_cmd_flag = 0;
            sum = sum_chars_in_buf(rx_buf);
            cmd_handler(hc, sie, sum);
            rx_buf_len = 0;
            usart_putchar('\n');
            usart_putstr(prompt);
        }
    }

    return 0;
}
