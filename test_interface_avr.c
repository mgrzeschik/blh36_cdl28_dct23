// interface_test_main.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// A test program for the SIE.

#include "usb_primitives.h"
#include "usb_interface.h"
#include <stdlib.h>

#define debug(string) sprintf(print_buffer, string); send_debug()

#if 0
void test_hc_based_junks()
{
    HC_T *hc;
    SIE_T *sie;
    uint8_t iter;
    TRANSACTION_T *trans;

    hc = HostController();
    sie = SerialInterfaceEngine();
    hc_bind_sie(hc, sie);

    trans = Transaction(CONTROL, TokenPacket(IN, 0, 0), DataPacket(DATA0, 0));
    hc_push_transaction(hc, trans);

    sie_reset(sie);
    for (iter = 0; iter < 100; iter++) 
        _delay_ms(10); // 17.476 ms max
    sie_idle(sie);

    hc_do_transaction(hc);
}

void testing_some_junks_names_get_worse()
{
    TOKEN_PACKET_T *tp;
    TRANSACTION_T *trans;
    SIE_T *sie;
    uint8_t iter;

    sie = SerialInterfaceEngine();
    tp = TokenPacket(IN, 0, 0);
    packet_make_crc5(tp);
    trans = Transaction(CONTROL, tp, NULL);
    transaction_place_in_buffer(trans, sie->buffer, &(sie->token_packet_length), 
                                &(sie->data_packet_length));
    sie_nrzi_encode_buffer(sie, 1);
    for (iter = 0; iter < 3; iter++) sie->buffer[iter] = flip_byte(sie->buffer[iter], 8);
    sie_reset(sie);
    for (iter = 0; iter < 21; iter++) _delay_ms(1000);
    sie_idle(sie);
    sie_control_transfer(sie);
}

void test_nrzi_transfer()
{
    SIE_T *sie;
    uint8_t iter;

    sie = SerialInterfaceEngine();
    sie->buffer_encoding = NRZI;
    sie->buffer[0] = 0xb1;
    sie->buffer[1] = 0xab;
    sie->buffer[2] = 0x46;
    for (iter = 0; iter < 3; iter++) {
        sie->buffer[iter] = flip_byte(sie->buffer[iter], 8);
    }
    
    sie_reset(sie);
    for (iter = 0; iter < 21; iter++) {
        _delay_ms(1);
    }
    sie_idle(sie);

    sie->token_packet_length = 24;
    sie->data_packet_length = 0;

    sie_control_transfer(sie);
}

void test_sie_transfer() 
{
    SIE_T *sie;
    unsigned char iter;

    debug("Program started.\n");

    sie = SerialInterfaceEngine();

    for (iter = 0; iter < 16; iter++) sie->buffer[iter] = 0;

    sie->buffer[0] = 0x0f;
    sie->token_packet_length = 8;
    sie->data_packet_length = 0;
    sie->data_direction = TRANSMIT;

    sie_control_transfer(sie);
    
    debug("Program ended.\n\n");

}
#endif
void test_hc_sie_interaction() 
{
    HC_T *hc;
    SIE_T *sie;
    uint8_t iter;

    // transaction-specific variables
    TRANSACTION_T *setup;
    TRANSACTION_T **ins;
    TOKEN_PACKET_T *tp;
    DATA_PACKET_T *dp;

    // create an SIE and a HC
    hc = HostController();
    sie = SerialInterfaceEngine();
    hc_bind_sie(hc, sie);

    ins = malloc(sizeof(TRANSACTION_T *) * 3);

    // hold reset
    sie_reset(sie);
    for (iter = 0; iter < 100; iter++) _delay_ms(10);
    sie_idle(sie);

    // create a setup transaction
    tp = TokenPacket(SETUP, 0, 0);

    dp = DataPacket(DATA0, 8);
    dp->data[7] = 0x80;
    dp->data[6] = 0x06;
    dp->data[5] = 0x00;
    dp->data[4] = 0x01;
    dp->data[3] = 0x00;
    dp->data[2] = 0x00;
    dp->data[1] = 0x12;
    dp->data[0] = 0x00;

    // create the control transaction with the new token packet
    setup = Transaction(CONTROL, tp, dp);
    hc_push_transaction(hc, setup);
    
    // we also need 3 INs to read the device descriptor
    for (iter = 0; iter < 3; iter++) {
        tp = TokenPacket(IN, 0, 0);
        // this is a data in, so PID and size don't matter
        dp = DataPacket(DATA0, 0); 
        ins[iter] = Transaction(CONTROL, tp, dp);
        // push the transaction on the HC queue
        hc_push_transaction(hc, ins[iter]);
    }

    // do the transactions
    for (iter = 0; iter < 4; iter++) hc_do_transaction(hc);

    debug("Program finished.\n");
    debug("-----------------\n");
    debug("-----------------\n\n\n");
}

int main()
{
    // initialization

    DDRA=0x0f;
    DDRC=0xff;

    //testing_some_junks_names_get_worse();
    //test_hc_based_junks();
    test_hc_sie_interaction();
    //test_sie_transfer();

    while (1);
}
