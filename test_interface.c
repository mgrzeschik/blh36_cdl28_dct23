// test_interface.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


#include "usb_interface.h"


/*void test_crc_junx()
{
    TOKEN_PACKET_T *tp;
    TRANSACTION_T *trans;
    SIE_T *sie;
    uint8_t iter;

    sie = SerialInterfaceEngine();
    tp = TokenPacket(IN, 1, 1);
    packet_make_crc5(tp);
    trans = Transaction(IN, tp, NULL);
    transaction_place_in_buffer(trans, sie->buffer, &(sie->token_packet_length), 
                                &(sie->data_packet_length));
    for (iter = 0; iter < 3; iter++) printf("%d: 0x%x\n", iter, sie->buffer[iter] & 0xff);
    sie_nrzi_encode_buffer(sie, 1);
    for (iter = 0; iter < 3; iter++) printf("%d: 0x%x\n", iter, sie->buffer[iter] & 0xff);
}*/


unsigned char sie_nrzi_toggle_buffer_test()
// Tests the function: void sie_nrzi_toggle_buffer(SIE_T *sie, unsigned char transient_first_state)
{
    unsigned char valid = 1;
    unsigned char iter;
    unsigned char expected;
    unsigned char *buffer;
    SIE_T *sie;

    sie = SerialInterfaceEngine();
    sie->buffer[0] = 0x96; // IN
    sie->token_packet_length = 1 * 8;
    
    // NRZI encode the buffer
    sie_nrzi_encode_buffer(sie, 1);
    buffer = (unsigned char *) sie->buffer;

    iter = (buffer[0] == 0xb1);
    valid = valid && iter;

    // NRZI decode the buffer -- we should get the original value
    sie_nrzi_decode_buffer(sie, 1);
    expected = 0x96;
    iter = (buffer[0] == expected);
    if (!iter) printf("nrzi unencoded buffer %d: value 0x%x, expected 0x%x\n", 0, buffer[0], 
                        expected & 0xff);
    valid = valid && iter;

    DestroySerialInterfaceEngine(sie);

    return valid;
}


unsigned char sie_bitstuff_buffer_test()
// Tests the function: void sie_bitstuff_buffer(SIE_T *sie)
{
    unsigned char valid = 1;
    unsigned char iter;
    SIE_T *sie;
    TRANSACTION_T *trans;
    uint8_t *buffer;

    sie = SerialInterfaceEngine();

    trans = Transaction(CONTROL, TokenPacket(IN, 2, 3), NULL);
    // put the transaction in the SIE's buffer
    transaction_place_in_buffer(trans, sie->buffer, &(sie->token_packet_length), 
                                &(sie->data_packet_length));
    // 1001 0110 (0100 000) (1 100)0 0000
    // 0x96       0x41       0x80
    //printf("sie tpl: %d; sie dpl: %d\n", sie->token_packet_length, sie->data_packet_length);
    sie_bitstuff_buffer(sie);
    buffer = sie->buffer;
    valid = valid && (buffer[0] == 0x96) && (buffer[1] == 0x41) && (buffer[2] == 0x80);
    DestroyWholeTransaction(trans);

    // use a fake PID to make sure buffer stuffing is working
    trans = Transaction(CONTROL, TokenPacket(0xff, 2, 3), NULL);
    
    // put the transaction in the SIE's buffer
    transaction_place_in_buffer(trans, sie->buffer, &(sie->token_packet_length), 
                                &(sie->data_packet_length));

    sie_bitstuff_buffer(sie);
    //for (iter = 0; iter < 3; iter++) printf("buffer[%d]: 0x%x\n", iter, sie->buffer[iter]);
    buffer = sie->buffer;
    valid = valid && (buffer[0] == 0xfd) && (buffer[1] == 0xa0) && (buffer[2] == 0xc0);
    DestroyWholeTransaction(trans);

    DestroySerialInterfaceEngine(sie);

    return valid;
}


int main()
{
    //test_crc_junx();

    if (sie_bitstuff_buffer_test()) printf("PASSED: sie_bitstuff_buffer_test()\n");
    else printf("FAILED: sie_bitstuff_buffer_test()\n");

    if (sie_nrzi_toggle_buffer_test()) printf("PASSED: sie_nrzi_toggle_buffer_test()\n");
    else printf("FAILED: sie_nrzi_toggle_buffer_test()\n");

    return 0;
}
