// interface_test_main.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// A test program for the SIE.

#include "usb_client.h"

#define debug(string) sprintf(print_buffer, string); send_debug()


void test_hc_sie_interaction() 
{
    HC_T *hc;
    SIE_T *sie;
    uint8_t iter;

    // transaction-specific variables
    STANDARD_REQUEST_T *request;
    TRANSACTION_T *trans1, *trans2;

    // create an SIE and a HC
    hc = HostController();
    sie = SerialInterfaceEngine();
    hc_bind_sie(hc, sie);

    // hold reset
    sie_reset(sie);
    for (iter = 0; iter < 100; iter++) _delay_ms(10);
    sie_idle(sie);

    // Ask for the first 8 bytes of the Device Descriptor
    request = standard_request_from_template(GET_DESCRIPTOR, DEVICE);
    request->wValueHi = DEVICE;
    request->wLength = 64;
    execute_standard_request(request, sie, hc, 0, 0, NULL, 1);

    // set address to 1
    request = standard_request_from_template(SET_ADDRESS, DEVICE);
    request->wValueLo = 1;
    execute_standard_request(request, sie, hc, 0, 0, NULL, NULL);
    
    // Ask for the first 18 bytes of the Device Descriptor
    request = standard_request_from_template(GET_DESCRIPTOR, DEVICE);
    request->wValueHi = DEVICE;
    request->wLength = 0x12;
    execute_standard_request(request, sie, hc, 1, 0, NULL, NULL);
    
    // Ask for the first 9 bytes of the Device Descriptor
    request = standard_request_from_template(GET_DESCRIPTOR, DEVICE);
    request->wValueHi = CONFIGURATION;
    request->wLength = 0x9;
    execute_standard_request(request, sie, hc, 1, 0, NULL, NULL);
    
        // Ask for a Report Descriptor
    request = hid_request_from_template(GET_REPORT);
    request->wValueHi = 1;
    request->wIndex = 0;
    request->wLength = 0x8;
    execute_standard_request(request, sie, hc, 1, 0, NULL, NULL);

    // set address to 1
    request = standard_request_from_template(SET_CONFIGURATION, DEVICE);
    request->wValueLo = 1;
    execute_standard_request(request, sie, hc, 1, 0, NULL, NULL);

    for (;;) {
        trans1 = Transaction(INTERRUPT, TokenPacket(IN, 1, 1), DataPacket(DATA0, 0));
        trans2 = Transaction(INTERRUPT, TokenPacket(IN, 1, 1), DataPacket(DATA0, 0));
        hc_push_transaction(hc, trans1);
        hc_push_transaction(hc, trans2);
        hc_do_transaction(hc);
        hc_do_transaction(hc);
        DestroyWholeTransaction(trans1);
        DestroyWholeTransaction(trans2);
    }

    debug("Program finished.\n");
    debug("-----------------\n");
    debug("-----------------\n\n\n");
}

int main()
{
    // initialization

    DDRA=0x0f;
    DDRC=0xff;

    test_hc_sie_interaction();

    while (1);
}
