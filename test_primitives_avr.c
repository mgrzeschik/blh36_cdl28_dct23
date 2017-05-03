// primitives_test_main.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// A test program for the USB primitives library.

#include <stdio.h>
#include "usb_primitives.h"

int main() {
    TRANSACTION_T *transaction;
    TOKEN_PACKET_T *tp;
    DATA_PACKET_T *dp;

    // create a setup transaction
    {
        // endp should be zero for default pipe, TODO: but I'm not sure about addr
        tp = TokenPacket(SETUP, 0, 0);

        // page 250 of USB2.0 specification -- GET_CONFIGURATION (10000000 == 0x80)
        dp = DataPacket(DATA0, 16);
        *dp->data = 0x80;

        transaction = Transaction(CONTROL, tp, dp, UNENCODED);
    }

    while (1);

    return 0;
}
