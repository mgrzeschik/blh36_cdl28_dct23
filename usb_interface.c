// usb_inteface.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width
// See usb_interface.h for headers.


#include "usb_interface.h"
#define debug(string) sprintf(print_buffer, string); send_debug()


// -------------------
// - HOST CONTROLLER -
// -------------------


HC_T *HostController()
// Constructor. Initializes the queue pointers and bound SIE pointer to NULL.
{
    HC_T *hc;

    hc = malloc(sizeof(HC_T));
    hc->transaction_queue_head = NULL;
    hc->transaction_queue_tail = NULL;
    hc->sie = NULL;
    return hc;
}


void DestroyHostController(HC_T *hc)
// Destructor. NOTE: it does not destroy a bound SIE, if there is one.
{
    free(hc);
}


void hc_bind_sie(HC_T *hc, SIE_T *sie)
// Binds an SIE to a host controller
{
    hc->sie = sie;
}


SIE_T *hc_unbind_sie(HC_T *hc)
// Unbinds an associated SIE from a host controller. Returns the previously-bound SIE pointer, in 
// case you cared.
{
    SIE_T *previously_bound_sie;

    previously_bound_sie = hc->sie;
    hc->sie = NULL;
    return previously_bound_sie;
}


void hc_push_transaction(HC_T *hc, TRANSACTION_T *new_transaction)
// Appends the new transaction onto the tail end of the queue.
{
    TRANSACTION_NODE_T *new_transaction_node;

    new_transaction_node = TransactionNode(new_transaction, hc->transaction_queue_tail, NULL);
    if (hc->transaction_queue_tail) hc->transaction_queue_tail->next = new_transaction_node;
    else hc->transaction_queue_head = new_transaction_node;
    // make the new node the tail of the queue
    hc->transaction_queue_tail = new_transaction_node;
}


TRANSACTION_T *hc_pop_transaction(HC_T *hc)
// Pop a transaction pointer off the front of the queue and returns it.
{
    TRANSACTION_NODE_T *return_transaction_node; // necessary so we can destroy it before return
    TRANSACTION_T *return_transaction;
    return_transaction_node = hc->transaction_queue_head;
    // make sure that there are transaction nodes on the queue
    if (return_transaction_node) {
        return_transaction = return_transaction_node->ptr;
        hc->transaction_queue_head = return_transaction_node->next;
        if (hc->transaction_queue_head == NULL) hc->transaction_queue_tail = NULL;
        DestroyTransactionNode(return_transaction_node);
        return return_transaction;
    }
    debug("failed to pop the transaction");
    // if we get to this point there was no transaction node in the queue, so return a null pointer
    return NULL;
}


TRANSACTION_T *hc_do_transaction(HC_T *hc)
{
    // NOTE: transient first state is idle line state. For low speed this is D- high, D+ low, which
    // corresponds to a differential zero.
    #define transient_first_state 1
    #define debug_buffer \
        sprintf(print_buffer, \
                "  0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", \
                (unsigned char) hc->sie->buffer[0] & 0xff, \
                (unsigned char) hc->sie->buffer[1] & 0xff, \
                (unsigned char) hc->sie->buffer[2] & 0xff, \
                (unsigned char) hc->sie->buffer[3] & 0xff, \
                (unsigned char) hc->sie->buffer[4] & 0xff, \
                (unsigned char) hc->sie->buffer[5] & 0xff, \
                (unsigned char) hc->sie->buffer[6] & 0xff, \
                (unsigned char) hc->sie->buffer[7] & 0xff, \
                (unsigned char) hc->sie->buffer[8] & 0xff, \
                (unsigned char) hc->sie->buffer[9] & 0xff, \
                (unsigned char) hc->sie->buffer[10] & 0xff, \
                (unsigned char) hc->sie->buffer[11] & 0xff, \
                (unsigned char) hc->sie->buffer[12] & 0xff, \
                (unsigned char) hc->sie->buffer[13] & 0xff, \
                (unsigned char) hc->sie->buffer[14] & 0xff,\
                (unsigned char) hc->sie->buffer[15] & 0xff); \
        send_debug()

    TRANSACTION_T *executing_transaction;
    unsigned char iter;
    size_t sie_valid_bytes;
    
    // Get the transaction off of the queue.
    executing_transaction = hc_pop_transaction(hc);
    if (!executing_transaction) {
        #ifdef DEBUG_HC
        {
            sprintf(print_buffer, "No executing transaction! Returning to caller.\n");
            send_debug();
        }
        #endif
        return executing_transaction;
    }

    // Calculate token packet CRC.
    packet_make_crc5(executing_transaction->token_packet);

    // Set token packet length in the SIE (before bit stuffing).
    hc->sie->token_packet_length = TOKEN_PACKET_SIZE * CHAR_BITS; 

    // Check if we have a data packet.
    if (executing_transaction->data_packet) {
        if (executing_transaction->data_packet->payload_size) {
            // if the data packet length is non-zero, then we're transmitting
            // indicate that SIE should be transmitting
            hc->sie->data_direction = TRANSMIT;
            // calculate data packet crc
            packet_make_crc16(executing_transaction->data_packet);
            // set data packet length in SIE
            hc->sie->data_packet_length = (BASE_DATA_PACKET_SIZE
                                           + executing_transaction->data_packet->payload_size) * 8;
        }
        else {
            // payload is size zero, which indicates that we're receiving
            
            hc->sie->data_direction = RECEIVE;
            hc->sie->data_packet_length = 0;
        }
    }
    else {
        // we have no data packet, so transmit, but transmit nothing
        hc->sie->data_direction = TRANSMIT;
        hc->sie->data_packet_length = 0;
    }

    // Place transaction in the buffer.
    transaction_place_in_buffer(executing_transaction, hc->sie->buffer, 
                                &(hc->sie->token_packet_length), &(hc->sie->data_packet_length));

    // Bit stuff the buffer. Note: assumes function will change the token and data packet lengths 
    // in the SIE as appropriate.
    sie_bitstuff_buffer(hc->sie);
    
    // Encode the buffer.
    sie_nrzi_encode_buffer(hc->sie, transient_first_state);

    sie_valid_bytes = ((hc->sie->token_packet_length + hc->sie->data_packet_length - 1) >> 3) + 1;

    // Flip everything in the buffer for the SIE.
    for (iter = 0; iter < sie_valid_bytes; iter++) {
        hc->sie->buffer[iter] = flip_byte(hc->sie->buffer[iter], 8);
    }

    // Tell the SIE where to put the handshake reply.
    hc->sie->handshake_result = &(executing_transaction->handshake);

    // Based on the transfer type, call the appropriate SIE transfer function -- it will perform 
    // this transfer with the data that has just been set up in its buffer and internal variables.
    switch (executing_transaction->transfer_type) {
        case CONTROL:
            #ifndef NO_AVR_GCC
            sie_control_transfer(hc->sie);
            #endif
            break;
        case INTERRUPT:
            #ifndef NO_AVR_GCC
            sie_interrupt_transfer(hc->sie);
            #endif
            break;
    }

    sie_valid_bytes = ((hc->sie->token_packet_length + hc->sie->data_packet_length - 1) >> 3) + 1;

    // Flip everything in the buffer -- the SIE puts it in backwards.
    for (iter = 0; iter < sie_valid_bytes; iter++) {
        hc->sie->buffer[iter] = flip_byte(hc->sie->buffer[iter], 8);
    }

    // At this point, the transaction has been completed.

    // Unencode the buffer.
    sie_nrzi_decode_buffer(hc->sie, transient_first_state);

    // Bit unstuff the buffer.
    sie_bitunstuff_buffer(hc->sie);

    // Flip everything in the buffer -- the SIE puts it in backwards.
    for (iter = 0; iter < sie_valid_bytes; iter++) {
        hc->sie->buffer[iter] = flip_byte(hc->sie->buffer[iter], 8);
    }

    #ifdef DEBUG_HC
    debug_buffer;
    #endif

    // Recall that the handshake result was already placed into the SIE (by the SIE) during the 
    // transfer.

    return executing_transaction;

    #undef debug_buffer
    #undef transient_first_state
}


// ---------------------------
// - SERIAL INTERFACE ENGINE -
// ---------------------------


SIE_T *SerialInterfaceEngine()
// Constructor. Returns a pointer to an SIE with an initialized buffer, where all pointers 
// (besides the buffer) are NULL and values are in their most undetermined state. This prepares 
// the values to be set by the Host Controller.
// NOTE: this undetermined-value-setting feature could be optimized out, if need be.
{
    SIE_T *sie;

    sie = malloc(sizeof(SIE_T));
    sie->buffer = calloc(sizeof(char), SIE_BUFFER_SIZE);
    sie->token_packet_length = 0;
    sie->data_packet_length = 0;
    // data direction left alone... no good value to set it to
    sie->handshake_result = NULL;
    sie->buffer_encoding = UNENCODED;
    return sie;
}


void DestroySerialInterfaceEngine(SIE_T *sie)
// Destructor. Frees only the value itself, as the buffer and other properties are set and unset
// by the Host Controller.
{
    free(sie);
}


void sie_bitstuff_buffer(SIE_T *sie)
{
    sie_bitstuff_buffer_helper(sie, STUFF);
}


void sie_bitunstuff_buffer(SIE_T *sie) 
{
    sie_bitstuff_buffer_helper(sie, UNSTUFF);
}


void sie_bitstuff_buffer_helper(SIE_T *sie, STUFF_METHOD_T stuff_method)
{
    #define byte_length(bit_count) (((bit_count - 1) >> 3) + 1)

    uint8_t *buf_ptr;
    uint8_t *new_token_buf, *new_data_buf;
    uint8_t new_token_packet_length;
    uint16_t new_data_packet_length;
    uint8_t iter;

    // Token packet bitstuffing.
    buf_ptr = sie->buffer;
    new_token_packet_length = bitstuff_buffer(&buf_ptr, sie->token_packet_length, stuff_method);
    new_token_buf = buf_ptr;

    // Copy the token values into the SIE buffer.
    memcpy(sie->buffer, new_token_buf, byte_length(sie->token_packet_length));
    sie->token_packet_length = new_token_packet_length;

    // Free the temp array.
    free(new_token_buf);

    if (sie->data_packet_length) {
        // Data packet bitstuffing.
        buf_ptr = &(sie->buffer[byte_length(sie->token_packet_length)]);
        new_data_packet_length = bitstuff_buffer(&buf_ptr, sie->data_packet_length, stuff_method);
        new_data_buf = buf_ptr;

        // Copy the data values into the SIE buffer.
        memcpy(&(sie->buffer[byte_length(sie->token_packet_length)]), new_data_buf, 
                byte_length(sie->data_packet_length));
        sie->data_packet_length = new_data_packet_length;

        // Free the temp array.
        free(new_data_buf);
    }

    #undef byte_length
}


void sie_nrzi_decode_buffer(SIE_T *sie, uint8_t transient_first_state)
// NRZI decodes (in place) for the buffer associated with the given SIE. Takes a transient first 
// state (H/L corresponding to 1/0), as it is necessary to determine the encoding for the first bit.
{
    // What these bitwise macros do is NRZI decode bit by bit. The first bit is XOR'd with 
    // the initial state of the line. This corresponds to the NRZI scheme:
    //      0 means the line stays the same
    //      1 means the line changes
    // This is effetively an XOR from one bit to the next. What these macros are doing is taking
    // bit n of the byte and XORing that bit with bit n-1, leaving the result in the n-bit spot.
    // The shift left one that you see on the left side of the expression is to align the (n-1)th
    // bit with the nth bit on the RHS. When you OR all these bits together, you will end up with
    // the final NRZI result, as given by the toggle macro.
    #define nrzi_bit_one(byte)          (~(((byte & 0x02) >> 1) ^ (byte & 0x01)) & 0x01)
    #define nrzi_bit_two(byte)          (~(((byte & 0x04) >> 1) ^ (byte & 0x02)) & 0x02)
    #define nrzi_bit_three(byte)        (~(((byte & 0x08) >> 1) ^ (byte & 0x04)) & 0x04)
    #define nrzi_bit_four(byte)         (~(((byte & 0x10) >> 1) ^ (byte & 0x08)) & 0x08)
    #define nrzi_bit_five(byte)         (~(((byte & 0x20) >> 1) ^ (byte & 0x10)) & 0x10)
    #define nrzi_bit_six(byte)          (~(((byte & 0x40) >> 1) ^ (byte & 0x20)) & 0x20)
    #define nrzi_bit_seven(byte)        (~(((byte & 0x80) >> 1) ^ (byte & 0x40)) & 0x40)
    #define nrzi_bit_eight(byte, init)  (~(((byte & 0x80) >> 0) ^ (init << 7)) & 0x80)
    // Returns the NRZI decoded version of the byte, given an initial state (which corresponds to 
    // the line state previous to the byte).
    #define nrzi_decode(byte, init) \
        (nrzi_bit_one(byte) | nrzi_bit_two(byte) | nrzi_bit_three(byte) | nrzi_bit_four(byte)\
        | nrzi_bit_five(byte) | nrzi_bit_six(byte) | nrzi_bit_seven(byte) \
        | nrzi_bit_eight(byte, init))

    uint8_t initial_state, iter, current_byte;
    const uint8_t token_packet_bytes = ((sie->token_packet_length - 1) >> 3) + 1;
    const uint8_t data_packet_bytes = ((sie->data_packet_length - 1) >> 3) + 1;

    for (iter = 0; iter < token_packet_bytes; iter++) sie->buffer[iter] = 0;

    initial_state = transient_first_state & 0x1;
    for (iter = token_packet_bytes; iter < token_packet_bytes + data_packet_bytes; iter++) {
        current_byte = sie->buffer[iter];
        sie->buffer[iter] = nrzi_decode(current_byte, initial_state);
        // Take the current byte as the next initial state -- note that only the LSb is used through
        // a shift-left-logical 7, so no masking is necessary.
        initial_state = current_byte;
    }

    for (iter = token_packet_bytes + data_packet_bytes; iter < SIE_BUFFER_SIZE; iter++)
        sie->buffer[iter] = 0;

    sie->buffer_encoding = UNENCODED;

    #undef nrzi_decode
    #undef nrzi_bit_eight
    #undef nrzi_bit_seven
    #undef nrzi_bit_six
    #undef nrzi_bit_five
    #undef nrzi_bit_four
    #undef nrzi_bit_three
    #undef nrzi_bit_two
    #undef nrzi_bit_one 
}


void sie_nrzi_encode_buffer(SIE_T *sie, uint8_t transient_first_state)
{
    const uint8_t token_packet_size = 3;
    const uint8_t data_packet_size = ((sie->data_packet_length - 1) >> 3) + 1;
    uint8_t byte_iter, bit_iter;
    uint8_t current_state;
    uint8_t current_buffer_byte;
    uint8_t shift_amount;
    uint8_t new_bit;

    current_state = transient_first_state;
    // iterate bit by bit -- if the current bit is set, toggle the current state
    for (byte_iter = 0; byte_iter < token_packet_size; byte_iter++) {
        // copy the current buffer byte, so that we can clobber the real one
        current_buffer_byte = sie->buffer[byte_iter];
        sie->buffer[byte_iter] = 0;
        for (bit_iter = 0; bit_iter < 8; bit_iter++) {
            shift_amount = (7 - bit_iter);
            // current state    current bit     new bit value
            // 0                0               1
            // 0                1               0
            // 1                0               0
            // 1                1               1
            // operation: XNOR!
            // take the nth bit of the current buffer byte and XNOR it with the current state
            new_bit = (current_buffer_byte & (1 << shift_amount)) ^ (((~current_state) & 0x01)
                       << shift_amount);
            sie->buffer[byte_iter] |= new_bit;
            // shift the new bit into the LSB to indicate the current state
            current_state = new_bit >> shift_amount;
        }
    }

    current_state = transient_first_state;
    // iterate bit by bit -- if the current bit is set, toggle the current state
    for (byte_iter = token_packet_size; byte_iter < data_packet_size + token_packet_size; byte_iter++) {
        // copy the current buffer byte, so that we can clobber the real one
        current_buffer_byte = sie->buffer[byte_iter];
        sie->buffer[byte_iter] = 0;
        for (bit_iter = 0; bit_iter < 8; bit_iter++) {
            shift_amount = (7 - bit_iter);
            // current state    current bit     new bit value
            // 0                0               1
            // 0                1               0
            // 1                0               0
            // 1                1               1
            // operation: XNOR!
            // take the nth bit of the current buffer byte and XNOR it with the current state
            new_bit = (current_buffer_byte & (1 << shift_amount)) ^ (((~current_state) & 0x01)
                       << shift_amount);
            sie->buffer[byte_iter] |= new_bit;
            // shift the new bit into the LSB to indicate the current state
            current_state = new_bit >> shift_amount;
        }
    }

    sie->buffer_encoding = NRZI;

}


#ifndef NO_AVR_GCC
void sie_transfer(SIE_T *sie, TRANSFER_TYPE_T type)
// This is where the magic happens.
// Handles transferring data between a device and the host (this).
{
    /* First, the relevant data from the SIE struct is extracted. We make copies so that we
     * can keep from trashing the original data.
     *
     * The fields are:
     * - buffer: The address of the buffer. We will load the first byte of the data packet
     *           from here.
     * - token_packet_length: The size (in bits) of the token packet.
     * - data_packet_length: The size (in bits) of the data packet, if it exists.
     * - data_direction: If 0, we are transmitting the data packet, if 1 we are receiving.
     * - handshake_result: The address where we will store the handshake if we receive one.
     */
    register uint8_t token_length asm("r16") = sie->token_packet_length;
    register uint8_t data_length asm("r17") = sie->data_packet_length;
    register uint8_t dir asm("r7") = sie->data_direction;

    /* These are variables declared solely for the use of the assembly code. They are
     * declared here to make it easier for GCC to find registers to store them in.
     */
    register uint8_t byte_buffer asm("r3") = 0;
    register uint8_t temp asm("r20") = 0;
    register uint8_t timeout_counter asm("r21") = 0;
    register uint8_t constant_5 asm("r10") = 5;
    register uint8_t NACK_byte asm("r11") = 0xf7;
    register uint8_t STALL_byte asm("r12") = 0xc4;
    register uint8_t transfer_type asm("r8") = type;

    // zero out the handshake result
    *(sie->handshake_result) = 0x7;

    /* We now send the device a resume signal before the transfer kicks in. This is in order
     * to wake the device up from suspend, which it most likely has entered as a result of the
     * very long debug statements.
     * 
     * After the resume signal, the SIE will then send a keep alive, which prevents low-speed
     * devices from going to sleep.
     */
    sie_resume(sie);
    for( temp = 0; temp < 25; temp++ ) {
        _delay_ms(1); // Delay for a total of 25 ms.
    } 
    asm volatile ( 
        "ldi r20, 0x04\n\t"
        "out %0, r20\n\t"
        NOP8
        "ldi r20, 0x04\n\t"
        "out %0, r20\n\t"
        NOP8
        "ldi r20, 0x05\n\t"
        "out %0, r20\n\t"
        NOP8
        :: "I" (_SFR_IO_ADDR(USBPORT))
        );
    sie_idle(sie);
    _delay_us(20);
    sie_keepalive(sie);
    sie_idle(sie);
    _delay_us(10);
    
    /* Here begins the assembly for the SIE. This will handle sending and receiving packets both
     * to and from a device. Once we begin the transfer, we have to stick to a pretty tight 10-cycle
     * interval between bits. We depend on the host controller to provide the SIE with enough
     * information so that it does not have to do much thinking. The thousand-foot view of the
     * assembly that follows is this:
     *
     * 1. Perform a small amount of initialization (storing constants into registers, etc.).
     * 2. Send the token packet (including sync and EOP).
     * 3. Decide if we are receiving or sending data and branch to the appropriate handler.
     * 4. 
    */
    asm volatile (
        NOP_INIT // needed for making nop jumps happen.
        "ldi r20, 0x05\n\t"
        "mov r10, r20\n\t"
        "ldi r20, 0x9c\n\t"
        "mov r11, r20\n\t"
        "ldi r20, 0xa0\n\t"
        "mov r12, r20\n\t"

        /* Sending the token packet happens regardless of what type of transfer we are doing,
         * control or interrupt. At most, we'll have to send 6 bytes (with bitstuffing). Once
         * the token packet is began, we then have to adhere to a pretty tight timing schedule:
         * 1 bit sent every 10 cycles.
         */
        ".sie_send_token:\n\t"
        SIE_SEND_SYNC
        "ld r3, X+\n\t" // buffer up the first token byte
        NOP3
        
        ".sie_send_token_bits:\n\t"
        // Send up to 6 bytes
        SIE_TOKEN_BYTE
        SIE_TOKEN_BYTE
        SIE_TOKEN_BYTE
        SIE_TOKEN_BYTE
        SIE_TOKEN_BYTE
        SIE_TOKEN_BYTE
        
        ".sie_send_token_eop:\n\t"
        SIE_SEND_EOP
        // 8 cycles free at this point
        
        /* This is where we differentiate between input and output transfers. A transfer is
         * anywhere from 1-8 bytes. However, we have to be ready to accept a handshake packet 
         * instead of a data packet if the device sends us one. In that case, we don't send 
         * any sort of handshake, and idle. Normally, we will respond with a handshake when we 
         * get all of the data.
         *
         * This little piece of code compares the register where the data direction is stored to 1.
         * If it's equal to 1, we are receiving data. Otherwise, we're transmitting the data.
         */
        "ldi r20, 0x01\n\t"
        "cp r7, r20\n\t"
        "breq .+4\n\t"
        "jmp .sie_tx_data\n\t"
        // If we take the jump: 3 cycles left
        // If we don't take the jump: 5 cycles left
        
        ".sie_rx_data:\n\t"
        // Turn off the transmitter so we can receive.
        "ldi r20, 0x00\n\t"
        "out %1, r20\n\t"

        /* Receiving the sync is a tricky time. We can not know when the device will begin to respond,
         * although we do know that it should be in the window of about 2 to 7.5 bit times (10 cycles
         * each). We assume (unfortunately) that the device *will* respond. This is so we can save
         * cycles when trying to find the first pulse of the sync. This means that if the device does
         * not respond, then we sit in an infinite loop, looking for this first pulse. Not ideal,
         * but it has to do. TODO: FIX THIS
         */
        SIE_RX_SYNC_FIRST 
        SIE_RX_SYNC

        /* At this point, we have a special byte receive for the 8th bit of a data packet. This 
         * is to handle receiving a handshake instead of a data packet as we expected. Otherwise we 
         * jump straight to the regular byte receivers, which handle regular data. This handler is 
         * necessary because a device is allowed to respond to a DATAIN request with a NACK if it 
         * has no data to send us.
         */
        ".sie_rx_control_data_bits:\n\t"
        SIE_DATA_RX_BYTE // byte 0
        SIE_DATA_RX_SPECIAL_BYTE // byte 1, if EOP on bit 0 jump to handshake handler
        SIE_DATA_RX_BYTE // byte 2
        SIE_DATA_RX_BYTE // byte 3
        SIE_DATA_RX_BYTE // byte 4
        SIE_DATA_RX_BYTE // byte 5
        SIE_DATA_RX_BYTE // byte 6
        SIE_DATA_RX_BYTE // byte 7
        SIE_DATA_RX_BYTE // byte 8
        SIE_DATA_RX_BYTE // byte 9
        SIE_DATA_RX_BYTE // byte 10
        SIE_DATA_RX_BYTE // byte 11
        SIE_DATA_RX_BYTE // byte 12

        /* This handles the special situation of an data transfer ending after 1 byte
         * is received. This could be just normal, or it could mean that the function was not ready
         * to send us data, and therefore sent a handshake (NAK or STALL) instead of actual data.
         * To handle this, we'll check the data that we just received and compare it to the two
         * cases given above.
         * 
         * A NAK is identified by 0x5a. The NRZI value is:
         * 01011010 -> (11) 00111001 (NRZI) -> 10011100 (stored in buffer)
         * A STALL is identified by 0x78. The NRZI value is:
         * 01111000 -> (11) 00000101 (NRZI) -> 10100000 (stored in buffer)
         *
         * If we see one of these two values in the buffer, then we've just received a handshake,
         * not data, and we'll store this into the handshake. The data, while stored in the buffer,
         * should be disregarded if there is a NAK or STALL in the handshake buffer. We'll also zero
         * out the data size, as it was a handshake and not actual data.
         */
        ".sie_rx_data_first_eop:\n\t"
        "in r20, %2\n\t"
        "cp r20, __zero_reg__\n\t"
        //"brne .sie_datain_error\n\t"

        "cp r3, r11\n\t"
        "breq .+8\n\t"
        "cp r3, r12\n\t"
        "breq .+4\n\t"
        "jmp .sie_ackout\n\t"
        "st Z, r3\n\t"
        "ldi r17, 0x00\n\t"
        "jmp .sie_done\n\t"

        /* This handles a normal EOP that would be received after a control or interrupt data in
         * finishes, and we don't suspect that it was a special case (as in a 1-byte data
         * in). As we're not able to check the CRC to make sure that it was right, we assume that the
         * data from the device is correct and we received it correctly. This might be a bad 
         * assumption, but it's necessary because the Mega32 is too slow. So we send out an ACK.
         */
        ".sie_rx_data_eop:\n\t"
        "in r20, %2\n\t"
        "cp r20, __zero_reg__\n\t"
        //"brne .sie_datain_error\n\t"
        "jmp .sie_ackout\n\t" // we got the data, send the ack

        ".sie_sync_timeout:\n\t" // TODO: implement this
        "jmp .sie_done\n\t"
        ".sie_datain_error:\n\t" // TODO: implement this also
        "jmp .sie_done\n\t"
        
        /* {{{ Sending a data packet */
        /* This is essentially a copy of sending the token packet, but modified to send more bytes.
         *
         * According to section 7.1.18.1, we need to insert between 2 and 7.5 "bit times" of delay
         * between the EOP of the token packet and the sync of the data packet.
         */
        ".sie_tx_data:"

        /* There needs to be between 20 and 75 cycles of delay between the end of the EOP
         * for the token packet and the sync of the data packet. This will insert a few.
         */
        "cpi r17, 0x00\n\t" // To handle 0-size data packets, we just end. This is mainly for debug.
        "brne .+4\n\t"
        "jmp .sie_done\n\t"
        NOP8
        NOP8
        NOP8
        NOP8
        NOP8
        NOP8
        NOP8

        SIE_SEND_SYNC
        NOP3 // Have 8 cycles, sending a data bit takes 3 cycles before it sends.

        ".sie_send_data_bits:\n\t"
        // Send 13 bytes, the maximum that we could be expected to send.
        "ld r3, X+\n\t"
        SIE_DATA_BYTE // byte 0
        SIE_DATA_BYTE // byte 1
        SIE_DATA_BYTE // byte 2
        SIE_DATA_BYTE // byte 3
        SIE_DATA_BYTE // byte 4
        SIE_DATA_BYTE // byte 5
        SIE_DATA_BYTE // byte 6
        SIE_DATA_BYTE // byte 7
        SIE_DATA_BYTE // byte 8
        SIE_DATA_BYTE // byte 9
        SIE_DATA_BYTE // byte 10
        SIE_DATA_BYTE // byte 11
        SIE_DATA_BYTE // byte 12
        
        ".sie_send_data_eop:\n\t"
        // no nops - needs to send right away.
        SIE_SEND_EOP // 9 cycles left
        "ldi r20, 0x00\n\t"
        "out %1, r20\n\t"
        // This should now "fall" right through to the ackin handler.
        
        /* Handshakes are a tricky issue in this implementation. We don't really have enough time
         * to know if the data was corrupted (CRC doesn't match up) or not, so we don't know
         * whether to send an ACK or NACK. At this point in time, we are forgoing the necessary
         * data checking and just sending an ACK. We may later find out a way to be able to verify
         * the data in the small amount of time that we have before we are required to send the
         * handshake.
         *
         * The amount of time that we have is anywhere from 2 to 7.5 "bit times", which is
         * equivalent to 20 to 75 cycles. This is enough time to calculate a CRC5, but not a CRC16,
         * which is what is needed to check the data packet.
         *
         * Receiving handshakes, however, doesn't have the same problems and we can rely on the 
         * function to send us whatever handshake it will.
         */

        /* We begin catching the sync and handshake immediately after the EOP of the data packet
         * that we just sent.
         */
        ".sie_ackin:\n\t"
        // Clear the buffer, sync up to the sync (ha!) and read the handshake. Then store it.
        "clr r3\n\t"
        SIE_RX_SYNC_FIRST_ACK
        SIE_RX_SYNC
        SIE_HANDSHAKE_RX_BYTE // after this, the handshake is in the buffer register (r3)
        "st Z, r3\n\t"   
        "jmp .sie_done\n\t"

        /* This will be called after a data in that got real data (not a handshake). We wait a little
         * bit, then push out the sync, ack, and EOP. We don't have time to do much else.
         */
        ".sie_ackout:\n\t"
        // Here's some more delay:
        NOP8
        NOP8
        NOP8
        NOP8
        NOP8

        NOP4
        SIE_SEND_SYNC
        NOP7
        SIE_HANDSHAKE_TX_ACK
        NOP7
        SIE_SEND_EOP
        
        /* The transfer is now done. Regardless of if we just received the handshake, or if we
         * sent it, we need to idle the lines. Hopefully the transmit lines will already be idled
         * by the handshake receive code, but for safe measure we do it again here.
         */
        ".sie_done:\n\t"
        NOP8
        "ldi r20, 0x00\n\t"
        "out %1, r20\n\t"

        : /* no other variables that are written */
        : /* read-only variables that we don't assign ourselves */
          "x" ((unsigned char *) sie->buffer),
          "I" (_SFR_IO_ADDR(USBPORT)), /* This and the pin are assigned by the user. */
          "I" (_SFR_IO_ADDR(USBPIN)),
          "z" ((unsigned char *) sie->handshake_result),
          "M" (20), /* Number of bit-times to wait before timing out on receives. */
          "I" (_SFR_IO_ADDR(PORTC))
        );

    /* Load up the correct values into the SIE struct. If we received data, we want to modify
     * the data packet length to reflect how many bits we received. This should be all that is
     * necessary for the interface to decipher the data.
     */
    sie->data_packet_length = data_length;
    sie->token_packet_length += token_length; // THIS IS A HACK

    // We're done transmitting everything, so let's resettle to the idle state.
    sie_idle(sie);

    if( dir != RECEIVE ) *(sie->handshake_result) += token_length; // THIS IS A HACK

}
#endif


#ifndef NO_AVR_GCC
void sie_idle(SIE_T *sie)
// Sets the output lines to the idle "J" state
{
    asm(
        "ldi r16, 0x40\n\t" // blank out the lines.
        "out %0, r16\n\t"
        NOP4
        "ldi r16, 0x00\n\t"
        "out %0, r16\n\t"
        : /* no input registers */
        : "I" (_SFR_IO_ADDR(USBPORT)) /* outputs */
        : "r16" /* clobbered */
        );
}
#endif


#ifndef NO_AVR_GCC
void sie_control_transfer(SIE_T *sie) 
// Runs through a control transfer.
{
    sie_transfer( sie, CONTROL );
}
#endif


#ifndef NO_AVR_GCC
void sie_interrupt_transfer(SIE_T *sie)
// Runs through a interrupt transfer.
{
    sie_transfer( sie, INTERRUPT );
}
#endif


#ifndef NO_AVR_GCC
void sie_keepalive(SIE_T *sie)
// Sends a keep alive signal port, same as a low speed EOP
{
    asm(
        "ldi r20, 0x04\n\t"
        "out %0, r20\n\t"
        NOP8
        "ldi r20, 0x04\n\t"
        "out %0, r20\n\t"
        NOP8
        "ldi r20, 0x05\n\t"
        "out %0, r20\n\t"

        : /* no input registers */
        : /* output registers */
          "I" (_SFR_IO_ADDR(USBPORT))
        : /* clobbers */
          "r20"
        );
    
    // Resume idle state.
    sie_idle(sie);
}
#endif


#ifndef NO_AVR_GCC
void sie_reset(SIE_T *sie)
// Holds a SE0 on the lines. It's up to the user to set the idle state after 10 ms.
{
    asm(
        "ldi r20, 0x04\n\t"
        "out %0, r20\n\t"
        : /* no input registers */
        : "I" (_SFR_IO_ADDR(USBPORT)) /* outputs */
        : "r20" /* clobbered */
        );
}
#endif


#ifndef NO_AVR_GCC
void sie_resume(SIE_T *sie)
// Sends a resume signal. It's again up to the user to set the idle state after 20 ms.
{
    asm(
        "ldi r20, 0x06\n\t" // differential 1 (K)
        "out %0, r20\n\t"
        : /* no input registers */
        : "I" (_SFR_IO_ADDR(USBPORT)) /* output */
        : "r20" /* clobbered */
        );
}
#endif


#ifndef NO_AVR_GCC
uint8_t sie_detect_device(SIE_T *sie)
// Detects if there is a low-speed device attached to the USB port.
{
    char result=0;

    /* There is a device connected (or activity on the lines), but we need to figure out what.
     * The values that we could see:
     * 0x40: Low-speed device connected.
     * 0x80: Full-speed device connected.
     * Else: Nothing we know of.
     */
    asm(    "sbis %1, 6\n\t"
            "ldi %0, 1\n\t"
            "sbis %1, 7\n\t"
            "ldi %0, -1\n\t"
            : "=d" (result) /* result gets written. */
            : "I" (_SFR_IO_ADDR(USBPIN))
        );

    return result;
}
#endif
