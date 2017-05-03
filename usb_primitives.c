// usb_primitives.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width
// See usb_primitives.h for headers.


#include "usb_primitives.h"


/***********/
/* GLOBALS */
/***********/


const uint8_t nibble_mirror[] =
// An array for looking up the mirror of a given nibble. Useful for flipping endian-ness. It's 
// actually a strange case that it goes here. Suffice it to say that gcc handles array linking
// unintuitively.
{
    0x0, // 0: 0000 -> 0000 (symmetrical)
    0x8, // 1: 0001 -> 1000
    0x4, // 2: 0010 -> 0100
    0xc, // 3: 0011 -> 1100
    0x2, // 4: 0100 -> 0010
    0xa, // 5: 0101 -> 1010
    0x6, // 6: 0110 -> 0110 (symmetrical)
    0xe, // 7: 0111 -> 1110
    0x1, // 8: 1000 -> 0001
    0x9, // 9: 1001 -> 1001 (symmetrical)
    0x5, // a: 1010 -> 0101
    0xd, // b: 1011 -> 1101
    0x3, // c: 1100 -> 0011
    0xb, // d: 1101 -> 1011
    0x7, // e: 1110 -> 0111
    0xf  // f: 1111 -> 1111 (symmetrical)
};

uint8_t print_buffer[75];


/************************/
/* FUNCTION DEFINITIONS */
/************************/


// -----------
// - PACKETS -
// -----------


TOKEN_PACKET_T *TokenPacket(PID_T pid, uint8_t addr, uint8_t endp)
// Constructor. Returns a pointer to a token packet created with the passed variables. crc5 and sync
// are not set.
{
    TOKEN_PACKET_T *new_packet;

    new_packet = malloc(sizeof(TOKEN_PACKET_T));
    // initialize it with the passed variables
    new_packet->pid = pid;
    new_packet->addr = addr;
    new_packet->endp = endp;
    return new_packet;
}


void DestroyTokenPacket(TOKEN_PACKET_T *token_packet)
// Destructor. Simply frees the memory allocated for the token packet.
{
    free(token_packet);
}


DATA_PACKET_T *DataPacket(PID_T pid, size_t payload_size)
// Consutrctor. Returns a pointer to a data packet created with payload of the passed variable in
// bytes, as well as allocing space for the data.
{
    DATA_PACKET_T *new_packet;

    new_packet = malloc(sizeof(DATA_PACKET_T));
    // initialize it with the passed variables
    new_packet->pid = pid;
    new_packet->payload_size = payload_size;
    new_packet->data = malloc(sizeof(uint8_t) * payload_size);
    return new_packet;
}


void DestroyDataPacket(DATA_PACKET_T *data_packet)
// Destructor. Frees the payload and the data packet itself.
{
    free(data_packet->data);
    free(data_packet);
}


/*
 * CRC Algorithms:
 *  NOTE: CRC notation represents binary strings a polynomials; e.g. x^n is the nth bit of the 
 *  binary string x, where the bits of x are zero-indexed (LSB is bit 0).
 *
 *  The general formula is: M(x) * x^n = Q(x) * G(x) + R(x)
 *      M(x) is the original message polynomial
 *      M(x) * x^n is the original message with n zeroes appended to the end (binary multiplication)
 *      G(x) is the degree-n generator polynomial
 *      R(x) is the remainder polynomial, which is used as the CRC checksum
 *      Q(x) is the quotient polynomial (not used for anything)
 *
 *  The sender transmits the message as: (M(x) * x^n) | R(x), putting the remainder in the place
 *  of the zeroes.
 *  The receiver checks whether M(x) * x^n - R(x) is divisible by G(x). If it is, the receiver
 *  assumes that the received message is correct.
 *
 * To decode:
 *      2. CRC16 -- M(x) is 8 bytes (64 bits) maximum
 *          a) Receiver gets [recvd = (M(x) * x^16 + R(x))] -- 80 bits maximum
 *          b) M(x) * x^16 - R(x): [hash_check = recvd & 0x0000 - recvd & 0xffff]
 *          c) if ([hash_check / G(x)] == 0) CRC good
 *          d) else CRC bad
 */


void packet_make_crc5(TOKEN_PACKET_T *token_packet)
// Calculates a CRC5 for the token packet, and places the result in the packet's CRC5 field.
{
    uint16_t input;

    // extract relevant fields
    input = (flip_byte(token_packet->addr, ADDR_BITS) << ENDP_BITS) | 
             flip_byte(token_packet->endp, ENDP_BITS);
    // calculate crc, and place in packet
    token_packet->crc5 = make_crc5(input, ENDP_BITS + ADDR_BITS);
}


uint8_t packet_validate_crc5(TOKEN_PACKET_T *token_packet)
// Validates that the CRC5 for a packet is correct. Returns 0 for incorrect, non-zero for
// correct.
{
    uint16_t input;

    input = (token_packet->addr << ENDP_BITS) | token_packet->endp;
    return token_packet->crc5 == make_crc5(input, ENDP_BITS + ADDR_BITS);
}


uint16_t make_crc5(uint16_t input, uint8_t bit_count)
{
    const uint16_t crc_number = 5; // for crc5
    // the polynomial coefficients
    const uint8_t poly = 0x05; 
    // all 1s of the same length as the polynomial
    const uint8_t crc = 0x1f; 
    // mask for integer's most significant bit
    const uint16_t int_msbit = (1 << (sizeof(uint16_t) * 8 - 1)); 
    // contains the polynomial divisor in its most significant bits
    const uint16_t poly_msb = (poly << (sizeof(uint16_t) * 8 - crc_number));

    // starts as a bunch of 1's in its most significant bits, ends up being the crc value
    uint16_t crc_output = (crc << (sizeof(uint16_t) * 8 - crc_number));
    // the input value, shifted to the most significant bits
    uint16_t input_msb = (input << (sizeof(uint16_t) * 8 - bit_count));

    // iterate through the bits of the input, dividing essentially by polynomial long division
    while (bit_count-- > 0) {
        // check to see if the MSB of the current state of the input is a 1...
        if ( (input_msb ^ crc_output) & int_msbit ) { 
            // shift out the MSB from the remainder
            crc_output <<= 1;
            // subtract the polynomial coefficients (divisor) from the remainder (dividend)
            crc_output ^= poly_msb;
        }
        else { // MSB is a 0
            // just shift out the MSB
            crc_output <<= 1;
        }
        // and then shift out the input's MSB
        input_msb <<= 1;
    }

    // shift back into position
    // shift the remainder back to fill up the least significant bits
    crc_output >>= (sizeof(uint16_t) * 8 - crc_number);

    // invert contents to generate crc field
    return crc_output ^ crc;
}


void packet_make_crc16(DATA_PACKET_T *dp)
// Calculates a CRC16 for the data packet, and places the result in the packet's CRC16 field.
{
    uint8_t data_packet_array[dp->payload_size];
    uint8_t iter;

    for (iter = 0; iter < dp->payload_size; iter++) {
        data_packet_array[iter] = flip_byte(dp->data[iter], CHAR_BITS);
    }
    dp->crc16 = make_crc16(data_packet_array, dp->payload_size * CHAR_BITS);
}


uint8_t packet_validate_crc16(DATA_PACKET_T *dp)
// Validates that the CRC16 for a packet is correct.
{
    // TODO: why is this char * cast necessary?
    return (dp->crc16 == make_crc16((char *) dp->data, dp->payload_size * CHAR_BITS));
}


uint16_t make_crc16(uint8_t *input, uint16_t bit_count)
{
    #define crc_number 16 // for crc16
    #define crc 0xffff // all 1's of the same length as the polynomial
    #define char_msbit (1 << (CHAR_BITS - 1))

    size_t num_bytes = ((bit_count-1) >> 3) + 1; // number of bytes in input array

    uint8_t *poly_msb;
    uint8_t *crc_output;
    uint8_t *input_msb;
    uint16_t crc_return;
    uint8_t i;

    // poly_msb contains the polynomial divisor in its most significant bits
    poly_msb = calloc(num_bytes, sizeof(uint8_t));
    poly_msb[num_bytes - 1] = 0x80;
    poly_msb[num_bytes - 2] = 0x05;

    // starts as a bunch of 1's in its most significant bits, ends up being the crc value
    crc_output = calloc(num_bytes, sizeof(uint8_t));
    crc_output[num_bytes - 1] = 0xff;
    crc_output[num_bytes - 2] = 0xff;

    // input_msb is made a local copy of the input
    input_msb = calloc(num_bytes, sizeof(uint8_t));
    
    // for the last byte we have to shift the least significant bits into the most significant bits
    for(i = 0; i < num_bytes; i++) {
      input_msb[i] = input[i];
    }
   
    // iterate through the bits of the input, dividing essentially by polynomial long division
    while (bit_count-- > 0) {
        // check to see if the MSB of the current state of the input is a 1...
        if ((input_msb[num_bytes - 1] ^ crc_output[num_bytes - 1]) & char_msbit) {
            // shift out the MSB from the remainder
            for(i = num_bytes - 1; i > 0; i--) {
                crc_output[i] = (crc_output[i] << 1) | (crc_output[i-1] >> (CHAR_BITS - 1));
            }
            crc_output[0] <<= 1;

            // subtract the polynomial coefficients (divisor) from the remainder (dividend)
            for(i = 0; i < num_bytes; i++) {
                crc_output[i] ^= poly_msb[i];
            }
        }
        else { // MSB is a 0
            // just shift out the MSB
            for(i = num_bytes - 1; i > 0; i--) {
                crc_output[i] = (crc_output[i] << 1) | (crc_output[i-1] >> (CHAR_BITS - 1));
            }
            crc_output[0] <<= 1;
        }
        // and then shift out the input's MSB
        for( i = num_bytes - 1; i > 0; i-- ) {
            input_msb[i] = (input_msb[i] << 1) | (input_msb[i-1] >> (CHAR_BITS - 1));
        }
        input_msb[0] <<= 1;
    }

    // Shift back into position
    crc_return = crc_output[num_bytes - 2] | (crc_output[num_bytes - 1] << 8); // put the remainder into an int

    return crc_return ^ crc; // invert contents to generate crc field

    #undef char_msbit
    #undef crc
    #undef crc_number
}


// ----------------
// - TRANSACTIONS -
// ----------------


TRANSACTION_T *Transaction(TRANSFER_TYPE_T transfer_type, TOKEN_PACKET_T *tp, DATA_PACKET_T *dp)
// Constructor. Returns a pointer to a transaction that wraps the token packet, data packet, and
// encoding. Sets the handshake to undetermined.
{
    TRANSACTION_T *return_transaction;

    return_transaction = malloc(sizeof(TRANSACTION_T));
    // initialize it with the passed variables
    return_transaction->transfer_type = transfer_type;
    return_transaction->token_packet = tp;
    return_transaction->data_packet = dp;
    return_transaction->handshake = UNDETERMINED; // undetermined handshake type on creation
    return return_transaction;
}


void DestroyTransaction(TRANSACTION_T *transaction)
// Destructor. Frees the transaction pointer, leaving the packets unaffected.
// See the DestroyWholeTransaction destructor for the ability to destroy the packets as well.
{
    if (transaction) free(transaction);
}


void DestroyWholeTransaction(TRANSACTION_T *transaction)
// Destructor. Destroys the packets which constitute the transaction in addition to destroying
// the transaction itself.
{
    if (transaction->token_packet) DestroyTokenPacket(transaction->token_packet);
    if (transaction->data_packet) DestroyDataPacket(transaction->data_packet);
    DestroyTransaction(transaction);
}


TRANSACTION_NODE_T *TransactionNode(TRANSACTION_T *transaction, TRANSACTION_NODE_T *prev,
                                    TRANSACTION_NODE_T *next)
// Constructor. Returns a pointer to a transaction node that wraps transaction, prev, and next.
{
    TRANSACTION_NODE_T *new_transaction_node;

    new_transaction_node = malloc(sizeof(TRANSACTION_NODE_T));
    new_transaction_node->ptr = transaction;
    new_transaction_node->prev = prev;
    new_transaction_node->next = next;
    return new_transaction_node;
}


void DestroyTransactionNode(TRANSACTION_NODE_T *transaction_node)
// Destructor. Frees up the node memory, as transaction nodes are just wrappers for more useful
// transaction pointers. Does not destroy transactions.
{
    free(transaction_node);
}


inline uint8_t flip_byte(uint8_t original_byte, uint8_t relevant_bits)
// Returns a flipped copy of the passed byte. To flip a byte, you are essentially mirroring the 
// LSB nibble and placing it in the MSB nibble's place, and mirroring the MSB nibble and placing it 
// in the MSB nibble's place.
{
    #define nibble_bits 4
    #define lower_nibble(byte) (byte & 0xf)
    #define upper_nibble(byte) (byte >> nibble_bits)

    uint8_t mirrored_byte;
    
    mirrored_byte = ((nibble_mirror[lower_nibble(original_byte)] << nibble_bits) | // upper portion
                     nibble_mirror[upper_nibble(original_byte)]); // lower portion
    return (mirrored_byte >> (CHAR_BITS - relevant_bits));
    #undef upper_nibble
    #undef lower_nibble
    #undef nibble_bits
}


void transaction_place_in_buffer(TRANSACTION_T *transaction, uint8_t *buffer, 
                                 uint8_t *token_packet_length_ptr, uint8_t *data_packet_length_ptr)
// WARNING: assumes the buffer is large enough!
// TODO: comment for this function.
{
    uint8_t flipped_temp;
    uint8_t byte_num;
    uint8_t bit_num;
    uint8_t roundoff;
    uint16_t iter;

    // ***********
    // * WARNING *
    // ***********
    // Do not use the non-pointer macro for pointers! This macro calls the
    // transaction_place_in_buffer_helper function with the appropriate value by /referencing/ the
    // data parameter and casting it to char *.

    #define pointer_helper_macro(data_ptr, size_bits) \
        transaction_place_in_buffer_helper(buffer, (uint8_t *)data_ptr, size_bits, &byte_num, \
                                           &bit_num);
    #define non_pointer_helper_macro(data, size_bits) \
        pointer_helper_macro(&data, size_bits)

    byte_num = 0;
    bit_num = 0;
    roundoff = 0;

    // token packet
    non_pointer_helper_macro(transaction->token_packet->pid, PID_BITS);
    flipped_temp = flip_byte(transaction->token_packet->addr, ADDR_BITS);
    non_pointer_helper_macro(flipped_temp, ADDR_BITS);
    flipped_temp = flip_byte(transaction->token_packet->endp, ENDP_BITS);
    non_pointer_helper_macro(flipped_temp, ENDP_BITS);
    non_pointer_helper_macro(transaction->token_packet->crc5, CRC5_BITS);

    if (token_packet_length_ptr) *token_packet_length_ptr = byte_num * 8 + bit_num;
    if (data_packet_length_ptr) *data_packet_length_ptr = 0;
    
    // data packet, if it exists (it could be absent in the case of an IN transaction, for example
    if (transaction->data_packet && transaction->data_packet->payload_size) {

        if (bit_num != 0) {
            // round off to the next byte for the start of the data packet
            byte_num++;
            roundoff = 8 - bit_num;
            bit_num = 0;
        }

        non_pointer_helper_macro(transaction->data_packet->pid, PID_BITS);

        // NOTE: Assumes that payload size is in bytes.
        // NOTE: Puts least significant byte in the buffer first. This is based on the following
        //  information from http://www.usbmadesimple.co.uk/ums_3.htm:
        //  "if, for example, a field is defined by 2 successive bytes, the first byte will be the 
        //   least significant, and the second byte transmitted will be the most significant."
        // NOTE: This also assumes that the value in the lowest index in the array is the least
        //  significant byte.
        for (iter = 0; iter < transaction->data_packet->payload_size; iter++) {
            flipped_temp = flip_byte(transaction->data_packet->data[transaction->data_packet->payload_size - iter - 1], CHAR_BITS);
            non_pointer_helper_macro(flipped_temp, CHAR_BITS);
        }

        // the crc16 has to be done in two bytes
        flipped_temp = transaction->data_packet->crc16 >> 8;
        non_pointer_helper_macro(flipped_temp, CHAR_BITS);
        flipped_temp = transaction->data_packet->crc16  & 0xff;
        non_pointer_helper_macro(flipped_temp, CHAR_BITS);

        if (data_packet_length_ptr && token_packet_length_ptr)
            *data_packet_length_ptr = byte_num * 8 + bit_num - *token_packet_length_ptr - roundoff;
    }

    #undef non_pointer_helper_macro
    #undef pointer_helper_macro
}


void transaction_place_in_buffer_helper(uint8_t *buffer_arr, uint8_t *data_arr, 
                                        uint16_t data_bits, uint8_t *buffer_byte_offset, 
                                        uint8_t *buffer_bit_offset)
// Takes an arbitrary amount of data in the following format:
// data_arr[n] = Most Significant Byte
// data_arr[n-1] = Next Most Significant Byte
// ...
// data_arr[0] = Least Significant Byte
// Any fractions of a byte are contained within the Most Significant Byte, and are aligned to the 
// LSB
// NOTE: This function is written for clarity due to its inherently confusing nature. I'm hoping GCC
// will do most of the optimization that I left out. I went with the Pythonic mentality on this one.
{
    size_t data_bytes;
    uint8_t fractional_data_bits;
    signed char leftover_fractional_data_bits;
    int iter;

    // calculate the number of data bytes from the number of data bits
    // for 8 bits: ((8 - 1) >> 3) + 1 = (7 >> 3) + 1 = 1
    // for 9 bits: ((9 - 1) >> 3) + 1 = (8 >> 3) + 1 = 2
    // that should be enough evidence that this expression works
    data_bytes = ((data_bits - 1) >> 3) + 1;

    // calculate the number of bits that don't fit nicely into a byte; effectively, (data_bits % 8)
    // NOTE: if this isn't zero, the fractional bits are assumed to be in the Most Significant Byte
    fractional_data_bits = data_bits & (CHAR_BITS - 1);

    // handle the most significant byte first as a special case
    // first, determine if there are fractions of a byte
    if (fractional_data_bits)
    // there is a fraction of a byte in the Most Significant Byte, so we have to handle this as an 
    // exceptional case
    {
        // first, clear out the rest of the buffer byte
        buffer_arr[*buffer_byte_offset] &= ~(0xff >> *buffer_bit_offset);

        // left align the fractional bits, then shift them to the right buffer_bit_offset to get it
        // fitting into the buffer appropriately
        buffer_arr[*buffer_byte_offset] |= data_arr[data_bytes - 1] 
                                           << (CHAR_BITS - fractional_data_bits) 
                                           >> *buffer_bit_offset;

        // now we have to consider that our buffer_bit_offset might have been larger than our 
        // fractional data bits; e.g. if our buffer_bit_offset was 5 and we had 5 fractional data 
        // bits, we'll have 2 bits left to put somewhere -- if this is the case, we have to handle 
        // that those bits
        leftover_fractional_data_bits = *buffer_bit_offset + fractional_data_bits - CHAR_BITS;

        if (leftover_fractional_data_bits == 0) {
            // this means that the buffer bit offset and fractional data bits combined to make 
            // exactly one new byte! our new bit offset is then zero, but we have to increment the 
            // buffer byte offset
            (*buffer_byte_offset)++;
            *buffer_bit_offset = leftover_fractional_data_bits;
        }
        else if (leftover_fractional_data_bits > 0) {
            // we'll just left align the leftover bits, place them into the next byte, and change 
            // the buffer_bit_offset for all subsequent bytes
            (*buffer_byte_offset)++;
            buffer_arr[*buffer_byte_offset] = data_arr[data_bytes - 1] << 
                                             (CHAR_BITS - leftover_fractional_data_bits);
            *buffer_bit_offset = leftover_fractional_data_bits;
        }
        else {
            // there are still bits remaining in the current byte! consequently, there is no need to
            // increment the buffer_byte_offset, since we can still use bits in the current byte. in
            // this case, the number of bits used in the current byte is simply 
            // buffer_bit_offset + fractional_data_bits, and we should set the buffer bit offset to 
            // reflect that accordingly
            *buffer_bit_offset = *buffer_bit_offset + fractional_data_bits;
        }
    }
    else
    // our Most Significant Byte is a full 8 bits!
    {
        // there are no fractions of a byte in the Most Significant Byte of the data, so we can 
        // just place as much of it as we can in the current buffer byte, and put the rest in the 
        // next byte

        // clear out the bits we'll be writing to
        buffer_arr[*buffer_byte_offset] &= ~(0xff >> *buffer_bit_offset);

        // 'or' in the most significant bits of this byte
        buffer_arr[*buffer_byte_offset] |= data_arr[data_bytes - 1] >> *buffer_bit_offset;

        // increment to the next byte offset, since we just filled the current one up
        (*buffer_byte_offset)++;
        if (*buffer_bit_offset) {
            // if the buffer bit offset isn't zero, then we couldn't fit the whole first byte into 
            // the original byte offset (remember that we know that our most significant byte is a 
            // full 8 bits) as a result, we have to get the (buffer_bit_offset many) LSB that we 
            // lost above and right align them in the next byte
            buffer_arr[*buffer_byte_offset] = data_arr[data_bytes - 1]
                                               << (CHAR_BITS - *buffer_bit_offset);

            // seeing as how we just put a full 8 bits into the buffer, the bit offset remains the 
            // same
        }
    }

    // zero out the bits that we will be 'or'-ing with
    // see in-loop comments for why we don't have to zero out these bits every iteration
    // this is done only as a prologue
    buffer_arr[*buffer_byte_offset] &= ~(0xff >> *buffer_bit_offset);

    // now, the iterative case: we have a buffer bit offset and a number of full bytes of data to 
    // put in the buffer; iterate over all the rest of the bytes (besides the Most Significant Byte,
    // which resides at index data_bytes-1)
    for (iter = data_bytes - 2; iter > 0; iter--) {
        // place the MSBits of the current byte
        buffer_arr[(*buffer_byte_offset)] |= data_arr[iter] >> *buffer_bit_offset;
        // increment to the next buffer byte
        (*buffer_byte_offset)++;
        // right align the LSBits of the current byte that got cut off above
        // NOTE: this actually zeroes out the less significant bits of this byte, thus removing the
        // need for us to zero out the bits in the beginning of the loop.
        buffer_arr[*buffer_byte_offset] = data_arr[iter] << (CHAR_BITS - *buffer_bit_offset);
    }
}


// ---------
// - Debug -
// ---------


#ifndef NO_AVR_GCC // this won't work without <avr/io.h>

void usart_putchar(uint8_t char_to_put)
// Waits for transmit flag to become clear, then sends a single byte.
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = char_to_put;
}


void usart_putstr(char *str_to_put)
// Transmits the given string over USART.
{
    uint8_t iter;
    uint8_t char_to_put;
    for (iter = 0; (char_to_put = str_to_put[iter]); iter++) {
        usart_putchar(char_to_put);
    }
}

void send_debug()
// Sends whatever is in the print buffer out on the USART. Takes awhile.
{
    #define USART_BAUDRATE 9600
    #define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

    uint8_t index;
    
    index = 0;

    // enable usart for transmit and receive
    UCSRB |= (1 << TXEN);   
    // 8-bit chars (uint8_t)
    UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 

    // load lower 8-bits of the baud rate value into the low byte of the UBRR register
    UBRRL = BAUD_PRESCALE; 
    // load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRRH = (BAUD_PRESCALE >> 8); 

    usart_putstr(print_buffer);

    _delay_ms(10);

    #undef USART_BAUDRATE
    #undef BAUD_PRESCALE
}
#endif


uint16_t bitstuff_buffer(uint8_t **buf_ptr, uint16_t buf_used_bit_len, STUFF_METHOD_T stuff_method)
// Given a pointer to an existing buffer and a number of used bits in that buffer, modifies the 
// buffer pointer to point to a malloc'd, bit stuffed copy of the buffer, and returns the used bit 
// length of the new buffer.
{
    #define byte_index(bit_number) (bit_number ? ((bit_number) >> 3) : 0)
    #define mod_eight(value) ((value) & 0x7)

    uint8_t *buf;
    uint8_t *new_buf;
    // Number of bit stuffs performed.
    uint16_t bit_stuff_count;
    // An iterator, indicating which bit we're currently analyzing in the original buffer.
    uint16_t bit_iter;
    uint8_t new_buf_shift_amount;
    uint8_t consecutive_one_count;
    uint8_t buf_bit_mask;
    uint8_t current_bit;

    new_buf = NULL;
    new_buf_shift_amount = 0;

    buf = *buf_ptr;
    // Make a buffer about double the size of the original -- really just need additional bits / 6,
    // but extra won't hurt much.
    if (stuff_method == STUFF) 
        new_buf = calloc(sizeof(uint8_t), 2 * byte_index(buf_used_bit_len) + 1);
    else if (stuff_method == UNSTUFF)
        new_buf = calloc(sizeof(uint8_t), 2 * byte_index(buf_used_bit_len) + 1);

    consecutive_one_count = 0;
    bit_stuff_count = 0;

    for (bit_iter = 0; bit_iter < buf_used_bit_len; bit_iter++)
    // Iterate over each bit in the original buffer.
    {
        buf_bit_mask = 1 << (7 - mod_eight(bit_iter));
        if (stuff_method == STUFF)
            new_buf_shift_amount = 7 - mod_eight(bit_iter + bit_stuff_count);
        else if (stuff_method == UNSTUFF)
            new_buf_shift_amount = 7 - mod_eight(bit_iter - bit_stuff_count);

        current_bit = buf[byte_index(bit_iter)] & buf_bit_mask;

        if (current_bit)
        // The bit we're analyzing is set. Note that current_bit is not necessarily in the 1s place.
        {
            // Note that this bit was set for future iterations by incrementing the consecutive bit 
            // count.
            consecutive_one_count++;

            // Since we know the bit was set, we can take a one, shift it into the appropriate bit 
            // place, and logical-or it with the calloc'd byte value in the temporary buffer, 
            // setting the most significant unused bit.
            if (stuff_method == STUFF)
                new_buf[byte_index(bit_iter + bit_stuff_count)] |= 1 << new_buf_shift_amount;
            else if (stuff_method == UNSTUFF)
                new_buf[byte_index(bit_iter - bit_stuff_count)] |= 1 << new_buf_shift_amount;

            if (consecutive_one_count == 6) {
                // We have to bit stuff -- fortunately, this is as easy as incrementing the bit
                // stuff counter! This will cause the index into the temporary buffer to increase,
                // and (because it was calloc'd) this is effectively inserting a zero in the array.
                bit_stuff_count++;
                // Don't forget that we have to reset the consecutive bit count after bit stuffing.
                consecutive_one_count = 0;
            }
        }
        else {
            // We don't have to do anything except reset the consecutive one count, as the bit_iter 
            // will increment for the next iteration, and the current bit in the temp buffer is 
            // already unset, since we calloc'd it originally.
            consecutive_one_count = 0;
        }

    }

    if (stuff_method == STUFF) {
        // Realloc the new buffer to more appropriately reflect the larger size.
        new_buf = realloc(new_buf, sizeof(uint8_t) * 
                (((buf_used_bit_len + bit_stuff_count - 1) >> 3) + 1));
    }

    // Modify the buffer pointer passed in to point to our newly made buffer.
    *buf_ptr = new_buf;

    if (stuff_method == STUFF) {
        // Return the new size, which is the original size plus the number of bit stuffs performed.
        return (buf_used_bit_len + bit_stuff_count);
    }
    else if (stuff_method == UNSTUFF) {
        return (buf_used_bit_len - bit_stuff_count);
    }
    return 0;

    #undef byte_index
}
