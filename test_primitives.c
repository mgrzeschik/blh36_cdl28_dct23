// primitives_regression_test.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


// NOTE: To add tests, simply state an equivalence condition, and logical and (&&) that with the
// "valid" variable. The idea of this is that if any test fails, the function returns 0, 
// corresponding to an invalid result.


#include "usb_primitives.h"


uint8_t test_bitstuff_buffer() {

    uint8_t valid = 1;
    uint8_t *original_buf, *new_buf;
    uint16_t original_buf_used_bit_len, new_buf_used_bit_len;
    uint8_t **buf_ptr;
    uint8_t valid_iter;

    original_buf = malloc(2);
    original_buf[0] = 0xff;
    original_buf[1] = 0xf0;
    original_buf_used_bit_len = 12;

    // 1111 1111 1111
    // original: 0xff 0xf0 (12)
    // 1111 1101 1111 10
    // new:      0xfd 0xf8 (14)
    // 1111 1101 0111 10
    // getting:  0xfd 0x78 (14)

    // STUFFING

    buf_ptr = &original_buf;
    new_buf_used_bit_len = bitstuff_buffer(buf_ptr, original_buf_used_bit_len, STUFF);
    new_buf = *buf_ptr;

    valid_iter = (new_buf[0] == 0xfd);
    if (!valid_iter) printf("new_buf[%d] -- value 0x%x, expected 0x%x\n", 0, new_buf[0], 0xfd);
    valid = valid && valid_iter;
    
    valid_iter = (new_buf[1] == 0xf8);
    if (!valid_iter) printf("new_buf[%d] -- value 0x%x, expected 0x%x\n", 1, new_buf[1], 0xf8);
    valid = valid && valid_iter;
    
    valid_iter = (new_buf_used_bit_len == 14);
    if (!valid_iter) 
        printf("new_buf_used_bit_len -- value %d, expected %d\n", new_buf_used_bit_len, 14);
    valid = valid && valid_iter;
    
    // UNSTUFFING

    buf_ptr = &new_buf;
    new_buf_used_bit_len = bitstuff_buffer(buf_ptr, new_buf_used_bit_len, UNSTUFF);
    new_buf = *buf_ptr;
    
    valid_iter = (new_buf[0] == 0xff);
    if (!valid_iter) printf("new_buf[%d] -- value 0x%x, expected 0x%x\n", 0, new_buf[0], 0xfd);
    valid = valid && valid_iter;
    
    valid_iter = (new_buf[1] == 0xf0);
    if (!valid_iter) printf("new_buf[%d] -- value 0x%x, expected 0x%x\n", 1, new_buf[1], 0xf8);
    valid = valid && valid_iter;
    
    valid_iter = (new_buf_used_bit_len == 12);
    if (!valid_iter) 
        printf("new_buf_used_bit_len -- value %d, expected %d\n", new_buf_used_bit_len, 14);
    valid = valid && valid_iter;

    return valid;
}


char* itoa(int val, int base){
	
	static char buf[32] = {0};
	
	int i = 30;
	
	for(; val && i ; --i, val /= base)
	
		buf[i] = "0123456789abcdef"[val % base];
	
	return &buf[i+1];
	
}


void dump_crc5_to_file(uint8_t crc5_outcome)
/* 
 * Takes all possible 11-bit values and dumps them and their corresponding CRC5s to file if their 
 * value is 6 
 */
{
    uint16_t iter;
    FILE *f;

    f = fopen("dump.crc5", "w");
    for (iter = 0; iter < (1UL << 11); iter++) {
        if (make_crc5(iter, 11) == crc5_outcome) fprintf(f, "%s\n", itoa(iter, 2));
    }
    fclose(f);
}


unsigned char test_flip_byte()
{
    unsigned char valid = 1;
    unsigned char iter;

    iter = (flip_byte(0x80, CHAR_BITS) == 0x01);
    valid = valid && iter;

    return valid;
}


unsigned char test_make_crc5()
// Tests the function: unsigned int make_crc5(unsigned int input, unsigned int bit_count);
{
    unsigned char valid = 1;
    unsigned int test_int;
    uint8_t iter;

    // 11-bit CRC5 test
    test_int = 0x547;
    valid = valid && (make_crc5(test_int, 11) == 0x17);
	
    test_int = 0x55A;
    valid = valid && (make_crc5(test_int, 11) == 0x6);
    if (!iter) printf("packet's crc5 invalid. value: 0x%x, target: 0x%x\n", test_int, 0x6);

    return valid;
}


unsigned char test_packet_make_crc5()
// Tests the function: void packet_make_crc5(TOKEN_PACKET_T *token_packet);
{
    unsigned char valid = 1;
    unsigned char iter;
    unsigned char target;
    TOKEN_PACKET_T *tp;

    tp = TokenPacket(SETUP, 0x15, 0xe);
    packet_make_crc5(tp);
    target = 0x17;
    iter = (tp->crc5 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc5 invalid. value: 0x%x, target: 0x%x\n", tp->crc5, target);
    DestroyTokenPacket(tp);

    tp = TokenPacket(OUT, 0x3a, 0xa);
    packet_make_crc5(tp);
    target = 0x1c;
    iter = (tp->crc5 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc5 invalid. value: 0x%x, target: 0x%x\n", tp->crc5, target);
    DestroyTokenPacket(tp);

    tp = TokenPacket(IN, 0x70, 0x4);
    packet_make_crc5(tp);
    target = 0x0e;
    iter = (tp->crc5 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc5 invalid. value: 0x%x, target: 0x%x\n", tp->crc5, target);
    DestroyTokenPacket(tp);

    return valid;
}


unsigned char test_make_crc16()
// Tests the function: unsigned int make_crc16(char* input, unsigned int bit_count);
{
    unsigned char valid = 1;
    char test_char_arr[] = { 0x91, 0xe6, 0xa2, 0xc4 };

    // 32-bit CRC16 test
    valid = valid && (make_crc16(test_char_arr, 32) == 0x7038);

    return valid;
}


uint8_t test_transaction_place_in_buffer()
/* 
 * Tests the function: void transaction_place_in_buffer
 * NOTE: depends on crc5 being accurate.
 */
{
    #define BUFFER_LEN 20
    unsigned char valid = 1;
    unsigned char iter;
    unsigned char buffer[BUFFER_LEN];
    // transaction variables
    TRANSACTION_T *transaction;
    TOKEN_PACKET_T *tp;
    DATA_PACKET_T *dp;

    // initialize the elements of the buffer to zero
    for (iter = 0; iter < BUFFER_LEN; iter++) buffer[iter] = 0;
    tp = TokenPacket(IN, 2, 3); // addr is 2 (7 bits), endp is 3 (4 bits)
    packet_make_crc5(tp); // crc: 0x16
    dp = NULL;
    transaction = Transaction(CONTROL, tp, dp);
    transaction_place_in_buffer(transaction, buffer, NULL, NULL);
    // should be length of token packet (3 bytes), since there is no data packet
    // field value:      0x96        0x02       0x3     0x0c
    //                  (1001 0110) (0000 010)(0 011)(0 1100)
    // fields reversed: (1001 0110)     (0100   000)(1    100)(0  0110)
    //                     0x96              0x20       0xc     0x06  
    // 
    // final result:       0x69                0x41          0x86
    iter = (buffer[0] == 0x96);
    valid = valid && iter;
    if (!iter) printf("buffer[0] invalid. value: 0x%x\n", buffer[0]);
    iter = (buffer[1] == 0x41);
    valid = valid && iter;
    if (!iter) printf("buffer[1] invalid. value: 0x%x\n", buffer[1]);
    iter = (buffer[2] == 0x86);
    valid = valid && iter;
    if (!iter) printf("buffer[2] invalid. value: 0x%x\n", buffer[2]);

    DestroyWholeTransaction(transaction);

    return valid;
}


uint8_t test_packet_make_crc16()
// Tests the function: void packet_make_crc16(DATA_PACKET_T *data_packet);
{
    unsigned char valid = 1;
    unsigned char iter;
    unsigned int target;
    DATA_PACKET_T *dp;

    dp = DataPacket(DATA0, 4);
    dp->data[3] = 0x0;
    dp->data[2] = 0x1;
    dp->data[1] = 0x2;
    dp->data[0] = 0x3;
    packet_make_crc16(dp);

    // 1111 0111 0101 1110 -- 0xf75e
    target = 0xf75e;
    iter = (dp->crc16 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc16 invalid. value: 0x%x, target: 0x%x\n", dp->crc16, target);
    DestroyDataPacket(dp);

    dp = DataPacket(DATA1, 4);
    dp->data[3] = 0x23;
    dp->data[2] = 0x45;
    dp->data[1] = 0x67;
    dp->data[0] = 0x89;
    packet_make_crc16(dp);

    // 0111 0000 0011 1000
    target = 0x7038;
    iter = (dp->crc16 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc16 invalid. value: 0x%x, target: 0x%x\n", dp->crc16, target);
    DestroyDataPacket(dp);
	
	dp = DataPacket(DATA1, 8);
    dp->data[7] = 0x12;
    dp->data[6] = 0x01;
    dp->data[5] = 0x10;
    dp->data[4] = 0x01;
    dp->data[3] = 0x00;
    dp->data[2] = 0x00;
    dp->data[1] = 0x00;
    dp->data[0] = 0x08;
    packet_make_crc16(dp);

    target = 0x7711;
    iter = (dp->crc16 == target);
    valid = valid && iter;
    if (!iter) printf("packet's crc16 invalid. value: 0x%x, target: 0x%x\n", dp->crc16, target);
    DestroyDataPacket(dp);

    return valid;
}


int main()
// Runs the desired regression tests and displays the result of each.
{
    printf("test_make_crc5()          ");
    if (test_make_crc5()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    printf("test_make_crc16()         ");
    if (test_make_crc16()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    //printf("test_make_crc16()         ");
    //if (transaction_place_in_buffer_test()) printf("Passed: transaction_place_in_buffer_test()\n");
    //else printf("!!!FAILED!!!: transaction_place_in_buffer_test()\n");

    printf("test_packet_make_crc5()     ");
    if (test_packet_make_crc5()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    printf("test_packet_make_crc16()    ");
    if (test_packet_make_crc16()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    printf("test_flip_byte()            ");
    if (test_flip_byte()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    printf("test_bitstuff_buffer()      ");
    if (test_bitstuff_buffer()) printf("passed\n");
    else printf("!!!FAILED!!!\n");

    return 0;
}
