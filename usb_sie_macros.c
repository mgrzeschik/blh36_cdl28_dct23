/* This file contains assembly macros used
 * in usb_interface.c. These are just too long
 * to fit comfortably in that file.
 */

/* Remember, we have only the three states of the lines in USB:
 * SEO: both D+ and D- low.
 *     - TX: 0x40
 *     - RX: 0x00
 * D1: differential 1, D+ high and D- low (K).
 *     - TX: 0x06
 *     - RX: 0x80
 * D0: differential 0, D+ low and D- high (J).
 *     - TX: 0x05
 *     - RX: 0x40
 * Pinout of the mcu:
 *     - P0: D- (transmit)
 *     - P1: D+ (transmit)
 *     - P2: TX enable
 *     - P6: D- (receive)
 *     - P7: D+ (receive)
 */

/* Registers that get loaded:
 * X: address of buffer 
 * Z: address of handshake 
 * %1: portx
 * %2: pinx
 * r3: buffer that we use when loading from memory.
 * r7: data direction.
 * r8: transfer type (control=0, interrupt=1)
 * r10: holds the value 5, used for transmitting data.
 * r11: holds the value of a NACK pid.
 * r12: holds the value of a STALL pid.
 * r16: length of token packet.
 * r17: length of data packet.
 * r20: temporary register for our use.
 * r21: timeout counter.
 */

/* {{{ Sync packet macros */
// Sync: KJKJKJKK
/* {{{ Transmitting a sync */
// There are 8 free cycles after sending the sync.
// It uses 1 cycle before sending data.
#define SIE_SEND_SYNC_K_NOPS                    \
    SIE_SEND_SYNC_K                             \
    NOP8
#define SIE_SEND_SYNC_J_NOPS                    \
    SIE_SEND_SYNC_J                             \
    NOP8
#define SIE_SEND_SYNC_K                         \
    "ldi r20, 0x06\n\t"                         \
    "out %1, r20\n\t"
#define SIE_SEND_SYNC_J                         \
    "ldi r20, 0x05\n\t"                         \
    "out %1, r20\n\t"
#define SIE_SEND_SYNC                           \
    SIE_SEND_SYNC_K_NOPS                        \
    SIE_SEND_SYNC_J_NOPS                        \
    SIE_SEND_SYNC_K_NOPS                        \
    SIE_SEND_SYNC_J_NOPS                        \
    SIE_SEND_SYNC_K_NOPS                        \
    SIE_SEND_SYNC_J_NOPS                        \
    SIE_SEND_SYNC_K_NOPS                        \
    SIE_SEND_SYNC_K
/* }}} */
/* {{{ Receiving a sync */
#define SIE_RX_SYNC_K           \
    "sbis %2, 7\n\t"            \
    "rjmp .-6\n\t"
#define SIE_RX_SYNC_J           \
    "sbis %2, 6\n\t"            \
    "rjmp .-6\n\t"
#define SIE_RX_SYNC_FIRST           \
    "ldi r21, 20\n\t"               \
    "clz\n\t"                       \
    "ldi r20, 0xff\n\t"             \
    "out %5, r20\n\t"               \
    ".sie_begin_get_sync:\n\t"      \
    "dec r21\n\t"                   \
    "sbic %2, 7\n\t"                \
    "rjmp .sie_done_sync\n\t"       \
    "sbic %2, 7\n\t"                \
    "rjmp .sie_done_sync\n\t"       \
    "breq .sie_done_sync\n\t"       \
    "sbic %2, 7\n\t"                \
    "rjmp .sie_done_sync\n\t"       \
    "sbis %2, 7\n\t"                \
    "rjmp .sie_begin_get_sync\n\t"  \
    ".sie_done_sync:\n\t"           \
    "brne .+4\n\t"                  \
    "jmp .sie_sync_timeout\n\t"     \
    "ldi r20, 0x00\n\t"             \
    "out %5, r20\n\t"
#define SIE_RX_SYNC_FIRST_ACK           \
    "ldi r21, 20\n\t"                   \
    "clz\n\t"                           \
    "ldi r20, 0xff\n\t"                 \
    "out %5, r20\n\t"                   \
    ".sie_begin_get_sync_ack:\n\t"      \
    "dec r21\n\t"                       \
    "sbic %2, 7\n\t"                    \
    "rjmp .sie_done_sync_ack\n\t"       \
    "sbic %2, 7\n\t"                    \
    "rjmp .sie_done_sync_ack\n\t"       \
    "breq .sie_done_sync_ack\n\t"       \
    "sbic %2, 7\n\t"                    \
    "rjmp .sie_done_sync_ack\n\t"       \
    "sbis %2, 7\n\t"                    \
    "rjmp .sie_begin_get_sync_ack\n\t"  \
    ".sie_done_sync_ack:\n\t"           \
    "brne .+4\n\t"                      \
    "jmp .sie_sync_timeout\n\t"         \
    "ldi r20, 0x00\n\t"                 \
    "out %5, r20\n\t"
#define SIE_RX_SYNC             \
    "ldi r20, 0xff\n\t"         \
    "out %5, r20\n\t"           \
    "ldi r20, 0x00\n\t"         \
    "out %5, r20\n\t"           \
    SIE_RX_SYNC_J               \
    NOP2                        \
    "ldi r20, 0xff\n\t"         \
    "out %5, r20\n\t"           \
    "ldi r20, 0x00\n\t"         \
    "out %5, r20\n\t"           \
    SIE_RX_SYNC_K               \
    NOP8                        \
    SIE_RX_SYNC_J               \
    NOP2                        \
    "ldi r20, 0xff\n\t"         \
    "out %5, r20\n\t"           \
    "ldi r20, 0x00\n\t"         \
    "out %5, r20\n\t"           \
    SIE_RX_SYNC_K               \
    NOP8                        \
    SIE_RX_SYNC_J               \
    NOP2                        \
    "ldi r20, 0xff\n\t"         \
    "out %5, r20\n\t"           \
    "ldi r20, 0x00\n\t"         \
    "out %5, r20\n\t"           \
    SIE_RX_SYNC_K               \
    NOP8                        \
    NOP8                        \
    NOP4
/* }}} */
/* }}} */

/* {{{ End-of-packet macros */
// 1 cycle before it sends data.
// 8 free cycles when it ends.
#define SIE_SEND_EOP_PULSE_SE0_NOPS             \
    SIE_SEND_EOP_PULSE_SE0                      \
    NOP8
#define SIE_SEND_EOP_PULSE_J_NOPS               \
    SIE_SEND_EOP_PULSE_J                        \
    NOP8
#define SIE_SEND_EOP_PULSE_SE0                  \
    "ldi r20, 0x04\n\t"                         \
    "out %1, r20\n\t"
#define SIE_SEND_EOP_PULSE_J                    \
    "ldi r20, 0x05\n\t"                         \
    "out %1, r20\n\t"
#define SIE_SEND_EOP                            \
    SIE_SEND_EOP_PULSE_SE0_NOPS                 \
    SIE_SEND_EOP_PULSE_SE0_NOPS                 \
    SIE_SEND_EOP_PULSE_J
/* }}} */

/* {{{ Token macros */
/* Overview of sending each bit:
 * 1. Copy the buffer to the temp register.
 * 2. AND the temp register with 0x01.
 * 3. ADD 5 to the temp.
 * 4. Send this out on the port.
 * 5. Shift the buffer right.
 * 6. Subtract 1 from the count.
 * 7-8. Branch over the next instruction if not 0.
 * 8. Jump to the end.
 * For bit 7:
 * 9-10. Load the next byte into the buffer.
 * Note that there is no looping here, to save cycles. The maximum number of bytes after
 * bit stuffing that we will have to send is 6 bytes.
 */
/* {{{ Token: Send a whole byte */

#define SIE_TOKEN_BYTE                          \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_NOP                           \
    SIE_TOKEN_BIT_BUFFER
/* }}} */
/* {{{ Token: Send a single bit */
#define SIE_TOKEN_BIT                           \
    "mov r20, r3\n\t"                           \
    "andi r20,0x01\n\t"                         \
    "add r20, r10\n\t"                          \
    "out %1, r20\n\t"                           \
    "lsr r3\n\t"                                \
    "subi r16, 1\n\t"                           \
    "brne .+4\n\t"                              \
    "jmp .sie_send_token_eop\n\t"
/* }}} */
/* {{{ Token: Send bit, and nop until next one */
#define SIE_TOKEN_BIT_NOP                       \
    SIE_TOKEN_BIT                               \
    NOP2
/* }}} */
/* {{{ Token: Send bit, and buffer another byte */
#define SIE_TOKEN_BIT_BUFFER                    \
    SIE_TOKEN_BIT                               \
    "ld r3, X+\n\t"                            
/* }}} */
/* }}} */

/* {{{ Data packet macros */
/* {{{ RX macros */
/* Receiving data on the line:
 * 1. Read input on the port.
 * 2. AND with 0xC0 to mask the high bits.
 * 3. Branch over the next instruction if the Z flag is not set.
 * 4. Jump to the EOP handler
 * 5. AND with 0x80 to get the D+ line.
 * 6. OR the result with the buffer.
 * 7. Increment the data buffer length.
 * This is then different per bit:
 * Bits 0:6:
 * 8. Shift buffer right.
 * Bit 7:
 * 8-9. Store buffer to X+.
 * The theoretical max number of bytes is 10.
 */
/* {{{ Data: Receive a byte */
#define SIE_DATA_RX_BYTE                        \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_BUFFER

#define SIE_DATA_RX_SPECIAL_BYTE                \
    SIE_DATA_RX_SPECIAL_BIT                     \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_LOW                         \
    SIE_DATA_RX_BIT_BUFFER
/* }}} */
/* {{{ Data: Receive a single bit */
#define SIE_DATA_RX_BIT                         \
    "in r20, %2\n\t"                            \
    "andi r20, 0xC0\n\t"                        \
    "brne .+4\n\t"                              \
    "jmp .sie_rx_data_eop\n\t"                  \
    "andi r20, 0x80\n\t"                        \
    "or r3, r20\n\t"                            \
    "inc r17\n\t"
/* }}} */
/* {{{ Data: (special byte) Receive a single bit, and get ready to handle a handshake. */
#define SIE_DATA_RX_SPECIAL_BIT             \
    "in r20, %2\n\t"                        \
    "andi r20, 0xC0\n\t"                    \
    "brne .+4\n\t"                          \
    "jmp .sie_rx_data_first_eop\n\t"        \
    "andi r20, 0x80\n\t"                    \
    "or r3, r20\n\t"                        \
    "inc r17\n\t"                           \
    "lsr r3\n\t"                            \
    "out %5, r20\n\t"                       \
    NOP1
/* }}} */
/* {{{ Data: Receive a low bit (bits 6:0) */
#define SIE_DATA_RX_BIT_LOW                 \
    SIE_DATA_RX_BIT                         \
    "lsr r3\n\t"                            \
    "out %5, r20\n\t"                       \
    NOP1
/* }}} */
/* {{{ Data: Receive the high bit, and store the buffer back to memory */
#define SIE_DATA_RX_BIT_BUFFER              \
    SIE_DATA_RX_BIT                         \
    "st X+, r3\n\t"                         \
    "out %5, r20\n\t"
/* }}} */
/* }}} */
/* }}} */
/* {{{ TX macros */
/* {{{ Data: Send a whole byte */
#define SIE_DATA_BYTE                           \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_NOP                            \
    SIE_DATA_BIT_BUFFER
/* }}} */
/* {{{ Data: Send a single bit */
#define SIE_DATA_BIT                            \
    "mov r20, r3\n\t"                          \
    "andi r20,0x01\n\t"                         \
    "add r20, r10\n\t"                          \
    "out %1, r20\n\t"                           \
    "lsr r3\n\t"                               \
    "subi r17, 1\n\t"                            \
    "brne .+4\n\t"                             \
    "jmp .sie_send_data_eop\n\t"
/* }}} */
/* {{{ Data: Send bit, and nop until next one */
#define SIE_DATA_BIT_NOP                        \
    SIE_DATA_BIT                                \
    NOP2
/* }}} */
/* {{{ Data: Send bit, and buffer another byte */
#define SIE_DATA_BIT_BUFFER                     \
    SIE_DATA_BIT                                \
    "ld r3, X+\n\t"                            \
    NOP1
/* }}} */
/* }}} */
/* }}} */

/* {{{ Handshake macros */
/* {{{ Transmitting handshakes */
// 1 cycle taken before sending.
// 8 cycles free afterward.
#define SIE_HANDSHAKE_HIGHBIT_TX        \
    "ldi r20, 0x06\n\t"                 \
    "out %1, r20\n\t"                   \
    NOP8
#define SIE_HANDSHAKE_LOWBIT_TX         \
    "ldi r20, 0x05\n\t"                 \
    "out %1,r20\n\t"                    \
    NOP8
/* {{{ Send ACK pid */
#define SIE_HANDSHAKE_TX_ACK            \
    /* ACK is 0x4b */                   \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_HIGHBIT_TX 
/* }}} */
/* {{{ Send NACK pid */
#define SIE_HANDSHAKE_TX_NACK           \
    /* NACK is 0x5a */                  \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_HIGHBIT_TX            \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_LOWBIT_TX             \
    SIE_HANDSHAKE_HIGHBIT_TX
/* }}} */
/* }}} */
/* {{{ Receiving handshakes */
// Starts receiving right away.
// No cycles free afterward.
#define SIE_HANDSHAKE_RX_BIT    \
    "in r20, %2\n\t"            \
    "andi r20, 0x80\n\t"        \
    "lsr r3\n\t"                \
    "or r3, r20\n\t"            \
    NOP6
#define SIE_HANDSHAKE_RX_BYTE   \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT        \
    SIE_HANDSHAKE_RX_BIT
/* }}} */
/* }}} */

/* {{{ Macros to generate nop-like sequences */
#define NOP_INIT                                \
    "rjmp .+2\n\t"                              \
    ".nop_function:\n\t"                        \
    "ret\n\t"
#define NOP1                                    \
    "nop\n\t"
#define NOP2                                    \
    "ld __tmp_reg__, X /* 2 nops */\n\t"
#define NOP3                                    \
    NOP2                                        \
    NOP1
#define NOP4                                    \
    "push __tmp_reg__ /* 2 nops */\n\t"         \
    "pop __tmp_reg__ /* 2 more nops*/\n\t"
#define NOP5                                    \
    NOP2                                        \
    NOP3
#define NOP6                                    \
    NOP3                                        \
    NOP3
#define NOP7                                    \
    NOP3                                        \
    NOP4
#define NOP8                                    \
    "call .nop_function /* really 8 nops */\n\t"
/* }}} */ // macros for nops

/* {{{ Delay macro */
#define SIE_DELAY_4         \
    "clr r20\n\t"           \
    "inc r20\n\t"           \
    "cpi r20, 10\n\t"       \
    "brne .-10\n\t"         \
    NOP1
/* }}} */
