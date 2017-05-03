// usb_interface.h
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


/*
 * The USB Interface is the lowest layer of the three-tier USB protocol stack. It contains both
 * the Host Controller and Serial Interface Engine.
 *
 * The Host Controller (HC) is the interface to the upper (USB System) layer, and it instructs 
 * the Serial Interface Engine (SIE) to perform transactions from its transaction queue. It
 * ensures that all the SIE's data fields (token length, packet length, etc.) are set properly
 * before sending it the 'execute transfer' request. This includes placing the transaction to
 * be executed into the SIE's buffer -- a process which includes packing the transaction fields
 * into the buffer, bit stuffing the buffer, NRZI encoding the buffer, and then performing any
 * prerequisite, SIE-required actions to the in-buffer data.
 *
 * NOTE: Several #idef DEBUG_XXX statements are scattered throughout the file which are used for
 * debugging over USART. Compilation with definition flags (-D THING_TO_DEFINE) will enable 
 * these USART debugging capabilities.
 *
 * NOTE: Sever #ifndef NO_AVR_GCC statements are also scattered throughout the file, using a 
 * precarious double negative. These statements intend to section off those portions of the USB 
 * Interface that cannot be compiled in a uniprocessor fashion. Most of these sections are in some 
 * way tied to Atmel 8-bit assembly code, which is used a great deal (and with great skill, thanks 
 * to Devrin "The Assembler" Talen) in the SIE.
 */


#ifndef _USB_INTERFACE_H
#define _USB_INTERFACE_H


/************/
/* INCLUDES */
/************/


#include <stdio.h>
#include <stdlib.h>
#include "usb_primitives.h"
#ifndef NO_AVR_GCC
# include <avr/io.h>
#endif


/***********/
/* DEFINES */
/***********/


#define USBPORT PORTA
#define USBPIN PINA
#define SIE_BUFFER_SIZE 16 // size of the SIE buffer length in bytes


typedef enum {PENDING, COMPLETED, CANCELED, NOTFILLED} TD_STATUS_T;


/***********/
/* STRUCTS */
/***********/


typedef struct SerialInterfaceEngine {
    uint8_t *buffer; // of length SIE_BUFFER_SIZE
    ENCODING_T buffer_encoding;
    uint8_t token_packet_length; // in bits
    uint8_t data_packet_length; // in bits
    TRANSMISSION_DIRECTION_T data_direction;
    PID_T *handshake_result;
} SIE_T;


typedef struct HostController {
    TRANSACTION_NODE_T *transaction_queue_head;
    TRANSACTION_NODE_T *transaction_queue_tail;
    SIE_T *sie; // bound to a serial interface engine
    uint8_t data_sequence_bit;
} HC_T;


/*********/
/* ENUMS */
/*********/


/***********/
/* GLOBALS */
/***********/

// None


// NOTE: this is a special include. Its goal is to reduce the length of this file by placing the
//  long assembly code macros in a separate file, thus making this file more readable. The include
//  is placed here so that the macros can use the includes, defines, and structs if need be.
#include "usb_sie_macros.c"


/**************/
/* PROTOTYPES */
/**************/


void transaction_pull_data_from_sie(TRANSACTION_T *transaction, SIE_T *sie);


// -------------------
// - HOST CONTROLLER -
// -------------------


HC_T *HostController();
void DestroyHostController(HC_T *hc);

void hc_bind_sie(HC_T *hc, SIE_T *sie);
SIE_T *hc_unbind_sie(HC_T *hc);
void hc_push_transaction(HC_T *hc, TRANSACTION_T *new_transaction);
TRANSACTION_T *hc_pop_transaction(HC_T *hc);
TRANSACTION_T *hc_do_transaction(HC_T *hc);


// ---------------------------
// - SERIAL INTERFACE ENGINE -
// ---------------------------


SIE_T *SerialInterfaceEngine();
void DestroySerialInterfaceEngine(SIE_T *sie);

// Setup functions: Operate on the buffers and do pre-processing.
void sie_bitstuff_buffer(SIE_T *sie);
void sie_bitunstuff_buffer(SIE_T *sie);
void sie_bitstuff_buffer_helper(SIE_T *sie, STUFF_METHOD_T stuff_method);
void sie_nrzi_encode_buffer(SIE_T *sie, uint8_t transient_first_state);
void sie_nrzi_decode_buffer(SIE_T *sie, uint8_t transient_first_state);

// Transfer functions: Affect the state of the output pins and work with the buffers.
void sie_idle(SIE_T *sie);
void sie_transfer(SIE_T *sie, TRANSFER_TYPE_T type);
void sie_control_transfer(SIE_T *sie);
void sie_interrupt_transfer(SIE_T *sie);
void sie_keepalive(SIE_T *sie);
void sie_reset(SIE_T *sie);
void sie_resume(SIE_T *sie);
uint8_t sie_detect_device(SIE_T *sie);

#endif
