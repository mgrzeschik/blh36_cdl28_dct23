// usb_system.h
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


#ifndef _USB_SYSTEM_H
#define _USB_SYSTEM_H


/************/
/* INCLUDES */
/************/


#include "usb_interface.h"


/***********/
/* DEFINES */
/***********/


/*********/
/* ENUMS */
/*********/


/***********/
/* STRUCTS */
/***********/

// ---------
// - PIPES -
// ---------

#if 0

typedef struct Pipe
// A logical connection between the USB system and an endpoint on a device.
{
    TRANSACTION_NODE_T *head;
    TRANSACTION_NODE_T *tail;
    uint8_t address; // a pipe is associated with an endpoint within a function
    uint8_t endp; // a pipe is associated with an endpoint
    uint8_t bandwidth;
    uint8_t maximum_packet_size;
    uint8_t maximum_buffer_size;
    TRANSFER_TYPE_T transfer_type;
    TRANSMISSION_DIRECTION_T data_direction;
    PIPE_TYPE_T pipe_type;
} PIPE_T;


typedef PtrNode PIPE_NODE_T;


/**************/
/* PROTOTYPES */
/**************/


// ---------
// - PIPES -
// ---------


PIPE_T *Pipe(uint8_t address, uint8_t endp, uint8_t bandwidth,
             uint8_t maximum_packet_size, uint8_t maximum_buffer_size, 
             TRANSFER_TYPE_T transfer_type, PIPE_TYPE_T pipe_type);
void DestroyPipe(PIPE_T *pipe); 

#endif

#endif
