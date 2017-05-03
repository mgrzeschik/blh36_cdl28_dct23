// usb_client.h
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width

#ifndef _USB_CLIENT_H
#define _USB_CLIENT_H


/************/
/* INCLUDES */
/************/


#include "usb_system.h"


/***********/
/* DEFINES */
/***********/


/*********/
/* ENUMS */
/*********/


typedef enum 
{ 
    GET_STATUS=0, CLEAR_FEATURE=1, SET_FEATURE=3, SET_ADDRESS=5, GET_DESCRIPTOR=6, SET_DESCRIPTOR=7,
    GET_CONFIGURATION=8, SET_CONFIGURATION=9, GET_INTERFACE=10, SET_INTERFACE=11, SYNCH_FRAME=12
} REQUEST_TYPE_T;

typedef enum
{
    GET_REPORT=1, GET_IDLE=2, GET_PROTOCOL=3, SET_REPORT=9, SET_IDLE=10, SET_PROTOCOL=11
} HID_REQUEST_TYPE_T;

typedef enum 
{ 
	DEVICE_REMOTE_WAKEUP = 0x1, ENDPOINT_HALT = 0x0, TEST_MODE = 0x2
} FEATURE_SELECTOR_T;


/***********/
/* STRUCTS */
/***********/


// ---------------------
// - STANDARD REQUESTS -
// ---------------------


typedef struct StandardRequest
/* 
 * Send as the data payload of the data packet in the setup transaction of a control transfer
 */
{
	uint8_t bmRequestType;
	REQUEST_TYPE_T bRequest;
	uint8_t wValueHi; // high byte of wValue
	uint8_t wValueLo; // low byte of wValue
	uint16_t wIndex;  // 2 bytes...
	uint16_t wLength;  // 2 bytes...
} STANDARD_REQUEST_T;

/*************/
/* FUNCTIONS */
/*************/

STANDARD_REQUEST_T *StandardRequest(DESCRIPTOR_TYPE_T request_target, REQUEST_TYPE_T request,
                                    uint8_t valueHi, uint8_t valueLo, uint16_t index,
                                    uint16_t length);

void DestroyStandardRequest(STANDARD_REQUEST_T *standard_request);
STANDARD_REQUEST_T *standard_request_from_template(REQUEST_TYPE_T request_type, 
						   DESCRIPTOR_TYPE_T request_target);
STANDARD_REQUEST_T *hid_request_from_template(HID_REQUEST_TYPE_T request_type);
DATA_PACKET_T *make_standard_request_data_packet(STANDARD_REQUEST_T *request);
DATA_PACKET_T *make_standard_request(DESCRIPTOR_TYPE_T request_target, REQUEST_TYPE_T request_type, 
                                    uint8_t valueHi, uint8_t valueLo,  uint16_t index, 
				     uint16_t length);
void execute_standard_request(STANDARD_REQUEST_T *request, SIE_T *sie, HC_T *hc, uint8_t addr, uint8_t endp, 
			      DATA_PACKET_T *payload, uint8_t reset_after);

#endif
