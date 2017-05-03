// usb_client.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width

#include "usb_client.h"

#define debug(string) sprintf(print_buffer, string); send_debug()

// ---------------------
// - STANDARD REQUESTS -
// ---------------------


STANDARD_REQUEST_T *StandardRequest(DESCRIPTOR_TYPE_T request_target, REQUEST_TYPE_T request,
                                    uint8_t valueHi, uint8_t valueLo, uint16_t index,
                                    uint16_t length)
/* 
 * Constructor.  Returns a pointer to a newly created standard request with the specified 
 * parameters.
 * 
 * request_target   DEVICE, INTERFACE, or ENDP -- used to determine bmRequestType
 * request          function to perform -- GET_STATUS, GET_DESCRIPTOR, etc.
 * valueHi          for GET_DESCRIPTOR and SET_DESCRIPTOR, contains descriptor type, otherwise 0
 * valueLo          contains various variables
 * index           contains endp or interface number, if applicable
 * length          if data is to be returned, contains the maximum allowable bytes of data
 */
{
    STANDARD_REQUEST_T *new_standard_request;
    uint8_t requestType_temp;
    
    new_standard_request = malloc(sizeof(STANDARD_REQUEST_T));
    
    if (request_target == HID) {
	if (request == GET_REPORT || request == GET_IDLE || request == GET_PROTOCOL)
	    requestType_temp = 0xa1;
	else
	    requestType_temp = 0x21;
    } else {
	
	// Set up the bmRequestType field to be a bitmap as follows:
	// 7: Data transfer direction (0 = Host-to-Device; 1 = Device-to-Host)
	// 6..5: Type (0 = Standard; 1 = Class; 2 = Vendor; 3 = Reserved)
	// 4..0: Recipient (0 = Device; 1 = Interface; 2 = Endpoint; 3 = Other; 4..31 Reserved)
	if (request == GET_STATUS || request == GET_DESCRIPTOR || request == GET_CONFIGURATION ||
	    request == GET_INTERFACE || request == SYNCH_FRAME) 
	    requestType_temp = 0x80; //Set data transfer direction to Device-to-Host
	else
	    requestType_temp = 0x00; //Set data transfer direction to Host-to-Device

	if (request_target == INTERFACE) requestType_temp |= 0x01;
	else if (request_target == ENDP) requestType_temp |= 0x02;
    }
    
    new_standard_request->bmRequestType = requestType_temp;
    new_standard_request->bRequest = request;
    new_standard_request->wValueHi = valueHi;
    new_standard_request->wValueLo = valueLo;
    new_standard_request->wIndex = index;
    new_standard_request->wLength = length;
    
    return new_standard_request;
}


void DestroyStandardRequest(STANDARD_REQUEST_T *standard_request)
/* 
 * Destructor. Simply frees the memory allocated for the standard request.
 */
{
    free(standard_request);
}

STANDARD_REQUEST_T *standard_request_from_template(REQUEST_TYPE_T request_type, 
						  DESCRIPTOR_TYPE_T request_target) 
{
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = 0x0;
    uint16_t length = 0x0;

    switch (request_type) {
    case GET_STATUS :
    case SYNCH_FRAME :
	length = 0x2;
	break;
    case GET_CONFIGURATION :
    case GET_INTERFACE :
    case SET_INTERFACE :
	length = 0x1;
	break;
    }

    return StandardRequest(request_target, request_type, valueHi, valueLo, index, length);
}

STANDARD_REQUEST_T *hid_request_from_template(HID_REQUEST_TYPE_T request_type) 
{
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = 0x0;
    uint16_t length = 0x0;
    DESCRIPTOR_TYPE_T request_target = HID;

    switch (request_type) {
    case GET_IDLE :
    case GET_PROTOCOL :
	length = 0x1;
	break;
    }

    return StandardRequest(request_target, request_type, valueHi, valueLo, index, length);
}

DATA_PACKET_T *make_standard_request_data_packet(STANDARD_REQUEST_T *request) 
{
    DATA_PACKET_T *dp;
    dp = DataPacket(DATA0, 8);
    dp->data[7] = request->bmRequestType;
    dp->data[6] = request->bRequest;
    dp->data[5] = request->wValueLo;
    dp->data[4] = request->wValueHi;
    dp->data[3] = request->wIndex & 0xff;
    dp->data[2] = request->wIndex >> 8;
    dp->data[1] = request->wLength & 0xff;
    dp->data[0] = request->wLength >> 8;

    
    sprintf(print_buffer, "datapacket: 0x%x %x %x %x %x %x %x %x \n",
	    dp->data[7], dp->data[6], dp->data[5], dp->data[4], dp->data[3], dp->data[2], dp->data[1], dp->data[0]);

    send_debug();
    return dp;
}

DATA_PACKET_T *make_standard_request(DESCRIPTOR_TYPE_T request_target, REQUEST_TYPE_T request_type, 
                                    uint8_t valueHi, uint8_t valueLo,  uint16_t index, 
                                    uint16_t length)
/*
 * Makes a data packet containing a standard request as a payload.  Used in control transfers.
 *   request_target:      either DEVICE, INTERFACE, or ENDP
 *   length:              the number of bytes of data you expect to receive.
 *   valueLo and valueHi: should be set to request-specific values
 *   index:               if an INTERFACE or ENDP request, put the interface or endpoint number here
 */
{
    STANDARD_REQUEST_T *new_request;
    
    new_request = StandardRequest(request_target, request_type, valueHi, valueLo, index, length);
    return make_standard_request_data_packet(new_request);
}



void execute_standard_request(STANDARD_REQUEST_T *request, SIE_T *sie, HC_T *hc, uint8_t addr, uint8_t endp, 
			      DATA_PACKET_T *payload, uint8_t reset_after)
//Takes in a request and a properly bound host controller
//Determines if it needs any, and how many, IN packets
//Puts together and executes the appropriate transaction
{
    uint8_t iter;
    uint8_t iter2;
    uint8_t packet_length;
    PID_T datatype = DATA0;
    uint8_t transaction_count = 0;

    // transaction-specific variables
    TRANSACTION_T *setup;
    TRANSACTION_T *ins;
    TRANSACTION_T *status;
    TOKEN_PACKET_T *tp;
    DATA_PACKET_T *dp;

 // create a setup transaction
    tp = TokenPacket(SETUP, addr, endp);
    dp = make_standard_request_data_packet(request);

    // create the control transaction with the new token packet
    setup = Transaction(CONTROL, tp, dp);
    debug("pushing a SETUP\n");
    hc_push_transaction(hc, setup);
    transaction_count++;
    
    // if we're GETting data, we need to issue some INs...
    if (request->bRequest == GET_STATUS || request->bRequest == GET_DESCRIPTOR
	|| request->bRequest == GET_CONFIGURATION || request->bRequest == GET_INTERFACE
	|| request->bRequest == SYNCH_FRAME || request->bRequest == GET_REPORT)
    {
	//wLength holds the max # of bytes that the function can return
	//IN packets can contain 8 bytes in low-speed devices
	//So send 1 IN packet for every 8 bytes that you want to receive
	for (iter = 0; iter < request->wLength; iter+=8) {
	    tp = TokenPacket(IN, addr, endp);
	    // this is a data in, so PID and size don't matter
	    dp = DataPacket(DATA0, 0); 
	    ins = Transaction(CONTROL, tp, dp);
	    // push the transaction on the HC queue
	    debug("pushing an IN\n");
	    hc_push_transaction(hc, ins);
	    transaction_count++;

	    //handle the reset...
	    if(reset_after != NULL) {
	        debug("doing transactions...\n");
		for (iter = 0; iter < transaction_count; iter++) hc_do_transaction(hc);
		debug("resetting...\n");
		sie_reset(sie);
		for (iter = 0; iter < 100; iter++) _delay_ms(10);
		debug("idling...\n");
		sie_idle(sie);
		return;
	    }
	}
	debug("sending status. \n");

	// and then put together the STATUS packet (acknowledge by sending a zero-length data packet)
	tp = TokenPacket(OUT, addr, endp);
	dp = DataPacket(DATA1, 0);
	status = Transaction(CONTROL, tp, dp);
	hc_push_transaction(hc, status);
	transaction_count++;
    }
    
    // if we're doing a SET_DESCRIPTOR request, we have to send the new data...
    else if (request->bRequest == SET_DESCRIPTOR)
    {
	for (iter = 0; iter < request->wLength; iter+=8) {
	    tp = TokenPacket(OUT, addr, endp);

	    packet_length = request->wLength - iter; // how many bytes should be sent this time...
	    if(packet_length > 8) packet_length = 8;

	    dp = DataPacket(datatype, packet_length); 

	    // if wLength is greater than payload->payload_size, then just put in empty bytes at the end
	    for(iter2 = iter; iter2 < iter + packet_length; iter++) {
		if(iter2 < payload->payload_size) dp->data[iter2] = payload->data[iter2];
	    }

	    //tick-tock between the two DATA packet types
	    datatype = (datatype == DATA0) ? DATA1 : DATA0;

	    ins = Transaction(CONTROL, tp, dp);
	    // push the transaction on the HC queue
	    hc_push_transaction(hc, ins);
	    transaction_count++;
	
	}
	debug("sending status. \n");
	// and then put together the STATUS packet (expect to see a zero-length data packet)
	tp = TokenPacket(IN, addr, endp);
	dp = DataPacket(DATA0, 0);
	status = Transaction(CONTROL, tp, dp);
	hc_push_transaction(hc, status);
	transaction_count++;
    }
    else //no data packets, but we still need to do the status packet
    {

	debug("sending status. \n");
	// and then put together the STATUS packet (expect to see a zero-length data packet)
	tp = TokenPacket(IN, addr, endp);
	dp = DataPacket(DATA0, 0);
	status = Transaction(CONTROL, tp, dp);
	hc_push_transaction(hc, status);
	transaction_count++;

	debug("sending status. \n");
	// and then put together the STATUS packet (expect to see a zero-length data packet)
	tp = TokenPacket(IN, addr, endp);
	dp = DataPacket(DATA0, 0);
	status = Transaction(CONTROL, tp, dp);
	hc_push_transaction(hc, status);
	transaction_count++;

	debug("sending status. \n");
	// and then put together the STATUS packet (expect to see a zero-length data packet)
	tp = TokenPacket(IN, addr, endp);
	dp = DataPacket(DATA0, 0);
	status = Transaction(CONTROL, tp, dp);
	hc_push_transaction(hc, status);
	transaction_count++;
    }
    debug("doing transactions...\n");
    // do the transactions
    for (iter = 0; iter < transaction_count; iter++) hc_do_transaction(hc);
    DestroyStandardRequest(request);
}



/* probably not needed anymore
DATA_PACKET_T make_GET_DEVICE_STATUS() 
{

    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = GET_STATUS;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = 0x0;
    uint16_t length = 0x2;

    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_INTERFACE_STATUS(uint8_t interface) 
{
    DESCRIPTOR_TYPE_T request_target = INTERFACE;
    REQUEST_TYPE_T request_type = GET_STATUS;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = interface;
    uint16_t length = 0x2;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_ENDPOINT_STATUS(uint8_t endpoint) 
{
    DESCRIPTOR_TYPE_T request_target = ENDP;
    REQUEST_TYPE_T request_type = GET_STATUS;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = endpoint;
    uint16_t length = 0x2;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_CLEAR_DEVICE_FEATURE(FEATURE_SELECTOR_T feature)
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = CLEAR_FEATURE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = feature;
    uint16_t index = 0x0;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_CLEAR_ENDPOINT_FEATURE(FEATURE_SELECTOR_T feature, uint8_t endpoint)
{
    DESCRIPTOR_TYPE_T request_target = ENDP;
    REQUEST_TYPE_T request_type = CLEAR_FEATURE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = feature;
    uint16_t index = endpoint;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SET_DEVICE_FEATURE(FEATURE_SELECTOR_T feature)
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = SET_FEATURE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = feature;
    uint16_t index = 0x0;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SET_ENDPOINT_FEATURE(FEATURE_SELECTOR_T feature, uint8_t endpoint)
{
    DESCRIPTOR_TYPE_T request_target = ENDP;
    REQUEST_TYPE_T request_type = SET_FEATURE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = feature;
    uint16_t index = endpoint;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SET_ADDRESS(uint8_t address) 
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = SET_ADDRESS;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = address;
    uint16_t index = 0x0;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_DESCRIPTOR(uint8_t desc_index, REQUEST_TYPE_T type, uint16_t length) 
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = GET_DESCRIPTOR;
    uint8_t valueHi = type;
    uint8_t valueLo = desc_index;
    uint16_t index = 0x0;
    uint16_t length = length;
    
    if(valueLo == STRING) index = 0x0409; // Language code for English (United States)
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_DEVICE_DESCRIPTOR(uint16_t length)
{
    return make_GET_DESCRIPTOR(0, DEVICE, length);
}


DATA_PACKET_T make_SET_DESCRIPTOR(uint8_t index, REQUEST_TYPE_T type, uint16_t length) 
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = SET_DESCRIPTOR;
    uint8_t valueHi = type;
    uint8_t valueLo = desc_index;
    uint16_t index = 0x0;
    uint16_t length = length;
    
    if(valueLo == STRING) index = 0x0409; // Language code for English (United States)
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_CONFIGURATION() 
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = GET_CONFIGURATION;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = 0x0;
    uint16_t length = 0x1;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SET_CONFIGURATION(uint8_t configuration) 
{
    DESCRIPTOR_TYPE_T request_target = DEVICE;
    REQUEST_TYPE_T request_type = SET_CONFIGURATION;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = configuration;
    uint16_t index = 0x0;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_GET_INTERFACE(uint8_t interface) 
{
    DESCRIPTOR_TYPE_T request_target = INTERFACE;
    REQUEST_TYPE_T request_type = GET_INTERFACE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = interface;
    uint16_t length = 0x1;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SET_INTERFACE(uint8_t interface, uint8_t new_interface) 
{
    DESCRIPTOR_TYPE_T request_target = INTERFACE;
    REQUEST_TYPE_T request_type = SET_INTERFACE;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = new_interface;
    uint16_t index = interface;
    uint16_t length = 0x0;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}


DATA_PACKET_T make_SYNCH_FRAME(uint8_t endpoint) 
{
    DESCRIPTOR_TYPE_T request_target = ENDP;
    REQUEST_TYPE_T request_type = SYNCH_FRAME;
    uint8_t valueHi = 0x0;
    uint8_t valueLo = 0x0;
    uint16_t index = endpoint;
    uint16_t length = 0x2;
    
    return make_standard_request(request_target, request_type, valueHi, valueLo, index, length);
}
*/
