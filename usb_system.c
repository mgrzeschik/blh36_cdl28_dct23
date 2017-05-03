// usb_system.c
// BCD -- Ben Hutton, Chris Leary, Devrin Talen
// 100-column width


#include "usb_system.h"

// ---------
// - PIPES -
// ---------

#if 0
PIPE_T *Pipe(uint8_t address, uint8_t endp, size_t bandwidth, uint8_t maximum_packet_size, 
             uint8_t maximum_buffer_size, TRANSFER_TYPE_T transfer_type, PIPE_TYPE_T pipe_type)
/*
 * Constructor. Returns a pointer to a newly created pipe with the specified parameters.
 */
{
    PIPE_T *new_pipe;

    new_pipe = malloc(sizeof(PIPE_T));

    // initialize the pipe with the passed arguments
    new_pipe->head = NULL;
    new_pipe->tail = NULL;
    new_pipe->address = address;
    new_pipe->endp = endp;
    new_pipe->bandwidth = bandwidth;
    new_pipe->maximum_packet_size = maximum_packet_size;
    new_pipe->maximum_buffer_size = maximum_buffer_size;
    new_pipe->transfer_type = transfer_type;
    new_pipe->pipe_type = pipe_type;

    return new_pipe;
}


void DestroyPipe(PIPE_T *pipe)
/*
 * Destructor. Frees the pipe.
 */
{
    free(pipe);
}
#endif
