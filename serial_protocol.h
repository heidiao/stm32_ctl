#ifndef __SERIAL_PROTOCOL_H
#define	__SERIAL_PROTOCOL_H

#include	"def.h"

int serial_send_command( int fd, void *buf, int length );
int serial_read( int fd, uint8_t *buf, int size );

#endif
