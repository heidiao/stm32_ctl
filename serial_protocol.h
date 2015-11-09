#ifndef __SERIAL_PROTOCOL_H
#define	__SERIAL_PROTOCOL_H

#include	"def.h"
#include	"queue.h"

uint32_t deserialize( const uint8_t *buf, int length );
void serialize( uint8_t *buf, int length, uint32_t data );
int serial_send_command( int fd, const void *buf, int length );
int serial_read_message( int fd, uint8_t *buf, int size );
int serial_read_buffer( int fd, uint8_t *buf, int size );
int serial_parse_buffer( queue *q, uint8_t *buf, int size );
int serial_extract_message( uint8_t *buf, int size, queue *q );

#endif
