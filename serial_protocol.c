#include	<unistd.h>
#include	<string.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<assert.h>
#include	"serial_protocol.h"
#include	"byte_stuffing.h"
#include	"util.h"

uint32_t deserialize( const uint8_t *buf, int length ) {
	uint32_t	temp = 0;
	while( length ) {
		temp <<= 8;
		temp |= buf[--length];
	}
	return temp;
}

void serialize( uint8_t *buf, int length, uint32_t data ) {
	while( length-- ) {
		*buf++ = data;
		data >>= 8;
	}
}

// no checksum in the buf
int serial_send_command( int fd, const void *buf, int length ) {
	const uint8_t	*ptr = (const uint8_t*) buf;
	uint8_t	checksum = -cal_checksum8( buf, length );
	int	total = 0;
	int	nwrite;

	// write data
	while( length-- ) {
		nwrite = write_byte_stuffing( fd, *ptr++ );
		if( nwrite<0 ) {
failed:
			perror( __func__ );
			return nwrite;
		}
		total += nwrite;
	}

	// write checksum
	nwrite = write_byte_stuffing( fd, checksum );
	if( nwrite < 0 )
		goto failed;
	total += nwrite;

	// write EOF
	nwrite = write( fd, tmsg_eof, sizeof(tmsg_eof) );
	if( nwrite < 0 )
		goto failed;
	total += nwrite;

	return total;
}

int serial_read_message( int fd, uint8_t *buf, int size ) {
	uint8_t *ptr = buf;
	int	total = 0;

	while( total<size ) {
		int	nread = read( fd, ptr, 1 );
		if( nread<0 ) {
			perror( "serial_read" );
			return 0;
		} else if( nread==0 ) {
			fprintf( stderr, "%s: no data\n", __func__ );
			return -2;
		}
		total += nread;
		if( *ptr==TMSG_EOF ) {
			total -= reverse_byte_stuffing( buf, total );
			if( total>1 ) {
				if( cal_checksum8( buf, total-1 )==0 ) {
					return total;
				} else {
					fprintf( stderr, "%s: checksum error\n", __func__ );
					dump( stderr, buf, total );
				}
			}
			fprintf( stderr, "%s: resync\n", __func__ );
			return -3;
		}
		ptr += nread;
	}
	return total;
}

/*
	queue one message and process it
	example:

	nread = serial_read_buffer( uart_fd, buf, BUF_SIZE );
	if( nread>0 ) {
		uint8_t	*ptr = buf;

//		dump( stdout, buf, nread );
		while( nread ) {
			int nparse = serial_parse_message( q, ptr, nread );
			if( nparse ) {
				int		msg_length = queue_length(q);
				uint8_t	msg[msg_length];

				msg_length = serial_extract_message( msg, msg_length, q );
				if( msg_length ) {
//					dump( stdout, msg, msg_length );
					// handle message
					// ...
				}
				ptr += nparse;
				nread -= nparse;
			} else
				break;
		}
	} else if( nread==-1 ) {
		// read() error
	} else {
		// usb disconnect
		exit(-2);
	}
*/
int serial_read_buffer( int fd, uint8_t *buf, int size ) {
	int	nread = read( fd, buf, size );
	if( nread<0 ) {
		perror( "serial_read" );
		return -1;
	} else if( nread==0 ) {
		fprintf( stderr, "%s: no data\n", __func__ );
		return -2;
	}
	return nread;
}

// parse one message into queue
// return none zero when found message
int serial_parse_message( queue *q, uint8_t *buf, int size ) {
	uint8_t	*ptr = buf;

	while( size-- ) {
		queue_enqueue( q, *ptr );
		if( *ptr++==TMSG_EOF )
			return ptr-buf;
	}
	return 0;
}

// allocate buf with queue length
int serial_extract_message( uint8_t *buf, int size, queue *q ) {
	int		length = queue_length(q);
	uint8_t	*ptr = buf;

	assert( queue_length(q)==size );
	assert( queue_tail(q)==TMSG_EOF );

	while( size-- )
		*ptr++ = queue_dequeue(q);
	length = length - reverse_byte_stuffing( buf, length );
	if( cal_checksum8( buf, length-1)==0 ) {
		return length;
	} else {
		fprintf( stderr, "%s: !!!!!!! checksum error !!!!!!!\n", __func__ );
		abort();
	}
	return 0;
}
