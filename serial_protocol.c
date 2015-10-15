#include	<unistd.h>
#include	<string.h>
#include	<stdio.h>
#include	"serial_protocol.h"
#include	"byte_stuffing.h"
#include	"util.h"

inline uint8_t* update_checksum( uint8_t *buf, int length ) {
	buf[length-1] = -cal_checksum8( buf, length-1 );
	return buf;
}

// last byte in buf must be checksum
int serial_send_command( int fd, void *buf, int length ) {
	int	bs_length = length + byte_stuffing( NULL, update_checksum(buf,length), length );
	uint8_t bs_buf[bs_length+1];
	int	nwrite;

	byte_stuffing( bs_buf, buf, bs_length );
	bs_buf[bs_length] = TMSG_EOF;
	bs_length++;

	nwrite = write( fd, bs_buf, bs_length );
	if( nwrite<0 ) {
		perror( __func__ );
		return nwrite;
	}
	return nwrite;
}

int serial_read( int fd, uint8_t *buf, int size ) {
	uint8_t *ptr = buf;
	int	total = 0;

	while( total<size ) {
		int	nread = read( fd, ptr, 1 );
		if( nread<0 ) {
			perror( "serial_read" );
			return -1;
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

