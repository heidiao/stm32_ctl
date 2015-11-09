#include	<stdio.h>
#include	"util.h"

void dump( FILE *fp, const uint8_t *buf, int length ) {
	fprintf( fp, "[%02d] ", length );
	while( length-- > 0 )
		fprintf( fp, "%02x ", *buf++ );
	putchar( '\n' );
}

uint8_t cal_checksum8( const void *buf, int length ) {
	uint8_t checksum = 0;
	const uint8_t	*ptr = (const uint8_t*) buf;

	while( length-- )
		checksum += *ptr++;
	return checksum;
}
