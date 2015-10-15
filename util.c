#include	<stdio.h>
#include	"util.h"

void dump( FILE *fp, uint8_t *buf, int length ) {
	fprintf( fp, "%02d: ", length );
	while( length-- > 0 )
		fprintf( fp, "%02x ", *buf++ );
	putchar( '\n' );
}

uint8_t cal_checksum8( uint8_t *ptr, int length ) {
	uint8_t checksum8 = 0;
	while( length-- )
		checksum8 += *ptr++;
	return checksum8;
}
