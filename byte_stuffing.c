#include	<stdio.h>
#include	<assert.h>
#include	<stdlib.h>
#include	"byte_stuffing.h"

int byte_stuffing( uint8_t *dest, uint8_t *src, int length ) {
	int	count = 0;

	assert( src!=dest );
	
	while( length-- ) {
		if( *src==TMSG_BS || *src==TMSG_EOF ) {
			count++;
			if( dest ) {
				*dest++ = TMSG_BS;
				*dest++ = *src==TMSG_BS ? TMSG_BS : TMSG_BS_EOF;
			} 
		} else if( dest )
			*dest++ = *src;
		src++;
	}
	return count;
}
							
int reverse_byte_stuffing( uint8_t *src, int length ) {
	uint8_t	*dest = src;
	int	count = 0;

	while( length-- ) {
		if( *src==TMSG_BS ) {
			count++;
			src++;
#if 0
			*dest++ = *src==TMSG_BS ? TMSG_BS : TMSG_EOF;
#else
			if( *src==TMSG_BS )
				*dest++ = TMSG_BS;
			else if( *src==TMSG_BS_EOF )
				*dest++ = TMSG_EOF;
			else
				fprintf( stderr, "%s: byte stuffing failed %02x\n", __func__, *src );
#endif
		} else
			*dest++ = *src;
		src++;
	}
	return count;
}
