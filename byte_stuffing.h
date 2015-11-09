#ifndef __BYTE_STUFFING_H
#define	__BYTE_STUFFING_H

#include	"def.h"

#define	TMSG_EOF							0xf0
#define	TMSG_BS								0xf1	// byte stuffing 
#define	TMSG_BS_EOF							0xf2

extern uint8_t	tmsg_bs_bs[2];
extern uint8_t	tmsg_bs_eof[2];
extern uint8_t	tmsg_eof[1];

int	write_byte_stuffing( int fd, uint8_t data );
int byte_stuffing( uint8_t *dest, uint8_t *src, int length );
int reverse_byte_stuffing( uint8_t *src, int length );

#endif