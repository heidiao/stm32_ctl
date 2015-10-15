#ifndef	__UTIL_H
#define	__UTIL_H

#include	<stdio.h>
#include	"def.h"

void dump( FILE *fp, uint8_t *buf, int length );
uint8_t cal_checksum8( uint8_t *ptr, int length );

#endif
