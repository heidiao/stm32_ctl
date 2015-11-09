#ifndef	__UTIL_H
#define	__UTIL_H

#include	<stdio.h>
#include	"def.h"

void dump( FILE *fp, const uint8_t *buf, int length );
uint8_t cal_checksum8( const void *buf, int length );

#endif
