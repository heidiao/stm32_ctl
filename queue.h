#ifndef __QUEUE_H
#define	__QUEUE_H

#include	"def.h"

#define		DEFAULT_QUEUE_SIZE	1

typedef struct {
	uint8_t	*buf;
	int		size;
	int		head, tail;
	int		length;
} queue;

queue* queue_init( queue *q );
void queue_enqueue( queue *q, uint8_t data );
uint8_t queue_dequeue( queue *q );
int queue_length( queue *q );
uint8_t	queue_tail( queue *q );

#endif
