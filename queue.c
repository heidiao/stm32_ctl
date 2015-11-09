#include	<stdlib.h>
#include	<string.h>
#include	"queue.h"

queue* queue_init( queue *q ) {
	q->size = DEFAULT_QUEUE_SIZE;
	q->buf = (uint8_t*) malloc( q->size );
	q->head = q->tail = 0;
	q->length = 0;
	return q;
}

void queue_enqueue( queue *q, uint8_t data ) {
	if( q->length == q->size ) {
		q->buf = (uint8_t*) realloc( q->buf, q->size*2 );
		memcpy( q->buf + q->size, q->buf, q->head );
		q->tail = q->head + q->length;
		q->size *= 2;
	}
	q->buf[q->tail++] = data;
	if( q->tail == q->size )
		q->tail = 0;
	q->length++;
}

uint8_t queue_dequeue( queue *q ) {
	uint8_t	temp;
	
	if( q->length )
		temp = q->buf[q->head++];
	else
		abort();
	if( q->head == q->size )
		q->head = 0;
	q->length--;
	return temp;
}

int queue_length( queue *q ) {
	return q->length;
}

uint8_t	queue_tail( queue *q ) {
	return q->buf[q->tail];
}
