/*
*  FIFO circular buffers.
*  Write by Kurochkin Michail 05.08.2010
*  email: stelhs@ya.ru
*/


#ifndef _FIFO_H_
#define _FIFO_H_

struct fifo_buffer {
	u8 *buf;
	int count;
	int item_size;
	int read_pointer;
	int write_pointer;
};

int init_fifo(struct fifo_buffer *fifo, int item_width, int count);
void free_fifo(struct fifo_buffer *fifo);

int fifo_push(struct fifo_buffer *fifo, void *item);
int fifo_pop(struct fifo_buffer *fifo, void *item);
int fifo_is_full(struct fifo_buffer *fifo);
int fifo_is_empty(struct fifo_buffer *fifo);

#endif /* _FIFO_H_ */
