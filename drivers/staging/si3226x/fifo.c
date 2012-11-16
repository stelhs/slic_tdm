/*
*  FIFO circular buffers.
*  Write by Kurochkin Michail 05.08.2010
*  email: stelhs@ya.ru
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "fifo.h"

static void inc_pointer(int *pointer, int item_size, int count_items);


/**
 * Init circular fifo buffer
 * @param buf - buffer descriptor
 * @param item_size - item size in bytes
 * @param count - count items in buffer
 * @return 0 - ok
 */
int init_fifo(struct fifo_buffer *fifo, int item_size, int count)
{
	fifo->read_pointer = 0;
	fifo->write_pointer = 0;

	fifo->buf = kzalloc(item_size * count, GFP_KERNEL);
	if(!fifo->buf)
		return -ENOMEM;

	fifo->count = count;
	fifo->item_size = item_size;
	memset(fifo->buf, 0, item_size * count);
	return 0;
}


/**
 * free fifo buffer
 * @param fifo
 */
void free_fifo(struct fifo_buffer *fifo)
{
	if(!fifo)
		return;

	if(!fifo->buf) {
		kfree(fifo);
		return;
	}

	kfree(fifo->buf);
}


/**
 * Push into fifo
 * @param fifo - fifo descriptor
 * @param item - pointer to item
 * @return 0 - ok, 1 - item pushed and buffer is full
 */
int fifo_push(struct fifo_buffer *fifo, void *item)
{
	memcpy(fifo->buf + fifo->write_pointer, item, fifo->item_size);

	inc_pointer(&fifo->write_pointer, fifo->item_size, fifo->count);

	if(fifo->write_pointer == fifo->read_pointer) { // If fifo is full
		inc_pointer(&fifo->read_pointer, fifo->item_size, fifo->count);
		return 1;
	}

	return 0;
}


/**
 * Check fifo buffer is full
 * @param fifo - fifo descriptor
 * @return 1 - buffer is full
 */
int fifo_is_full(struct fifo_buffer *fifo)
{
	int p;
	p = fifo->write_pointer;
	inc_pointer(&p, fifo->item_size, fifo->count);
	if(p == fifo->read_pointer)
		return 1;

	return 0;
}


/**
 * Check fifo is empty
 * @param fifo - fifo descriptor
 * @return 1 - buffer is empty
 */
int fifo_is_empty(struct fifo_buffer *fifo)
{
	return fifo->read_pointer == fifo->write_pointer;
}


/**
 * Pop from fifo buffer
 * @param fifo - fifo descriptor
 * @param item - pointer to poped item
 * @return 0 - ok, 1 - buffer is empty
 */
int fifo_pop(struct fifo_buffer *fifo, void *item)
{
	if(fifo->read_pointer == fifo->write_pointer)
		return 1;

	memcpy(item, fifo->buf + fifo->read_pointer, fifo->item_size);
	inc_pointer(&fifo->read_pointer, fifo->item_size, fifo->count);
	return 0;
}


/**
 * Increment pointer in circular buffer
 * @param pointer - pointer in circular buffer
 * @param item_size - item size in bytes
 * @param count_items - count items in fifo
 */
static void inc_pointer(int *pointer, int item_size, int count_items)
{
	(*pointer) += item_size;

	if(*pointer >= (count_items * item_size))
		*pointer = 0;
}

