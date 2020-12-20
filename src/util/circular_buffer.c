#include "circular_buffer.h"

uint32_t circular_buffer_free(volatile circular_buffer_t *c) {
  if (c->head >= c->tail) {
    return (c->size - c->head) + c->tail;
  }
  return (c->tail - c->head);
}

uint8_t circular_buffer_write(volatile circular_buffer_t *c, uint8_t data) {
  uint32_t next = c->head + 1;
  if (next >= c->size)
    next = 0;

  if (next == c->tail)
    return 0;

  c->buffer[c->head] = data;
  c->head = next;
  return 1;
}

uint32_t circular_buffer_available(volatile circular_buffer_t *c) {
  if (c->head >= c->tail) {
    return c->head - c->tail;
  }
  return c->size + c->head - c->tail;
}

uint8_t circular_buffer_read(volatile circular_buffer_t *c, uint8_t *data) {
  if (c->head == c->tail)
    return 0;

  uint32_t next = c->tail + 1;
  if (next >= c->size)
    next = 0;

  *data = c->buffer[c->tail];
  c->tail = next;
  return 1;
}

void circular_buffer_clear(volatile circular_buffer_t *c) {
  c->tail = c->head;
}
