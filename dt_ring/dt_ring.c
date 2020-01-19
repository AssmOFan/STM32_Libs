#include <string.h>
#include "dt_ring.h"
#include "dt_porting.h"

/* Local macros ------------------------------ */
#define DT_RING_IS_VALID(ring)  \
    ((ring != NULL &&           \
      ring->buf_size > 0 &&     \
      ring->storage != NULL))

#define DT_RING_CLEAR(ring) do { \
    ring->head = 0;              \
    ring->tail = 0;              \
    ring->count = 0;             \
    } while (0);                 

#define DT_RING_NEXT_HEAD(ring) ((ring->head + 1) % ring->buf_size)
#define DT_RING_NEXT_TAIL(ring) ((ring->tail + 1) % ring->buf_size)
#define DT_RING_IS_FULL(ring)   (DT_RING_NEXT_HEAD(ring) == ring->tail)
#define DT_RING_IS_EMPTY(ring)  (ring->head == ring->tail)

/* Local prototypes -------------------------- */

/* Public fucntions -------------------------- */
dt_status_t dt_ring_init(dt_ring_t* ring)
{
    if (DT_RING_IS_VALID(ring)) {
        DT_RING_CLEAR(ring);
        return DT_STATUS_OK;
    }

    return DT_STATUS_ERR;
}

dt_status_t dt_ring_clear(dt_ring_t* ring)
{
  dt_status_t status = DT_STATUS_ERR;
  DT_PORTING_ENTER_CRITICAL();
  status = dt_ring_clear_from_isr(ring);
  DT_PORTING_EXIT_CRITICAL();
  return status;
}

dt_status_t dt_ring_push(dt_ring_t* ring, uint8_t elem)
{
    if (!DT_RING_IS_VALID(ring)) {
        return DT_STATUS_ERR;
    }

    dt_status_t status = DT_STATUS_ERR;
    DT_PORTING_ENTER_CRITICAL();
    status = dt_ring_push_from_isr(ring, elem);
    DT_PORTING_EXIT_CRITICAL();
    return status;
}

dt_status_t dt_ring_pop(dt_ring_t* ring, uint8_t* elem_p)
{
    if (!DT_RING_IS_VALID(ring) || elem_p == NULL) {
        return DT_STATUS_ERR;
    }
    
    dt_status_t status = DT_STATUS_ERR;
    DT_PORTING_ENTER_CRITICAL();
    status = dt_ring_pop_from_isr(ring, elem_p);
    DT_PORTING_EXIT_CRITICAL();
    return status;
}


dt_status_t dt_ring_push_arr(dt_ring_t* ring, uint8_t* elem_p, uint16_t len)
{
  dt_status_t status = DT_STATUS_OK;
  
  DT_PORTING_ENTER_CRITICAL();
  status = dt_ring_push_arr_from_isr(ring, elem_p, len);
  DT_PORTING_EXIT_CRITICAL();
  return status;
}

dt_status_t dt_ring_pop_arr(dt_ring_t* ring, uint8_t* elem_p, uint16_t size, uint16_t* len)
{
  dt_status_t status = DT_STATUS_OK;
  
  DT_PORTING_ENTER_CRITICAL();
  status = dt_ring_pop_arr_from_isr(ring, elem_p, size, len);
  DT_PORTING_EXIT_CRITICAL();
  return status;
}

/* Functions to be used from interrupts. Don't have critical sections */
dt_status_t dt_ring_clear_from_isr(dt_ring_t* ring)
{   
    if (DT_RING_IS_VALID(ring)) {
        DT_RING_CLEAR(ring);
        return DT_STATUS_OK;
    }
    return DT_STATUS_ERR;
}

dt_status_t dt_ring_push_from_isr(dt_ring_t* ring, uint8_t elem)
{
    if (!DT_RING_IS_VALID(ring)) {
        return DT_STATUS_ERR;
    }

    if(ring->count < (ring->buf_size - 1)) {
      ring->count++;
    }
    
    ring->storage[ring->head] = elem;
    if (DT_RING_IS_FULL(ring)) {
        ring->tail = DT_RING_NEXT_TAIL(ring);
    }
    ring->head = DT_RING_NEXT_HEAD(ring);
    
    return DT_STATUS_OK;
}

dt_status_t dt_ring_pop_from_isr(dt_ring_t* ring, uint8_t* elem_p)
{
    if (!DT_RING_IS_VALID(ring) || elem_p == NULL) {
        return DT_STATUS_ERR;
    }

    if (DT_RING_IS_EMPTY(ring)) {
        return DT_STATUS_ERR;
    }
    
    ring->count--;
    *elem_p = ring->storage[ring->tail];
    ring->tail = DT_RING_NEXT_TAIL(ring);

    return DT_STATUS_OK;
}

dt_status_t dt_ring_push_arr_from_isr(dt_ring_t* ring, uint8_t* elem_p, uint16_t len)
{
  dt_status_t status = DT_STATUS_OK;
  
  while(len > 0 && DT_STATUS_OK == (status = dt_ring_push_from_isr(ring, *elem_p))) {
    len--;
    elem_p++;
  }
  return status;
}

dt_status_t dt_ring_pop_arr_from_isr(dt_ring_t* ring, uint8_t* elem_p, uint16_t size, uint16_t* len)
{
  dt_status_t status = DT_STATUS_OK;
  
  *len = 0;
  while(size > 0 && DT_STATUS_OK == (status = dt_ring_pop_from_isr(ring, elem_p))) {
    size--;
    elem_p++;
    (*len)++;
  }
  return status;
}

uint16_t dt_ring_get_count(dt_ring_t* ring)
{  
  if (!DT_RING_IS_VALID(ring)) {
    return 0;
  }
  
  return ring->count;
}


/* Local fucntions --------------------------- */
