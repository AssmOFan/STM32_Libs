#ifndef DT_RING_H_
#define DT_RING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "dt_common_types.h"

    /* Types ------------------------------------- */
    typedef struct {
        uint8_t* storage; 
        uint16_t buf_size;
        uint16_t tail;
        uint16_t head;
        uint16_t count;
    } dt_ring_t;

    /* Macros ------------------------------------ */
#define DT_RING_GET_OBJ(name) name##_obj

#define DT_RING_INIT(name, buf_sz)            \
    uint8_t name##_storage[buf_sz] = {0};     \
    dt_ring_t DT_RING_GET_OBJ(name) = {       \
      name##_storage,                         \
      buf_sz,                                 \
      0,                                      \
      0,                                      \
      0                                       \
    };                                        \
    dt_ring_t* name = & DT_RING_GET_OBJ(name)
        
#define DT_RING_EXPORT(name) extern dt_ring_t* name

/* Functions --------------------------------- */

    dt_status_t dt_ring_init(dt_ring_t* ring);
    dt_status_t dt_ring_clear(dt_ring_t* ring);
    dt_status_t dt_ring_push(dt_ring_t* ring, uint8_t elem);
    dt_status_t dt_ring_pop(dt_ring_t* ring, uint8_t* elem_p);
    dt_status_t dt_ring_push_arr(dt_ring_t* ring, uint8_t* elem_p, uint16_t len);
    dt_status_t dt_ring_pop_arr(dt_ring_t* ring, uint8_t* elem_p, uint16_t size, uint16_t* len);
    
    dt_status_t dt_ring_clear_from_isr(dt_ring_t* ring);
    dt_status_t dt_ring_push_from_isr(dt_ring_t* ring, uint8_t elem);
    dt_status_t dt_ring_pop_from_isr(dt_ring_t* ring, uint8_t* elem_p);
    dt_status_t dt_ring_push_arr_from_isr(dt_ring_t* ring, uint8_t* elem_p, uint16_t len);
    dt_status_t dt_ring_pop_arr_from_isr(dt_ring_t* ring, uint8_t* elem_p, uint16_t size, uint16_t* len);

    uint16_t dt_ring_get_count(dt_ring_t* ring);

#ifdef __cplusplus
}
#endif
#endif // DT_RING_H_
