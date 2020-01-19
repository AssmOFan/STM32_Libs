#ifndef DT_COMMON_TYPES_
#define DT_COMMON_TYPES_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DT_STATUS_OK = 0,
  DT_STATUS_ERR,
  DT_STATUS_BUSY,
  DT_STATUS_INVALID_PARAMETERS,
	DT_STATUS_COUNT
} dt_status_t;

#ifdef __cplusplus
}
#endif

#endif // DT_COMMON_TYPES_
