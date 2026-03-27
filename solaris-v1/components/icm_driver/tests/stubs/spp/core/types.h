#ifndef STUB_TYPES_H
#define STUB_TYPES_H

#include <stdbool.h>

typedef unsigned char      spp_uint8_t;
typedef unsigned short     spp_uint16_t;
typedef unsigned long      spp_uint32_t;
typedef signed char        spp_int8_t;
typedef signed short       spp_int16_t;
typedef signed long        spp_int32_t;
typedef unsigned long long spp_uint64_t;
typedef signed long long   spp_int64_t;
typedef bool               spp_bool_t;
typedef spp_uint32_t       spp_size_t;

typedef void (*spp_task_func_t)(void *arg);
typedef void *spp_task_handle_t;

#endif
