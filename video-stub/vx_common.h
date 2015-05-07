#ifndef VX_COMMON_H
#define VX_COMMON_H

#include <vector>
#include <initializer_list>
#include "VX/vx.h"
#include "vx_debug.h"
#include "add_kernels/add_kernels.h"

#define INIT_DEBUG(zones, num) \
    { \
        for(int i = 0; i < (num); i++) \
            vx_set_debug_zone((zones)[i]); \
    }

#define CHECK(var) \
    { \
        if( var ) \
        {\
            VX_PRINT(VX_ZONE_ERROR, "Expression " #var " isn't true\n"); \
            return VX_FAILURE; \
        } \
    }

#define CHECK_NULL(var) \
    { \
        if( (var) == NULL ) \
        {\
            VX_PRINT(VX_ZONE_ERROR, "NULL reference of " #var "\n"); \
            return VX_FAILURE; \
        } \
    }

#define CHECK_STATUS(var) \
    { \
        if( (var) != VX_SUCCESS ) \
        { \
            VX_PRINT(VX_ZONE_ERROR, #var "return bad status\n"); \
            return VX_FAILURE; \
        } \
    }

#endif // VX_COMMON_H
