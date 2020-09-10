set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__laser_M88
    MAIN laser_M88
#    COMPILE_FLAGS
#        -Os
    SRCS
        laser_M88.c
    DEPENDS
#        platforms__common
    ) 
