set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__laser_p40
    MAIN laser_p40
#    COMPILE_FLAGS
#        -Os
    SRCS
        laser_p40.c
    DEPENDS
#        platforms__common
    ) 
