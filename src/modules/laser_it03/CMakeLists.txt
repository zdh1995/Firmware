set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__laser_it03
    MAIN laser_it03
#    COMPILE_FLAGS
#        -Os
    SRCS
        laser_it03.c
    DEPENDS
#        platforms__common
    ) 
