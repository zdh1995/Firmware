set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__laser
    MAIN laser
#    COMPILE_FLAGS
#        -Os
    SRCS
        laser.c
    DEPENDS
#        platforms__common
    ) 
