# general source files
list( APPEND GENSRCS 
    "iot_light.c"
    "i2c.c"
    "i2c_bus.c"
    "spi.c"
    "rotary.c"
    "ws2812.c"
    "qma7981.c"
    "button_irq.c" 
    "button_poll.c"
    "touch_button.c" 
)
# general include files
list( APPEND GENINCS
    "." 
    "../../main"
    "../commandline"
    "../iotools"
    "../powerstep01"
)
# general dependencies
list( APPEND GENREQS
    soc 
)


idf_component_register( 
    SRCS ${GENSRCS}
    INCLUDE_DIRS ${GENINCS}
    REQUIRES ${GENREQS}
)
