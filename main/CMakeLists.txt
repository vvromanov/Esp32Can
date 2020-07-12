idf_component_register(
	SRCS 
	"uart_events_example_main.c" 
	"protocol_data.c" 
	"lz4/lz4.c"
        "mcp25xxfd_driver/drv_canfdspi_api.c"
        "mcp25xxfd_driver/drv_canfdspi_spi.c"
    INCLUDE_DIRS "." "mcp25xxfd_driver")

#add_subdirectory(lz4)

#target_link_libraries(${COMPONENT_LIB} PUBLIC lz4)

