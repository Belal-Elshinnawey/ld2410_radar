idf_component_register( SRCS 
                        "src/ld2410_radar.c"
                        INCLUDE_DIRS "src"
                       )
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")