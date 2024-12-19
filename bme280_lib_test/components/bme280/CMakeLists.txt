# components/bme280/CMakeLists.txt
idf_component_register(SRCS "bme280.c"
                       INCLUDE_DIRS "."
                        PRIV_REQUIRES driver)