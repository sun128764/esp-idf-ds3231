idf_component_register(
    INCLUDE_DIRS "include"
    SRCS
        "source/io.c"
        "source/driver.c"
    REQUIRES esp_event driver
)