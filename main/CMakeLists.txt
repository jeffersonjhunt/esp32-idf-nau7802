FILE(GLOB_RECURSE drv_sources src/*.cpp)

idf_component_register(
        SRCS ${drv_sources}
        INCLUDE_DIRS inc
        PRIV_REQUIRES "driver" "esp_timer"
)
