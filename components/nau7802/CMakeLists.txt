FILE(GLOB_RECURSE drv_sources *.cpp)

idf_component_register(
        SRCS ${drv_sources}
        INCLUDE_DIRS include
        PRIV_REQUIRES "driver" "esp_timer"
)
