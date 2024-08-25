idf_component_register(
        SRCS nau7802.cpp
        INCLUDE_DIRS include
        PRIV_REQUIRES "driver" "esp_timer"
)
