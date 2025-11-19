# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c" "pmw3901.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp32-camera
        esp_timer
        nvs_flash
        driver
        esp_driver_gpio
        esp_driver_uart
        esp_driver_spi
        esp_psram
)

# Optimisations de compilation spécifiques
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O3
    -funroll-loops
    -ffast-math

)

# Link-time optimization
target_link_options(${COMPONENT_LIB} PRIVATE
    -flto
)