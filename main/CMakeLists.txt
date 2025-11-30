# main/CMakeLists.txt

# Base source files
set(SRCS
    "main.c"
    "drivers/pmw3901.c"
    "drivers/dts6012m.c"
    "drivers/tfmini.c"
    "drivers/mtf02.c"
    "drivers/camera_optical_flow.c"
)

# Base include directories
set(INCLUDE_DIRS "." "drivers")

# Base requirements
set(REQUIRES
    esp32-camera
    esp_timer
    nvs_flash
    driver
    esp_driver_gpio
    esp_driver_uart
    esp_driver_spi
    esp_psram
    esp_netif
    esp_http_server
    esp_wifi
    esp_netif
    esp_event
    
)

# Conditionally add webserver if enabled
if(CONFIG_ENABLE_WEBSERVER)
    message(STATUS "Webserver enabled - adding WiFi and HTTP server components")
    list(APPEND SRCS
        "webserver/wifi_station.c"
        "webserver/app_httpd.c"
        "webserver/webserver_ctrl.c"
    )
    list(APPEND INCLUDE_DIRS "webserver")
    list(APPEND REQUIRES
        
        
    )
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES ${REQUIRES}
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