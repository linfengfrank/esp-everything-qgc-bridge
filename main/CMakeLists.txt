idf_component_register(
    SRCS
        "main.c"
        "mavlink_task.c"
        "tof_task.c"
        "vfh.c"
        "nav_task.c"
        "at_detect.c"
        "camera_stream.c"
        "wifi_task.c"
        "odom.c"
    INCLUDE_DIRS "."
    REQUIRES
        mavlink
        vl53l5cx
        esp_driver_i2c
        esp_driver_uart
        esp_driver_gpio
        freertos
        esp_timer
        esp-apriltag
        esp32-camera
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        lwip
)

# Suppress -Waddress-of-packed-member from MAVLink and VL53L5CX generated headers
target_compile_options(__idf_main PRIVATE -Wno-address-of-packed-member)