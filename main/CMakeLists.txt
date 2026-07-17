idf_component_register(SRCS "esp32-bluetooth-mouse.c"
                    "drivers/battery.c" 
                    "drivers/encoder.c"
                    "drivers/button.c"
                    "drivers/mx8650.c"
                    "ble/ble_hid.c"
                    INCLUDE_DIRS ".")
