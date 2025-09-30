# CMakeLists.txt pour le composant tab5_camera

idf_component_register(
    SRCS "tab5_camera.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_cam esp_cam_sensor sccb_intf
    PRIV_REQUIRES esp_psram
)

# Ajouter les flags de compilation
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    BOARD_HAS_PSRAM=1
    CONFIG_CAMERA_CORE0=1
    USE_ESP32=1
)

# Si vous utilisez le BSP M5Stack, ajoutez-le ici
# REQUIRES bsp_tab5
