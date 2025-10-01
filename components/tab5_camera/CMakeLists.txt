# CMakeLists.txt pour le composant tab5_camera

idf_component_register(
    SRCS 
        "tab5_camera.cpp"
        "esp_cam_sensor.c"
        "esp_sccb_i2c.c"
        "esp_cam_sensor_detect.c"
        "sensors/sc202cs/sc202cs.c"
        "sensors/sc202cs/sc202cs_settings.c"
    INCLUDE_DIRS 
        "."
        "sensors/sc202cs"
        "sensors/sc202cs/private_include"
    REQUIRES 
        driver
        esp_timer
        freertos
        esp_driver_i2c
        esp_mm
    PRIV_REQUIRES 
        esp_psram
)

# Ajouter les flags de compilation
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    BOARD_HAS_PSRAM=1
    CONFIG_CAMERA_CORE0=1
    CONFIG_CAMERA_SC202CS=1
    CONFIG_CAMERA_SC202CS_AUTO_DETECT_MIPI_INTERFACE_SENSOR=1
    CONFIG_CAMERA_SC202CS_MIPI_IF_FORMAT_INDEX_DAFAULT=0
    CONFIG_CAMERA_SC202CS_MAX_SUPPORT=1
    CONFIG_CAMERA_SC202CS_ABSOLUTE_GAIN_LIMIT=63008
    CONFIG_CAMERA_SC202CS_ANA_GAIN_PRIORITY=1
)

# Si vous utilisez le BSP M5Stack, ajoutez-le ici
# REQUIRES bsp_tab5
