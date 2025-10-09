import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_FREQUENCY,
)
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c", "esp32"]
MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Configuration simplifiée
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_SENSOR = "sensor"
CONF_LANE = "lane"
CONF_ADDRESS_SENSOR = "address_sensor"
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_FRAMERATE = "framerate"
CONF_JPEG_QUALITY = "jpeg_quality"

# Enums C++
CameraResolution = tab5_camera_ns.enum("CameraResolution")
RESOLUTION_720P = CameraResolution.RESOLUTION_720P

PixelFormat = tab5_camera_ns.enum("PixelFormat")
PIXEL_FORMAT_RGB565 = PixelFormat.PIXEL_FORMAT_RGB565
PIXEL_FORMAT_YUV422 = PixelFormat.PIXEL_FORMAT_YUV422
PIXEL_FORMAT_RAW8 = PixelFormat.PIXEL_FORMAT_RAW8

# Mapping YAML
PIXEL_FORMATS = {
    "RGB565": PIXEL_FORMAT_RGB565,
    "YUV422": PIXEL_FORMAT_YUV422,
    "RAW8": PIXEL_FORMAT_RAW8,
}

# Sensors supportés
SUPPORTED_SENSORS = {
    "sc202cs": {
        "default_address": 0x36,
        "lanes": 1,
        "bayer_pattern": 3,  # BGGR
    }
}

def validate_sensor_config(config):
    """Valide la configuration du sensor"""
    sensor_type = config.get(CONF_SENSOR, "sc202cs")
    
    if sensor_type not in SUPPORTED_SENSORS:
        raise cv.Invalid(f"Sensor '{sensor_type}' non supporté. Supportés: {', '.join(SUPPORTED_SENSORS.keys())}")
    
    # Utiliser les valeurs par défaut du sensor si non spécifiées
    sensor_info = SUPPORTED_SENSORS[sensor_type]
    
    if CONF_ADDRESS_SENSOR not in config:
        config[CONF_ADDRESS_SENSOR] = sensor_info["default_address"]
    
    if CONF_LANE not in config:
        config[CONF_LANE] = sensor_info["lanes"]
    
    return config

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.Any(
                cv.int_range(min=0, max=50),
                pins.internal_gpio_output_pin_schema
            ),
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.int_range(min=6000000, max=40000000),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SENSOR, default="sc202cs"): cv.string,
            cv.Optional(CONF_LANE): cv.int_range(min=1, max=2),
            cv.Optional(CONF_ADDRESS_SENSOR): cv.i2c_address,
            cv.Optional(CONF_RESOLUTION, default="720P"): cv.string,  # Seulement 720P
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36)),
    validate_sensor_config
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    
    # Pin externe de l'horloge
    ext_clock_pin_config = config[CONF_EXTERNAL_CLOCK_PIN]
    if isinstance(ext_clock_pin_config, int):
        cg.add(var.set_external_clock_pin(ext_clock_pin_config))
    else:
        pin_num = ext_clock_pin_config[pins.CONF_NUMBER]
        cg.add(var.set_external_clock_pin(pin_num))
    
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    
    # Configuration sensor
    sensor_type = config[CONF_SENSOR]
    sensor_info = SUPPORTED_SENSORS[sensor_type]
    
    cg.add(var.set_sensor_type(sensor_type))
    cg.add(var.set_sensor_address(config[CONF_ADDRESS_SENSOR]))
    cg.add(var.set_lane_count(config[CONF_LANE]))
    cg.add(var.set_bayer_pattern(sensor_info["bayer_pattern"]))
    
    # Format - toujours 720P
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Pin reset si présent
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # Build flags pour ESP32-P4
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag(f"-DCONFIG_CAMERA_{sensor_type.upper()}=1")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
