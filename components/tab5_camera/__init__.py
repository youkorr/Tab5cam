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

# Configuration
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_SENSOR = "sensor"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_FRAMERATE = "framerate"
CONF_JPEG_QUALITY = "jpeg_quality"

# Enums
PixelFormat = tab5_camera_ns.enum("PixelFormat")
PIXEL_FORMAT_RGB565 = PixelFormat.PIXEL_FORMAT_RGB565
PIXEL_FORMAT_YUV422 = PixelFormat.PIXEL_FORMAT_YUV422
PIXEL_FORMAT_RAW8 = PixelFormat.PIXEL_FORMAT_RAW8

PIXEL_FORMATS = {
    "RGB565": PIXEL_FORMAT_RGB565,
    "YUV422": PIXEL_FORMAT_YUV422,
    "RAW8": PIXEL_FORMAT_RAW8,
}

# Dictionnaire des sensors disponibles
# Chaque sensor a son module Python qui génère son driver
AVAILABLE_SENSORS = {}

# Importer dynamiquement les sensors disponibles
def load_sensors():
    """Charge tous les sensors disponibles"""
    import logging
    logger = logging.getLogger(__name__)
    
    # SC202CS
    try:
        from .sensor_mipi_csi_sc202cs import get_sensor_info, get_driver_code
        AVAILABLE_SENSORS['sc202cs'] = {
            'info': get_sensor_info(),
            'driver': get_driver_code
        }
        logger.info("✓ SC202CS sensor loaded")
    except ImportError as e:
        logger.warning(f"SC202CS sensor not available: {e}")
    except Exception as e:
        logger.error(f"Error loading SC202CS: {e}")
    
    # OV5640
    try:
        from .sensor_mipi_csi_ov5640 import get_sensor_info, get_driver_code
        AVAILABLE_SENSORS['ov5640'] = {
            'info': get_sensor_info(),
            'driver': get_driver_code
        }
        logger.info("✓ OV5640 sensor loaded")
    except ImportError as e:
        logger.warning(f"OV5640 sensor not available: {e}")
    except Exception as e:
        logger.error(f"Error loading OV5640: {e}")
    
    # Ajouter d'autres sensors ici automatiquement
    
    if not AVAILABLE_SENSORS:
        raise cv.Invalid(
            "Aucun sensor MIPI disponible. "
            "Assurez-vous que sensor_mipi_csi_XXX.py est dans components/tab5_camera/"
        )
    
    logger.info(f"Sensors disponibles: {', '.join(AVAILABLE_SENSORS.keys())}")

# Charger les sensors au démarrage
load_sensors()

def validate_sensor(value):
    """Valide que le sensor est disponible"""
    if value not in AVAILABLE_SENSORS:
        available = ', '.join(AVAILABLE_SENSORS.keys())
        raise cv.Invalid(
            f"Sensor '{value}' non disponible. Disponibles: {available}"
        )
    return value

def auto_configure_from_sensor(config):
    """Configure automatiquement depuis les infos du sensor"""
    sensor_name = config[CONF_SENSOR]
    sensor_info = AVAILABLE_SENSORS[sensor_name]['info']
    
    # Auto-config depuis le sensor
    if 'address' not in config:
        config['address'] = sensor_info['i2c_address']
    
    # Stocker les infos du sensor
    config['_sensor_info'] = sensor_info
    config['_sensor_name'] = sensor_name
    
    return config

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="MIPI Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.Any(
                cv.int_range(min=0, max=50),
                pins.internal_gpio_output_pin_schema
            ),
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.int_range(min=6000000, max=40000000),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_SENSOR): validate_sensor,
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36)),
    auto_configure_from_sensor
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
    
    # Configuration du sensor
    sensor_name = config['_sensor_name']
    sensor_info = config['_sensor_info']
    
    cg.add(var.set_sensor_type(sensor_name))
    cg.add(var.set_sensor_address(sensor_info['i2c_address']))
    cg.add(var.set_lane_count(sensor_info['lane_count']))
    cg.add(var.set_bayer_pattern(sensor_info['bayer_pattern']))
    cg.add(var.set_lane_bitrate(sensor_info['lane_bitrate_mbps']))
    cg.add(var.set_resolution(sensor_info['width'], sensor_info['height']))
    
    # Format
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Pin reset si présent
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # ========================================================================
    # GÉNÉRATION DU CODE DU DRIVER
    # ========================================================================
    
    # Récupérer le code généré du sensor
    driver_code_func = AVAILABLE_SENSORS[sensor_name]['driver']
    driver_code = driver_code_func()
    
    # Créer le fichier header du driver généré
    driver_filename = f"sensor_{sensor_name}_generated.h"
    
    # Ajouter le code généré comme fichier global
    cg.add_global(cg.RawExpression(driver_code))
    
    # Build flags pour ESP32-P4
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
    
    # Flag pour indiquer quel sensor est utilisé
    sensor_flag = f"-DSENSOR_{sensor_name.upper()}"
    cg.add_build_flag(sensor_flag)
    
    # Log pour debug
    cg.add(cg.RawExpression(f'''
        ESP_LOGI("compile", "Sensor driver generated: {sensor_name}");
        ESP_LOGI("compile", "Resolution: {sensor_info['width']}x{sensor_info['height']}");
        ESP_LOGI("compile", "Lanes: {sensor_info['lane_count']}");
    '''))
