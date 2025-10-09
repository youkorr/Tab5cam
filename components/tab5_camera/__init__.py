import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_FREQUENCY,
    CONF_ADDRESS,
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
CONF_LANE = "lane"
CONF_ADDRESS_SENSOR = "address_sensor"
CONF_RESOLUTION = "resolution"
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

# Résolutions supportées (pour l'instant uniquement 720P)
RESOLUTIONS = {
    "720P": (1280, 720),
    # Autres résolutions à ajouter plus tard:
    # "1080P": (1920, 1080),
    # "VGA": (640, 480),
    # "QVGA": (320, 240),
}

# Dictionnaire des sensors disponibles
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
    
    # Pour ajouter un nouveau sensor, créez sensor_mipi_csi_XXX.py
    # avec les fonctions get_sensor_info() et get_driver_code()
    # puis ajoutez son import ici
    
    if not AVAILABLE_SENSORS:
        raise cv.Invalid(
            "Aucun sensor MIPI disponible. "
            "Assurez-vous que sensor_mipi_csi_sc202cs.py est dans components/tab5_camera/"
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

def validate_resolution(value):
    """Valide la résolution (pour l'instant uniquement 720P)"""
    if isinstance(value, str):
        value_upper = value.upper()
        if value_upper not in RESOLUTIONS:
            available = ', '.join(RESOLUTIONS.keys())
            raise cv.Invalid(
                f"Résolution '{value}' non supportée. Disponibles: {available}"
            )
        return RESOLUTIONS[value_upper]
    raise cv.Invalid("Le format de résolution doit être une chaîne (ex: '720P')")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
        
        # Pins et horloge
        cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.Any(
            cv.int_range(min=0, max=50),
            pins.internal_gpio_output_pin_schema
        ),
        cv.Optional(CONF_FREQUENCY, default=24000000): cv.int_range(min=6000000, max=40000000),
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        
        # Configuration sensor - CONTRÔLE MANUEL
        cv.Required(CONF_SENSOR): validate_sensor,
        cv.Optional(CONF_LANE, default=1): cv.int_range(min=1, max=4),
        cv.Optional(CONF_ADDRESS_SENSOR, default=0x36): cv.i2c_address,
        cv.Optional(CONF_RESOLUTION, default="720P"): validate_resolution,
        
        # Format et qualité
        cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
        cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
        cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x36))


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
    
    # Configuration du sensor - DEPUIS LE YAML
    sensor_name = config[CONF_SENSOR]
    width, height = config[CONF_RESOLUTION]
    
    cg.add(var.set_sensor_type(sensor_name))
    cg.add(var.set_sensor_address(config[CONF_ADDRESS_SENSOR]))
    cg.add(var.set_lane_count(config[CONF_LANE]))
    cg.add(var.set_resolution(width, height))
    
    # Récupérer les infos du sensor pour les métadonnées nécessaires
    sensor_info = AVAILABLE_SENSORS[sensor_name]['info']
    cg.add(var.set_bayer_pattern(sensor_info['bayer_pattern']))
    cg.add(var.set_lane_bitrate(sensor_info['lane_bitrate_mbps']))
    
    # Format
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Pin reset si présent
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # ========================================================================
    # GÉNÉRATION DU CODE DU DRIVER ET DE LA FONCTION FACTORY
    # ========================================================================
    
    import os
    
    # Générer le code de TOUS les sensors disponibles
    all_drivers_code = ""
    
    for sensor_id, sensor_data in AVAILABLE_SENSORS.items():
        driver_code_func = sensor_data['driver']
        all_drivers_code += driver_code_func() + "\n\n"
    
    # Générer la fonction factory globale
    factory_code = f'''
namespace esphome {{
namespace tab5_camera {{

// =============================================================================
// FONCTION FACTORY GLOBALE (inline pour éviter définitions multiples)
// Créée automatiquement depuis tous les sensors disponibles
// =============================================================================

inline ISensorDriver* create_sensor_driver(const std::string& sensor_type, i2c::I2CDevice* i2c) {{
'''
    
    # Ajouter les conditions pour chaque sensor
    for sensor_id in AVAILABLE_SENSORS.keys():
        sensor_upper = sensor_id.upper()
        factory_code += f'''
    if (sensor_type == "{sensor_id}") {{
        return new {sensor_upper}Adapter(i2c);
    }}
'''
    
    factory_code += f'''
    
    // Sensor non trouvé
    ESP_LOGE("tab5_camera", "Unknown sensor type: %s", sensor_type.c_str());
    return nullptr;
}}

}}  // namespace tab5_camera
}}  // namespace esphome
'''
    
    # Combiner tout le code
    complete_code = all_drivers_code + factory_code
    
    # Écrire le fichier généré physiquement
    component_dir = os.path.dirname(__file__)
    generated_file_path = os.path.join(component_dir, "tab5_camera_drivers_generated.h")
    
    with open(generated_file_path, 'w') as f:
        f.write("// FICHIER AUTO-GÉNÉRÉ - NE PAS MODIFIER MANUELLEMENT\n")
        f.write("// Généré par __init__.py lors de la compilation ESPHome\n\n")
        f.write("#ifndef TAB5_CAMERA_DRIVERS_GENERATED_H\n")
        f.write("#define TAB5_CAMERA_DRIVERS_GENERATED_H\n\n")
        f.write(complete_code)
        f.write("\n#endif  // TAB5_CAMERA_DRIVERS_GENERATED_H\n")
    
    # Build flags pour ESP32-P4
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    cg.add_build_flag("-DCONFIG_CAMERA_CORE0=1")
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
    
    # Log pour debug
    cg.add(cg.RawExpression(f'''
        ESP_LOGI("compile", "✓ Camera configuration:");
        ESP_LOGI("compile", "  Sensor: {sensor_name}");
        ESP_LOGI("compile", "  Resolution: {width}x{height}");
        ESP_LOGI("compile", "  Lanes: {config[CONF_LANE]}");
        ESP_LOGI("compile", "  Address: 0x{config[CONF_ADDRESS_SENSOR]:02X}");
        ESP_LOGI("compile", "  Format: {config[CONF_PIXEL_FORMAT]}");
        ESP_LOGI("compile", "  FPS: {config[CONF_FRAMERATE]}");
    '''))
