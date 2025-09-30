#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

#define SC202CS_CHIP_ID_H 0x3107
#define SC202CS_CHIP_ID_L 0x3108
#define SC202CS_CHIP_ID_VALUE 0x2311
#define SC202CS_REG_RESET 0x0103
#define SC202CS_REG_MODE_SELECT 0x0100

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera (SC202CS)...");

  if (this->ext_clock_pin_ != nullptr) {
    this->ext_clock_pin_->setup();
    this->ext_clock_pin_->digital_write(false);
  }

  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(10);
  }

  if (!this->init_camera_()) {
    ESP_LOGE(TAG, "Failed to initialize camera");
    this->mark_failed();
    return;
  }

  this->initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera initialized successfully");
}

void Tab5Camera::loop() {
  if (!this->initialized_ || !this->streaming_) {
    return;
  }
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Sensor: SC202CS (SC2356)");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->sensor_address_);

  if (this->ext_clock_pin_ != nullptr) {
    LOG_PIN("  MCLK Pin: ", this->ext_clock_pin_);
  }
  ESP_LOGCONFIG(TAG, "  MCLK Frequency: %d Hz", this->ext_clock_freq_);

  const char *resolution_str[] = {"1080P", "720P", "VGA", "QVGA"};
  ESP_LOGCONFIG(TAG, "  Resolution: %s", resolution_str[this->resolution_]);

  const char *format_str[] = {"RGB565", "YUV422", "RAW8", "JPEG"};
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", format_str[this->pixel_format_]);

  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup Failed!");
  }
}

bool Tab5Camera::init_camera_() {
#ifdef USE_ESP32
  ESP_LOGI(TAG, "Initializing ESP32 Camera...");

  this->reset_camera_();

  if (!this->configure_csi_interface_()) {
    ESP_LOGE(TAG, "Failed to configure CSI interface");
    return false;
  }

  if (!this->init_sc202cs_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize SC202CS sensor");
    return false;
  }

  memset(&this->camera_config_, 0, sizeof(camera_config_t));

  this->camera_config_.pin_pwdn = -1;
  this->camera_config_.pin_reset = this->reset_pin_ ? this->reset_pin_->pin() : -1;
  this->camera_config_.pin_xclk  = this->ext_clock_pin_ ? this->ext_clock_pin_->pin() : 36;


  this->camera_config_.pin_sccb_sda = 31;
  this->camera_config_.pin_sccb_scl = 32;
  this->camera_config_.sccb_i2c_port = 0;

  this->camera_config_.pin_d7 = -1;
  this->camera_config_.pin_d6 = -1;
  this->camera_config_.pin_d5 = -1;
  this->camera_config_.pin_d4 = -1;
  this->camera_config_.pin_d3 = -1;
  this->camera_config_.pin_d2 = -1;
  this->camera_config_.pin_d1 = -1;
  this->camera_config_.pin_d0 = -1;
  this->camera_config_.pin_vsync = -1;
  this->camera_config_.pin_href = -1;
  this->camera_config_.pin_pclk = -1;

  this->camera_config_.xclk_freq_hz = this->ext_clock_freq_;
  this->camera_config_.ledc_timer = LEDC_TIMER_0;
  this->camera_config_.ledc_channel = LEDC_CHANNEL_0;

  switch (this->pixel_format_) {
    case CAMERA_RGB565:
      this->camera_config_.pixel_format = PIXFORMAT_RGB565;
      break;
    case CAMERA_YUV422:
      this->camera_config_.pixel_format = PIXFORMAT_YUV422;
      break;
    case CAMERA_RAW8:
      this->camera_config_.pixel_format = PIXFORMAT_GRAYSCALE;
      break;
    case CAMERA_JPEG:
      this->camera_config_.pixel_format = PIXFORMAT_JPEG;
      break;
  }

  switch (this->resolution_) {
    case CAMERA_1080P:
      this->camera_config_.frame_size = FRAMESIZE_HD;
      break;
    case CAMERA_720P:
      this->camera_config_.frame_size = FRAMESIZE_HD;
      break;
    case CAMERA_VGA:
      this->camera_config_.frame_size = FRAMESIZE_VGA;
      break;
    case CAMERA_QVGA:
      this->camera_config_.frame_size = FRAMESIZE_QVGA;
      break;
  }

  this->camera_config_.jpeg_quality = this->jpeg_quality_;
  this->camera_config_.fb_count = 2;
  this->camera_config_.fb_location = CAMERA_FB_IN_PSRAM;
  this->camera_config_.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&this->camera_config_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, this->camera_config_.frame_size);
    s->set_quality(s, this->jpeg_quality_);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 300);
    s->set_gainceiling(s, (gainceiling_t)0);
    s->set_bpc(s, 0);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);
  }

  ESP_LOGI(TAG, "Camera initialized successfully");
  return true;
#else
  ESP_LOGE(TAG, "Camera only supported on ESP32");
  return false;
#endif
}

bool Tab5Camera::configure_csi_interface_() {
  ESP_LOGI(TAG, "Configuring CSI interface for ESP32-P4");
  return true;
}

bool Tab5Camera::init_sc202cs_sensor_() {
  ESP_LOGI(TAG, "Initializing SC202CS sensor at address 0x%02X", this->sensor_address_);

  delay(50);

  uint8_t chip_id_h = 0, chip_id_l = 0;

  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_H, chip_id_h)) {
    ESP_LOGE(TAG, "Failed to read chip ID high byte");
    return false;
  }

  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_L, chip_id_l)) {
    ESP_LOGE(TAG, "Failed to read chip ID low byte");
    return false;
  }

  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  ESP_LOGI(TAG, "SC202CS Chip ID: 0x%04X (expected: 0x%04X)", chip_id, SC202CS_CHIP_ID_VALUE);

  if (chip_id != SC202CS_CHIP_ID_VALUE) {
    ESP_LOGW(TAG, "Chip ID mismatch! Expected 0x%04X but got 0x%04X", SC202CS_CHIP_ID_VALUE, chip_id);
    ESP_LOGW(TAG, "Continuing anyway as some SC202CS variants may have different IDs");
  }

  ESP_LOGI(TAG, "Performing software reset of SC202CS sensor");
  if (!this->write_sensor_reg_(SC202CS_REG_RESET, 0x01)) {
    ESP_LOGE(TAG, "Failed to reset sensor");
    return false;
  }
  delay(50);

  ESP_LOGI(TAG, "SC202CS sensor initialized successfully");
  return true;
}

void Tab5Camera::reset_camera_() {
  if (this->reset_pin_ != nullptr) {
    ESP_LOGI(TAG, "Hardware reset of camera sensor");
    this->reset_pin_->digital_write(false);
    delay(20);
    this->reset_pin_->digital_write(true);
    delay(50);
  } else {
    ESP_LOGI(TAG, "No reset pin configured, skipping hardware reset");
  }
}

bool Tab5Camera::write_sensor_reg_(uint16_t reg, uint8_t value) {
  uint8_t data[3] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF),
    value
  };

  auto err = this->write(data, 3);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write register 0x%04X = 0x%02X: error %d", reg, value, err);
    return false;
  }

  ESP_LOGVV(TAG, "Write register 0x%04X = 0x%02X", reg, value);
  return true;
}

bool Tab5Camera::read_sensor_reg_(uint16_t reg, uint8_t &value) {
  uint8_t reg_addr[2] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF)
  };

  auto err = this->write_read(reg_addr, 2, &value, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%04X: error %d", reg, err);
    return false;
  }

  ESP_LOGVV(TAG, "Read register 0x%04X = 0x%02X", reg, value);
  return true;
}

bool Tab5Camera::take_snapshot() {
#ifdef USE_ESP32
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  ESP_LOGI(TAG, "Taking snapshot...");

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Failed to capture frame");
    return false;
  }

  ESP_LOGI(TAG, "Snapshot captured successfully!");
  ESP_LOGI(TAG, "  Resolution: %dx%d", fb->width, fb->height);
  ESP_LOGI(TAG, "  Size: %d bytes", fb->len);
  ESP_LOGI(TAG, "  Format: %d", fb->format);

  esp_camera_fb_return(fb);

  return true;
#else
  ESP_LOGE(TAG, "Camera not supported on this platform");
  return false;
#endif
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  if (this->streaming_) {
    ESP_LOGW(TAG, "Streaming already started");
    return true;
  }

  this->streaming_ = true;
  ESP_LOGI(TAG, "Camera streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Streaming already stopped");
    return true;
  }

  this->streaming_ = false;
  ESP_LOGI(TAG, "Camera streaming stopped");
  return true;
}

bool Tab5Camera::get_frame(std::vector<uint8_t> &buffer) {
#ifdef USE_ESP32
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Failed to get frame");
    return false;
  }

  buffer.assign(fb->buf, fb->buf + fb->len);

  esp_camera_fb_return(fb);

  return true;
#else
  return false;
#endif
}

}  // namespace tab5_camera
}  // namespace esphome

