#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

// Les drivers auto-générés s'incluent ici via la compilation
// Chaque sensor_mipi_csi_XXX.py génère son driver qui implémente ISensorDriver

#ifdef USE_ESP32_VARIANT_ESP32P4

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// ============================================================================
// ADAPTATEUR POUR CONNECTER LE DRIVER AUTO-GÉNÉRÉ À L'INTERFACE
// ============================================================================

// Include du driver généré (le nom du fichier dépend du sensor)
// Ceci sera injecté par le système de build selon sensor_type_
#ifdef SENSOR_SC202CS
  //#include "sensor_sc202cs_generated.h"
  using namespace esphome::mipi_camera;
  
  class SC202CSAdapter : public ISensorDriver {
  public:
    SC202CSAdapter(i2c::I2CDevice* i2c) : driver_(i2c) {}
    
    const char* get_name() const override { 
      return driver_.get_metadata().name; 
    }
    uint16_t get_pid() const override { 
      return driver_.get_metadata().pid; 
    }
    uint8_t get_i2c_address() const override { 
      return driver_.get_metadata().i2c_address; 
    }
    uint8_t get_lane_count() const override { 
      return driver_.get_metadata().lane_count; 
    }
    uint8_t get_bayer_pattern() const override { 
      return driver_.get_metadata().bayer_pattern; 
    }
    uint16_t get_lane_bitrate_mbps() const override { 
      return driver_.get_metadata().lane_bitrate_mbps; 
    }
    uint16_t get_width() const override { 
      return driver_.get_metadata().width; 
    }
    uint16_t get_height() const override { 
      return driver_.get_metadata().height; 
    }
    uint8_t get_fps() const override { 
      return driver_.get_metadata().fps; 
    }
    
    esp_err_t init() override { 
      return driver_.init(); 
    }
    esp_err_t read_id(uint16_t* pid) override { 
      return driver_.read_id(pid); 
    }
    esp_err_t start_stream() override { 
      return driver_.start_stream(); 
    }
    esp_err_t stop_stream() override { 
      return driver_.stop_stream(); 
    }
    esp_err_t set_gain(uint32_t gain_index) override { 
      return driver_.set_gain(gain_index); 
    }
    esp_err_t set_exposure(uint32_t exposure) override { 
      return driver_.set_exposure(exposure); 
    }
    esp_err_t write_register(uint16_t reg, uint8_t value) override { 
      return driver_.write_register(reg, value); 
    }
    esp_err_t read_register(uint16_t reg, uint8_t* value) override { 
      return driver_.read_register(reg, value); 
    }
    
  private:
    SC202CSDriver driver_;
  };
#endif

#ifdef SENSOR_OV5640
  #include "sensor_ov5640_generated.h"
  // Même pattern pour OV5640
#endif

// ============================================================================
// IMPLÉMENTATION TAB5 CAMERA (GÉNÉRIQUE)
// ============================================================================

void Tab5Camera::setup() {
  ESP_LOGI(TAG, "🎥 Init MIPI Camera");
  ESP_LOGI(TAG, "   Sensor type: %s", this->sensor_type_.c_str());
  
#ifdef USE_ESP32_VARIANT_ESP32P4
  
  // 1. Init pins
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }
  
  // 2. Créer le driver du sensor
  if (!this->create_sensor_driver_()) {
    ESP_LOGE(TAG, "❌ Driver creation failed");
    this->mark_failed();
    return;
  }
  
  // 3. Init sensor
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Sensor init failed");
    this->mark_failed();
    return;
  }
  
  // 4. Init LDO
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "❌ LDO init failed");
    this->mark_failed();
    return;
  }
  
  // 5. Init CSI
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ CSI init failed");
    this->mark_failed();
    return;
  }
  
  // 6. Init ISP
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ ISP init failed");
    this->mark_failed();
    return;
  }
  
  // 7. Allouer buffers
  if (!this->allocate_buffer_()) {
    ESP_LOGE(TAG, "❌ Buffer alloc failed");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "✅ Camera ready (%ux%u)", this->width_, this->height_);
  
#else
  ESP_LOGE(TAG, "❌ ESP32-P4 required");
  this->mark_failed();
#endif
}

#ifdef USE_ESP32_VARIANT_ESP32P4

bool Tab5Camera::create_sensor_driver_() {
  ESP_LOGI(TAG, "Creating driver for: %s", this->sensor_type_.c_str());
  
  // Factory pattern - créer le bon driver selon sensor_type_
  #ifdef SENSOR_SC202CS
  if (this->sensor_type_ == "sc202cs") {
    this->sensor_driver_ = new SC202CSAdapter(this);
    ESP_LOGI(TAG, "✓ SC202CS driver created");
    return true;
  }
  #endif
  
  #ifdef SENSOR_OV5640
  if (this->sensor_type_ == "ov5640") {
    // this->sensor_driver_ = new OV5640Adapter(this);
    ESP_LOGI(TAG, "✓ OV5640 driver created");
    return true;
  }
  #endif
  
  ESP_LOGE(TAG, "Unknown sensor: %s", this->sensor_type_.c_str());
  return false;
}

bool Tab5Camera::init_sensor_() {
  if (!this->sensor_driver_) {
    ESP_LOGE(TAG, "No sensor driver");
    return false;
  }
  
  ESP_LOGI(TAG, "Init sensor: %s", this->sensor_driver_->get_name());
  
  // Récupérer les métadonnées du sensor
  this->width_ = this->sensor_driver_->get_width();
  this->height_ = this->sensor_driver_->get_height();
  this->lane_count_ = this->sensor_driver_->get_lane_count();
  this->bayer_pattern_ = this->sensor_driver_->get_bayer_pattern();
  this->lane_bitrate_mbps_ = this->sensor_driver_->get_lane_bitrate_mbps();
  
  ESP_LOGI(TAG, "   Resolution: %ux%u", this->width_, this->height_);
  ESP_LOGI(TAG, "   Lanes: %u", this->lane_count_);
  ESP_LOGI(TAG, "   Bayer: %u", this->bayer_pattern_);
  ESP_LOGI(TAG, "   Bitrate: %u Mbps", this->lane_bitrate_mbps_);
  
  // Vérifier l'ID
  uint16_t pid = 0;
  esp_err_t ret = this->sensor_driver_->read_id(&pid);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read sensor ID");
    return false;
  }
  
  if (pid != this->sensor_driver_->get_pid()) {
    ESP_LOGE(TAG, "Wrong PID: 0x%04X (expected 0x%04X)", 
             pid, this->sensor_driver_->get_pid());
    return false;
  }
  
  ESP_LOGI(TAG, "✓ Sensor ID: 0x%04X", pid);
  
  // Initialiser le sensor
  ret = this->sensor_driver_->init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Sensor init failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ Sensor initialized");
  return true;
}

bool Tab5Camera::init_ldo_() {
  ESP_LOGI(TAG, "Init LDO MIPI");
  
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LDO failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ LDO OK (2.5V)");
  return true;
}

bool Tab5Camera::init_csi_() {
  ESP_LOGI(TAG, "Init MIPI-CSI");
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = this->width_;
  csi_config.v_res = this->height_;
  csi_config.lane_bit_rate_mbps = this->lane_bitrate_mbps_;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = this->lane_count_;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 2;
  
  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI failed: %d", ret);
    return false;
  }
  
  // Callbacks
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = Tab5Camera::on_csi_new_frame_,
    .on_trans_finished = Tab5Camera::on_csi_frame_done_,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Callbacks failed: %d", ret);
    return false;
  }
  
  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Enable CSI failed: %d", ret);
    return false;
  }
  
  ESP_LOGI(TAG, "✓ CSI OK");
  return true;
}

bool Tab5Camera::init_isp_() {
  ESP_LOGI(TAG, "Init ISP");
  
  uint32_t isp_clock_hz = 120000000;
  
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = this->width_;
  isp_config.v_res = this->height_;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = isp_clock_hz;
  isp_config.bayer_order = (color_raw_element_order_t)this->bayer_pattern_;
  
  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP creation failed: 0x%x", ret);
    return false;
  }
  
  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP enable failed: 0x%x", ret);
    esp_isp_del_processor(this->isp_handle_);
    this->isp_handle_ = nullptr;
    return false;
  }
  
  ESP_LOGI(TAG, "✓ ISP OK (bayer=%u)", this->bayer_pattern_);
  return true;
}

bool Tab5Camera::allocate_buffer_() {
  this->frame_buffer_size_ = this->width_ * this->height_ * 2;  // RGB565
  
  this->frame_buffers_[0] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  this->frame_buffers_[1] = (uint8_t*)heap_caps_aligned_alloc(
    64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM
  );
  
  if (!this->frame_buffers_[0] || !this->frame_buffers_[1]) {
    ESP_LOGE(TAG, "Buffer alloc failed");
    return false;
  }
  
  this->current_frame_buffer_ = this->frame_buffers_[0];
  
  ESP_LOGI(TAG, "✓ Buffers: 2x%u bytes", this->frame_buffer_size_);
  return true;
}

bool IRAM_ATTR Tab5Camera::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  trans->buffer = cam->frame_buffers_[cam->buffer_index_];
  trans->buflen = cam->frame_buffer_size_;
  return false;
}

bool IRAM_ATTR Tab5Camera::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  Tab5Camera *cam = (Tab5Camera*)user_data;
  
  if (trans->received_size > 0) {
    cam->frame_ready_ = true;
    cam->buffer_index_ = (cam->buffer_index_ + 1) % 2;
  }
  
  return false;
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }
  
  ESP_LOGI(TAG, "Start streaming");
  
  // Démarrer sensor
  if (this->sensor_driver_) {
    esp_err_t ret = this->sensor_driver_->start_stream();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Sensor start failed: %d", ret);
      return false;
    }
    delay(100);
  }
  
  // Démarrer CSI
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI start failed: %d", ret);
    return false;
  }
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming active");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  esp_cam_ctlr_stop(this->csi_handle_);
  
  if (this->sensor_driver_) {
    this->sensor_driver_->stop_stream();
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "⏹ Streaming stopped");
  return true;
}

bool Tab5Camera::capture_frame() {
  if (!this->streaming_) {
    return false;
  }
  
  bool was_ready = this->frame_ready_;
  if (was_ready) {
    this->frame_ready_ = false;
    uint8_t last_buffer = (this->buffer_index_ + 1) % 2;
    this->current_frame_buffer_ = this->frame_buffers_[last_buffer];
  }
  
  return was_ready;
}

#endif  // USE_ESP32_VARIANT_ESP32P4

void Tab5Camera::loop() {
  // Géré par callbacks ISR
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI Camera:");
  if (this->sensor_driver_) {
    ESP_LOGCONFIG(TAG, "  Sensor: %s", this->sensor_driver_->get_name());
    ESP_LOGCONFIG(TAG, "  PID: 0x%04X", this->sensor_driver_->get_pid());
  } else {
    ESP_LOGCONFIG(TAG, "  Sensor: %s (driver not loaded)", this->sensor_type_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Format: RGB565");
  ESP_LOGCONFIG(TAG, "  Lanes: %u", this->lane_count_);
  ESP_LOGCONFIG(TAG, "  Bayer: %u", this->bayer_pattern_);
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "YES" : "NO");
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
























