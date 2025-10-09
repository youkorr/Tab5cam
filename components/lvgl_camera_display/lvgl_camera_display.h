#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_esphome.h"
#include "../tab5_camera/tab5_camera.h"

namespace esphome {
namespace lvgl_camera_display {

class LVGLCameraDisplay : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  void set_camera(tab5_camera::Tab5Camera *camera) { this->camera_ = camera; }
  void set_canvas_id(const std::string &canvas_id) { this->canvas_id_ = canvas_id; }
  void set_update_interval(uint32_t interval_ms) { this->update_interval_ = interval_ms; }
  
  void configure_canvas(lv_obj_t *canvas);
  
  // Nouvelles fonctions pour resize
  void resize_canvas(uint16_t width, uint16_t height, int16_t x = 0, int16_t y = 0, uint16_t radius = 0);
  void set_fullscreen(bool fullscreen = true);
  void set_minimized(bool minimized = true);
  
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  tab5_camera::Tab5Camera *camera_{nullptr};
  lv_obj_t *canvas_obj_{nullptr};
  std::string canvas_id_{};
  uint32_t update_interval_{100};  // 100ms = 10 FPS
  uint32_t last_update_{0};
  
  // Variables pour contrôle des logs
  uint32_t frame_count_{0};
  bool first_update_{true};
  bool canvas_warning_shown_{false};
  
  // Variables pour gestion du resize
  bool is_fullscreen_{false};
  bool is_minimized_{true};
  
  struct CanvasSize {
    uint16_t width;
    uint16_t height;
    int16_t x;
    int16_t y;
    uint16_t radius;
  };
  
  CanvasSize fullscreen_size_{1280, 720, 0, 0, 0};
  CanvasSize minimized_size_{640, 480, 141, 0, 12};
  CanvasSize current_size_{640, 480, 141, 0, 12};
  
  void update_canvas_();
  void apply_canvas_size_(const CanvasSize &size);
};

}  // namespace lvgl_camera_display
}  // namespace esphome
