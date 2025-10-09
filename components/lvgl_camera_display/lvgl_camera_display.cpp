#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "🎥 Configuration LVGL Camera Display...");
  
  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "❌ Camera non configurée");
    this->mark_failed();
    return;
  }
  
  ESP_LOGI(TAG, "✅ LVGL Camera Display initialisé");
  ESP_LOGI(TAG, "   Update interval: %u ms (~%d FPS)", 
           this->update_interval_, 1000 / this->update_interval_);
  ESP_LOGI(TAG, "   Fullscreen: %ux%u", this->fullscreen_size_.width, this->fullscreen_size_.height);
  ESP_LOGI(TAG, "   Minimized: %ux%u", this->minimized_size_.width, this->minimized_size_.height);
}

void LVGLCameraDisplay::loop() {
  uint32_t now = millis();
  
  // Vérifier si c'est le moment de mettre à jour
  if (now - this->last_update_ < this->update_interval_) {
    return;
  }
  
  this->last_update_ = now;
  
  // Si la caméra est en streaming, capturer ET mettre à jour le canvas
  if (this->camera_->is_streaming()) {
    // Capturer une nouvelle frame
    bool frame_captured = this->camera_->capture_frame();
    
    if (frame_captured) {
      this->update_canvas_();
      
      // Compteur de frames pour debug RÉDUIT
      this->frame_count_++;
      
      // Logger seulement toutes les 100 frames
      if (this->frame_count_ % 100 == 0) {
        ESP_LOGD(TAG, "✓ %u frames affichées", this->frame_count_);
      }
    }
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  FPS cible: ~%d", 1000 / this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Canvas configuré: %s", this->canvas_obj_ ? "OUI" : "NON");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->is_fullscreen_ ? "FULLSCREEN" : (this->is_minimized_ ? "MINIMIZED" : "CUSTOM"));
}

void LVGLCameraDisplay::update_canvas_() {
  if (this->camera_ == nullptr) {
    return;
  }
  
  if (this->canvas_obj_ == nullptr) {
    if (!this->canvas_warning_shown_) {
      ESP_LOGW(TAG, "❌ Canvas null - pas encore configuré?");
      this->canvas_warning_shown_ = true;
    }
    return;
  }
  
  // Obtenir les données de l'image (toujours 1280x720)
  uint8_t* img_data = this->camera_->get_image_data();
  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();
  
  if (img_data == nullptr) {
    return;
  }
  
  // Debug: vérifier les premières valeurs SEULEMENT UNE FOIS
  if (this->first_update_) {
    ESP_LOGI(TAG, "🖼️  Premier update canvas:");
    ESP_LOGI(TAG, "   Dimensions caméra: %ux%u", width, height);
    ESP_LOGI(TAG, "   Canvas size: %ux%u", this->current_size_.width, this->current_size_.height);
    ESP_LOGI(TAG, "   Buffer: %p", img_data);
    ESP_LOGI(TAG, "   Premiers pixels (RGB565): %02X%02X %02X%02X %02X%02X", 
             img_data[0], img_data[1], img_data[2], img_data[3], img_data[4], img_data[5]);
    this->first_update_ = false;
  }
  
  // Mettre à jour le buffer du canvas avec les données RGB565
  // LVGL va automatiquement redimensionner si la taille du canvas est différente
  lv_canvas_set_buffer(this->canvas_obj_, img_data, 
                      this->current_size_.width, 
                      this->current_size_.height, 
                      LV_IMG_CF_TRUE_COLOR);
  
  // Invalider le canvas pour forcer le redessinage
  lv_obj_invalidate(this->canvas_obj_);
}

void LVGLCameraDisplay::configure_canvas(lv_obj_t *canvas) { 
  this->canvas_obj_ = canvas;
  ESP_LOGI(TAG, "🎨 Canvas configuré: %p", canvas);
  
  if (canvas != nullptr) {
    // Vérifier les propriétés du canvas
    lv_coord_t w = lv_obj_get_width(canvas);
    lv_coord_t h = lv_obj_get_height(canvas);
    ESP_LOGI(TAG, "   Taille initiale canvas: %dx%d", w, h);
    
    // Appliquer la taille par défaut (minimized)
    this->apply_canvas_size_(this->minimized_size_);
  }
}

void LVGLCameraDisplay::apply_canvas_size_(const CanvasSize &size) {
  if (this->canvas_obj_ == nullptr) {
    ESP_LOGW(TAG, "Cannot apply size: canvas is null");
    return;
  }
  
  // Mettre à jour la taille
  lv_obj_set_size(this->canvas_obj_, size.width, size.height);
  
  // Mettre à jour la position
  lv_obj_set_pos(this->canvas_obj_, size.x, size.y);
  
  // Mettre à jour le radius
  lv_obj_set_style_radius(this->canvas_obj_, size.radius, 0);
  
  // Sauvegarder la taille actuelle
  this->current_size_ = size;
  
  ESP_LOGI(TAG, "📐 Canvas resized: %ux%u @ (%d,%d) radius=%u", 
           size.width, size.height, size.x, size.y, size.radius);
}

void LVGLCameraDisplay::resize_canvas(uint16_t width, uint16_t height, int16_t x, int16_t y, uint16_t radius) {
  CanvasSize custom_size = {width, height, x, y, radius};
  this->apply_canvas_size_(custom_size);
  this->is_fullscreen_ = false;
  this->is_minimized_ = false;
}

void LVGLCameraDisplay::set_fullscreen(bool fullscreen) {
  if (fullscreen) {
    this->apply_canvas_size_(this->fullscreen_size_);
    this->is_fullscreen_ = true;
    this->is_minimized_ = false;
    ESP_LOGI(TAG, "📺 Mode: FULLSCREEN (1280x720)");
  } else {
    this->apply_canvas_size_(this->minimized_size_);
    this->is_fullscreen_ = false;
    this->is_minimized_ = true;
    ESP_LOGI(TAG, "📺 Mode: MINIMIZED (760x440)");
  }
}

void LVGLCameraDisplay::set_minimized(bool minimized) {
  this->set_fullscreen(!minimized);
}

}  // namespace lvgl_camera_display
}  // namespace esphome
