#include <ui-setting.h>

void ui_event_SystemSettingBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_MainScreen_screen_init);
}
}

void ui_event_PIDCalibration( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_screen_change( &ui_PIDCalibrationScreen, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_PIDCalibrationScreen_screen_init);
}
}

void ui_event_CustomCurve( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_screen_change( &ui_CustomCurveScreen, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_CustomCurveScreen_screen_init);
}
}

uint16_t temp_limited;

void ui_event_SliderTempLimited( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTempLimited, target, "", "℃");
      temp_limited = lv_slider_get_value(target);
}
}

uint16_t sleep_time;

void ui_event_SliderSleepTime( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextSleepTime, target, "", "Min");
      sleep_time = lv_slider_get_value(target) * 60;
}
}

uint8_t brightness;

void ui_event_SliderBrightness( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextBrightness, target, "", "%");
      brightness = lv_slider_get_value(target);
      analogWrite(6, brightness);
}
}

void ui_event_SaveConfig( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SaveConfig( e );
      Serial.println(buzzer_state);
      Serial.println(temp_limited);
      Serial.println(sleep_time);
      Serial.println(brightness);
}
}