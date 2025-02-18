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

void ui_event_SliderTempLimited( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTempLimited, target, "", "℃");
}
}

void ui_event_SliderSleepTime( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextSleepTime, target, "", "Min");
}
}

void ui_event_SliderBrightness( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextBrightness, target, "", "%");
      int8_t brightness = lv_slider_get_value(target);
      analogWrite(6, brightness);
}
}