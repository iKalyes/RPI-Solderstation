#include <ui-customcurve.h>

uint16_t stage1_temp;
uint16_t stage1_time;
uint16_t stage2_temp;
uint16_t stage2_time;
uint16_t stage3_temp;
uint16_t stage3_time;
uint16_t stage4_temp;
uint16_t stage4_time;
uint16_t stage5_temp;
uint16_t stage5_time;

void ui_event_CustomCurveBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_SystemSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SystemSettingScreen_screen_init);
}
}

void ui_event_SliderProfileOne( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextProfileOne, target, "", "℃");
      stage1_temp = lv_slider_get_value(target);
}
}

void ui_event_SliderTimeOne( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTimeOne, target, "", "S");
      stage1_time = lv_slider_get_value(target);
}
}

void ui_event_SliderProfileTwo( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextProfileTwo, target, "", "℃");
      stage2_temp = lv_slider_get_value(target);
}
}

void ui_event_SliderTimeTwo( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTimeTwo, target, "", "S");
      stage2_time = lv_slider_get_value(target);
}
}

void ui_event_SliderProfileThree( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextProfileThree, target, "", "℃");
      stage3_temp = lv_slider_get_value(target);
}
}

void ui_event_SliderTimeThree( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTimeThree, target, "", "S");
      stage3_time = lv_slider_get_value(target);
}
}

void ui_event_SliderProfileFour( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextProfileFour, target, "", "℃");
      stage4_temp = lv_slider_get_value(target);
}
}

void ui_event_SliderTimeFour( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTimeFour, target, "", "S");
      stage4_time = lv_slider_get_value(target);
}
}

void ui_event_SliderProfileFive( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextProfileFive, target, "", "℃");
      stage5_temp = lv_slider_get_value(target);
}
}

void ui_event_SliderTimeFive( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_TextTimeFive, target, "", "S");
      stage5_time = lv_slider_get_value(target);
}
}