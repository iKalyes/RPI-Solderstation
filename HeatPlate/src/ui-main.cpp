#include <ui-main.h>

void ui_event_TempSet( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      tempset_status = 1;
      _ui_screen_change( &ui_TemperatureSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_TemperatureSettingScreen_screen_init);
}
}

void ui_event_Setting( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_SystemSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SystemSettingScreen_screen_init);
}
}

void ui_event_FanSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      FanTurnON( e );
      fan_on();
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      FanTurnOFF( e );
      fan_off();
}
}

void ui_event_BuzzerSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      lv_img_set_src(ui_BuzzerStatus, &ui_img_185202102);
      lv_img_set_src(ui_ChartBuzzerStatus, &ui_img_185202102);
      buzzer_status = 1;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      lv_img_set_src(ui_BuzzerStatus, &ui_img_1699618864);
      lv_img_set_src(ui_ChartBuzzerStatus, &ui_img_1699618864);
      buzzer_status = 0;
}
}

void ui_event_StartStop( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      HeatingStart( e );
      lv_obj_add_state(ui_ChartSwitch, LV_STATE_CHECKED);
      clock_status = 1;
      heating_status = 1;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      HeatingStop( e );
      lv_obj_clear_state(ui_ChartSwitch, LV_STATE_CHECKED);
      clock_status = 0;
      heater_stop();
}
}

void ui_event_ScreenSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_screen_change( &ui_ChartScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_ChartScreen_screen_init);
      lv_timer_resume(chart_update_timer);
}
}