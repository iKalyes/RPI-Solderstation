#include <ui-pidcalibration.h>

void ui_event_PIDCalibrationBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_screen_change( &ui_SystemSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SystemSettingScreen_screen_init);
}
}

void ui_event_PIDTempSet( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      tempset_status = 2;
      _ui_screen_change( &ui_TemperatureSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_TemperatureSettingScreen_screen_init);
}
}

void ui_event_PIDCalibrationSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      PIDCalibrationStart( e );
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      PIDCalibrationStop( e );
}
}
