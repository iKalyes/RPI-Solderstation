#include <ui-chart.h>

void ui_event_ChartScreenBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_screen_change( &ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SystemSettingScreen_screen_init);
      lv_timer_pause(chart_update_timer);
      lvgl_group_to_main();
}
}

void ui_event_ChartTempSet( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      tempset_status = 2;
      _ui_screen_change( &ui_TemperatureSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_TemperatureSettingScreen_screen_init);
      lvgl_group_to_tempset();
}
}

void ui_event_ChartSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      ChartHeaterStart( e );
      lv_obj_add_state(ui_StartStop, LV_STATE_CHECKED);
      clock_status = 1;
      heating_status = 1;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      ChartHeaterStop( e );
      lv_obj_clear_state(ui_StartStop, LV_STATE_CHECKED);
      clock_status = 0;
      heater_stop();
}
}
