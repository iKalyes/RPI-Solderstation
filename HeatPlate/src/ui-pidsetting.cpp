#include <ui-pidsetting.h>

void ui_event_PIDSettingBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_SystemSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_SystemSettingScreen_screen_init);
}
}

void ui_event_KPDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKPMinus( e );
      lv_spinbox_decrement( ui_KPSPinBox );
}
}

void ui_event_KPUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKPPlus( e );
      lv_spinbox_increment( ui_KPSPinBox );
}
}

void ui_event_KIDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKIMinus( e );
      lv_spinbox_decrement( ui_KISpinBox );
}
}

void ui_event_KIUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKIPlus( e );
      lv_spinbox_increment( ui_KISpinBox );
}
}

void ui_event_KDDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKDMinus( e );
      lv_spinbox_decrement( ui_KDSpinBox );
}
}

void ui_event_KDUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKDPlus( e );
      lv_spinbox_increment( ui_KDSpinBox );
}
}