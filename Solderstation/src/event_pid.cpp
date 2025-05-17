#include <event_pid.h>

void ui_event_PIDSettingBack( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_SystemSettingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_SystemSettingScreen_screen_init);
}
}

void ui_event_SolderingKP( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      SolderingKpUpdate( e );
}
}

void ui_event_SolderingKI( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      SolderingKiUpdate( e );
}
}

void ui_event_SolderingKD( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      SolderingKdUpdate( e );
}
}

void ui_event_SolderingKPDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKPDown( e );
        lv_spinbox_decrement(ui_SolderingKP);
}
}

void ui_event_SolderingKPUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKPUp( e );
        lv_spinbox_increment(ui_SolderingKP);
}
}

void ui_event_SolderingKIDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKIDown( e );
        lv_spinbox_decrement(ui_SolderingKI);
}
}

void ui_event_SolderingKIUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKIUp( e );
        lv_spinbox_increment(ui_SolderingKI);
}
}

void ui_event_SolderingKDDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKDDown( e );
        lv_spinbox_decrement(ui_SolderingKD);
}
}

void ui_event_SolderingKDUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      SolderingKDUp( e );
        lv_spinbox_increment(ui_SolderingKD);
}
}

void ui_event_HeatgunKP( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      HeatgunKpUpdate( e );
}
}

void ui_event_HeatgunKI( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      HeatgunKiUpdate( e );
}
}

void ui_event_HeatgunKD( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      HeatgunKdUpdate( e );
}
}

void ui_event_HeatgunKPDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKPDown( e );
        lv_spinbox_decrement(ui_HeatgunKP);
}
}

void ui_event_HeatgunKPUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKPUp( e );
        lv_spinbox_increment(ui_HeatgunKP);
}
}

void ui_event_HeatgunKIDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKIDown( e );
        lv_spinbox_decrement(ui_HeatgunKI);
}
}

void ui_event_HeatgunKIUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKIUp( e );
        lv_spinbox_increment(ui_HeatgunKI);
}
}

void ui_event_HeatgunKDDown( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKDDown( e );
        lv_spinbox_decrement(ui_HeatgunKD);
}
}

void ui_event_HeatgunKDUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      HeatgunKDUp( e );
        lv_spinbox_increment(ui_HeatgunKD);
}
}