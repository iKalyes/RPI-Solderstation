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
      lv_spinbox_decrement( ui_KPSpinBox );
}
}

void ui_event_KPUp( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      PIDKPPlus( e );
      lv_spinbox_increment( ui_KPSpinBox );
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

void ui_event_KPSpinBox( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      KpUpdate( e );
      pid_update();
}
}

void ui_event_KISpinBox( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      KiUpdate( e );
      pid_update();
}
}

void ui_event_KDSpinBox( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_VALUE_CHANGED) {
      KdUpdate( e );
      pid_update();
}
}

QuickPID PID;

void pid_setting()
{
    // 转换为整数 (乘以1000)
    uint32_t Kp = (uint32_t)(all_Kp * 1000.0f + 0.5f); 
    uint32_t Ki = (uint32_t)(all_Ki * 1000.0f + 0.5f); 
    uint32_t Kd = (uint32_t)(all_Kd * 1000.0f + 0.5f); 

    lv_spinbox_set_value(ui_KPSpinBox, Kp);
    lv_spinbox_set_value(ui_KISpinBox, Kp);
    lv_spinbox_set_value(ui_KDSpinBox, Kp);
}

void pid_update()
{
    float Kp = lv_spinbox_get_value(ui_KPSpinBox) / 1000.0f;
    float Ki = lv_spinbox_get_value(ui_KISpinBox) / 1000.0f;
    float Kd = lv_spinbox_get_value(ui_KDSpinBox) / 1000.0f;
    PID.SetTunings(Kp, Ki, Kd);
}