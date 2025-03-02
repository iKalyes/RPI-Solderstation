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

uint8_t FanPercent = 0;
char fanPercentStr[4];  // 3位数字加终止符需要4字节

void ui_event_FanUP(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_RELEASED) {
        if (FanPercent <= 90) {
            FanPercent += 10;
        } else {
            FanPercent = 100;
        }
        sprintf(fanPercentStr, "%03d", FanPercent);  // 格式化为3位数，不足补0
        lv_label_set_text(ui_FanPercent, fanPercentStr);
    }
}

void ui_event_FanDown(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_RELEASED) {
        if (FanPercent >= 10) {
            FanPercent -= 10;
        } else {
            FanPercent = 0;
        }
        sprintf(fanPercentStr, "%03d", FanPercent);  // 格式化为3位数，不足补0
        lv_label_set_text(ui_FanPercent, fanPercentStr);
    }
}

void ui_event_FanSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      FanPercent = 100;
      _ui_label_set_property(ui_FanPercent, _UI_LABEL_PROPERTY_TEXT, "100");
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      FanPercent = 0;
      _ui_label_set_property(ui_FanPercent, _UI_LABEL_PROPERTY_TEXT, "000");
}
}

void ui_event_USER1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_HeatingMode, _UI_LABEL_PROPERTY_TEXT, "Thermostatic");
}
}

void ui_event_USER2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_HeatingMode, _UI_LABEL_PROPERTY_TEXT, "RSS Curve");
}
}

void ui_event_USER3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_HeatingMode, _UI_LABEL_PROPERTY_TEXT, "RTS Curve");
}
}

void ui_event_USER4( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_HeatingMode, _UI_LABEL_PROPERTY_TEXT, "Custom Curve");
}
}

void ui_event_BuzzerSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      lv_img_set_src(ui_BuzzerStatus, &ui_img_185202102);
      buzzer_status = 1;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      lv_img_set_src(ui_BuzzerStatus, &ui_img_1699618864);
      buzzer_status = 0;
}
}

void ui_event_StartStop( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);

if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      HeatingStart( e );
      clock_status = 1;
      heating_status = 1;
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      HeatingStop( e );
      clock_status = 0;
      heating_status = 0;
}
}