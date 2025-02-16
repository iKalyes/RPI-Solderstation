#include <uicontrol.h>

void ui_event_TempSet( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_TemperatureSetting, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_TemperatureSetting_screen_init);
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

void ui_event_USER1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_TargetTemp, _UI_LABEL_PROPERTY_TEXT, "100");
}
}

void ui_event_USER2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_TargetTemp, _UI_LABEL_PROPERTY_TEXT, "200");
}
}

void ui_event_USER3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_TargetTemp, _UI_LABEL_PROPERTY_TEXT, "300");
}
}

void ui_event_USER4( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_PRESSED) {
      _ui_label_set_property(ui_TargetTemp, _UI_LABEL_PROPERTY_TEXT, "400");
}
}

void ui_event_Setting( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);

if ( event_code == LV_EVENT_RELEASED) {
      _ui_screen_change( &ui_SystemSetting, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_SystemSetting_screen_init);
}
}