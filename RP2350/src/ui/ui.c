// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: UI

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_MainScreen
void ui_MainScreen_screen_init(void);
lv_obj_t * ui_MainScreen;
lv_obj_t * ui_StatusDock;
lv_obj_t * ui_Temp;
lv_obj_t * ui_TempC;
lv_obj_t * ui_TempPic;
lv_obj_t * ui_BuzzerStatus;
lv_obj_t * ui_HeatDock;
lv_obj_t * ui_HeatPic;
lv_obj_t * ui_HeatTemp;
lv_obj_t * ui_HeatTempC;
lv_obj_t * ui_Target;
lv_obj_t * ui_TargetTemp;
lv_obj_t * ui_TempSet;
lv_obj_t * ui_FAN;
lv_obj_t * ui_FANPic;
lv_obj_t * ui_FanPercent;
lv_obj_t * ui_Percent;
lv_obj_t * ui_FanUP;
lv_obj_t * ui_FanDown;
lv_obj_t * ui_USER1;
lv_obj_t * ui_USER2;
lv_obj_t * ui_USER3;
lv_obj_t * ui_USER4;
lv_obj_t * ui_StartStop;
lv_obj_t * ui_USER6;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_MainScreen_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_MainScreen);
}