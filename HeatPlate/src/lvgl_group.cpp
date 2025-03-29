#include <lvgl_group.h>

lv_group_t* group;

void lvgl_group_init()
{
    group = lv_group_create();
    lv_group_set_default(group);
    lv_indev_set_group(get_encoder_indev(), group);

    lv_group_add_obj(group, ui_StartStop);
    lv_group_add_obj(group, ui_Setting);
    lv_group_add_obj(group, ui_ScreenSwitch);
    lv_group_add_obj(group, ui_BuzzerSwitch);
    lv_group_add_obj(group, ui_FanSwitch);
    lv_group_add_obj(group, ui_TempSet);
}

void lvgl_group_to_setting()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_CustomCurve);
    lv_group_add_obj(group, ui_PIDSetting);
    lv_group_add_obj(group, ui_SliderTempLimited);
    lv_group_add_obj(group, ui_SliderSleepTime);
    lv_group_add_obj(group, ui_SliderBrightness);
    lv_group_add_obj(group, ui_SaveConfig);
    lv_group_add_obj(group, ui_SystemSettingBack);
}

void lvgl_group_to_curve()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_CustomCurveBack);

    lv_group_add_obj(group, ui_SliderProfileOne);
    lv_group_add_obj(group, ui_SliderTimeOne);
    lv_group_add_obj(group, ui_SliderProfileTwo);
    lv_group_add_obj(group, ui_SliderTimeTwo);
    lv_group_add_obj(group, ui_SliderProfileThree);
    lv_group_add_obj(group, ui_SliderTimeThree);
    lv_group_add_obj(group, ui_SliderProfileFour);
    lv_group_add_obj(group, ui_SliderTimeFour);
    lv_group_add_obj(group, ui_SliderProfileFive);
    lv_group_add_obj(group, ui_SliderTimeFive);

}

void lvgl_group_to_pid()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_PIDSettingBack);

    lv_group_add_obj(group, ui_KPSpinBox);
    lv_group_add_obj(group, ui_KISpinBox);
    lv_group_add_obj(group, ui_KDSpinBox);
}

void lvgl_group_to_tempset()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_TempSettingBack);

    lv_group_add_obj(group, ui_Num1);
    lv_group_add_obj(group, ui_Num2);
    lv_group_add_obj(group, ui_Num3);
    lv_group_add_obj(group, ui_Num4);
    lv_group_add_obj(group, ui_Num5);
    lv_group_add_obj(group, ui_Num6);
    lv_group_add_obj(group, ui_Num7);
    lv_group_add_obj(group, ui_Num8);
    lv_group_add_obj(group, ui_Num9);
    lv_group_add_obj(group, ui_Num0);

    lv_group_add_obj(group, ui_Confirm);
    lv_group_add_obj(group, ui_Delete);
}

void lvgl_group_to_chart()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_ChartScreenBack);

    lv_group_add_obj(group, ui_ChartTempSet);
    lv_group_add_obj(group, ui_ChartSwitch);
}

void lvgl_group_to_main()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_StartStop);
    lv_group_add_obj(group, ui_Setting);
    lv_group_add_obj(group, ui_ScreenSwitch);
    lv_group_add_obj(group, ui_BuzzerSwitch);
    lv_group_add_obj(group, ui_FanSwitch);
    lv_group_add_obj(group, ui_TempSet);
}