#include <lvgl_group.h>

lv_group_t* group;

void lvgl_group_init()
{
    group = lv_group_create();
    lv_group_set_default(group);
    lv_indev_set_group(get_encoder_indev(), group);

    lv_group_add_obj(group, ui_Buzzer);
    lv_group_add_obj(group, ui_CoolingFan);
    lv_group_add_obj(group, ui_Setting);

    lv_group_add_obj(group, ui_SolderingSwitch);
    lv_group_add_obj(group, ui_SolderingSet);
    lv_group_add_obj(group, ui_HeatgunSet);
    lv_group_add_obj(group, ui_HeatgunSwitch);

    lv_group_add_obj(group, ui_SolderingTargetTemp);
    lv_group_add_obj(group, ui_HeatgunTargetTemp);
    lv_group_add_obj(group, ui_HeatgunWindSpeed);
}

void lvgl_group_to_setting()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());

    lv_group_add_obj(group, ui_SystemSettingBack);

    lv_group_add_obj(group, ui_PIDSetting);
    lv_group_add_obj(group, ui_SolderingMaxTemp);
    lv_group_add_obj(group, ui_SolderingMinTemp);
    lv_group_add_obj(group, ui_SolderingStandbyTemp);
    lv_group_add_obj(group, ui_SolderingStandbyTime);

    lv_group_add_obj(group, ui_HeatgunMaxTemp);
    lv_group_add_obj(group, ui_HeatgunMinTemp);
    lv_group_add_obj(group, ui_SolderingTempAdjust);
    lv_group_add_obj(group, ui_HeatgunTempAdjust);

    lv_group_add_obj(group, ui_SystemBrightness);
    lv_group_add_obj(group, ui_SettingSave);
}

void lvgl_group_to_main()
{
    lv_group_remove_all_objs(group);
    
    // 等待输入设备释放后再添加对象到组
    lv_indev_wait_release(get_encoder_indev());
    
    lv_group_add_obj(group, ui_Buzzer);
    lv_group_add_obj(group, ui_CoolingFan);
    lv_group_add_obj(group, ui_Setting);

    lv_group_add_obj(group, ui_SolderingSwitch);
    lv_group_add_obj(group, ui_SolderingSet);
    lv_group_add_obj(group, ui_HeatgunSet);
    lv_group_add_obj(group, ui_HeatgunSwitch);

    lv_group_add_obj(group, ui_SolderingTargetTemp);
    lv_group_add_obj(group, ui_HeatgunTargetTemp);
    lv_group_add_obj(group, ui_HeatgunWindSpeed);
}