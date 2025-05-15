#include <lvgl_group.h>

lv_group_t* group;

void lvgl_group_init()
{
    group = lv_group_create();
    lv_group_set_default(group);
    lv_indev_set_group(get_encoder_indev(), group);
    lv_group_add_obj(group, ui_SplderingSwitch);
    lv_group_add_obj(group, ui_SolderingSet);
    lv_group_add_obj(group, ui_HeatgunSet);
    lv_group_add_obj(group, ui_HeatgunSwitch);
    lv_group_add_obj(group, ui_SolderingTargetTemp);
    lv_group_add_obj(group, ui_HeatgunTargetTemp);
    lv_group_add_obj(group, ui_HeatgunWindSpeed);
}

