#include <display.h>

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
FT6336U touch_6336;

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{

  if (touch_6336.available())
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touch_6336.touchPoint.tp[0].x;
    data->point.y = touch_6336.touchPoint.tp[0].y;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }

}

void backlight_init()
{
  // 第一个 PWM 引脚设置 (例如 GPIO 6)
  gpio_set_function(6, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(6);
  uint channel = pwm_gpio_to_channel(6);
  
  // 为第一个引脚设置时钟分频和计数范围（决定频率）
  pwm_set_clkdiv(slice_num, 230.0);  // 分频器
  pwm_set_wrap(slice_num, 1000);     // 最大计数值 (分辨率)
    if(brightness == 0)
    {
      pwm_set_chan_level(slice_num, channel, 500);  // 占空比
    }
    else
    {
      pwm_set_chan_level(slice_num, channel, brightness * 10);  // 占空比
    } 
  pwm_set_enabled(slice_num, true);
}

void backlight_refresh()
{
  uint slice_num = pwm_gpio_to_slice_num(6);
  uint channel = pwm_gpio_to_channel(6);
  pwm_set_chan_level(slice_num, channel, brightness * 10);  // 占空比
}

void display_init()
{
    lv_init();
    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    touch_6336.begin(Wire);
    lv_disp_draw_buf_init( &draw_buf, buf_1, buf_2, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );
  
    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    if(buzzer_status == 1 || buzzer_status > 1){
        buzzer_status = 1;
        lv_obj_add_state(ui_BuzzerSwitch, LV_STATE_CHECKED);
        lv_img_set_src(ui_BuzzerStatus, &ui_img_185202102);
    }
    else{
        lv_obj_clear_state(ui_BuzzerSwitch, LV_STATE_CHECKED);
        lv_img_set_src(ui_BuzzerStatus, &ui_img_1699618864);

    }

    if(brightness == 0 || brightness > 100)
    {
      brightness = 50;
    }
    else
    {
      lv_slider_set_value(ui_SliderBrightness, brightness, LV_ANIM_OFF);
      lv_label_set_text_fmt(ui_TextBrightness, "%d%%", brightness);
    }

    if(temp_limited == 0 || temp_limited > 400)
    {
      temp_limited = 400;
    }
    else
    {
      lv_slider_set_value(ui_SliderTempLimited, temp_limited, LV_ANIM_OFF);
      lv_label_set_text_fmt(ui_TextTempLimited, "%d℃", temp_limited);
    }

    if(sleep_time == 0 || sleep_time > 60*60)
    {
      sleep_time = 10*60;
    }
    else
    {
      lv_slider_set_value(ui_SliderSleepTime, sleep_time/60, LV_ANIM_OFF);
      lv_label_set_text_fmt(ui_TextSleepTime, "%dMin", sleep_time/60);
    }

    if(SetTemp == 0 || SetTemp > 400)
    {
      lv_label_set_text(ui_TargetTemp, "000");
      lv_bar_set_range(ui_BarHeaterTemp, 0, 100);
    }
    else if(SetTemp > temp_limited)
    {
      SetTemp = temp_limited;
      lv_label_set_text_fmt(ui_TargetTemp, "%d", temp_limited);
      lv_bar_set_range(ui_BarHeaterTemp, 0, temp_limited);
    }
    else
    {
      lv_label_set_text_fmt(ui_TargetTemp, "%d", SetTemp);
      lv_bar_set_range(ui_BarHeaterTemp, 0, SetTemp);
    }

    if(stage1_temp == 0 || stage1_temp > 400)
    {
      stage1_temp = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextProfileOne, "%d℃", stage1_temp);
      lv_slider_set_value(ui_SliderProfileOne, stage1_temp, LV_ANIM_OFF);
    }
    if(stage1_time == 0 || stage1_time > 300)
    {
      stage1_time = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextTimeOne, "%dS", stage1_time);
      lv_slider_set_value(ui_SliderTimeOne, stage1_time, LV_ANIM_OFF);
    }

    if(stage2_temp == 0 || stage2_temp > 400)
    {
      stage2_temp = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextProfileTwo, "%d℃", stage2_temp);
      lv_slider_set_value(ui_SliderProfileTwo, stage2_temp, LV_ANIM_OFF);
    }
    if(stage2_time == 0 || stage2_time > 300)
    {
      stage2_time = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextTimeTwo, "%dS", stage2_time);
      lv_slider_set_value(ui_SliderTimeTwo, stage2_time, LV_ANIM_OFF);
    }

    if(stage3_temp == 0 || stage3_temp > 400)
    {
      stage3_temp = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextProfileThree, "%d℃", stage3_temp);
      lv_slider_set_value(ui_SliderProfileThree, stage3_temp, LV_ANIM_OFF);
    }
    if(stage3_time == 0 || stage3_time > 300)
    {
      stage3_time = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextTimeThree, "%dS", stage3_time);
      lv_slider_set_value(ui_SliderTimeThree, stage3_time, LV_ANIM_OFF);
    }

    if(stage4_temp == 0 || stage4_temp > 400)
    {
      stage4_temp = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextProfileFour, "%d℃", stage4_temp);
      lv_slider_set_value(ui_SliderProfileFour, stage4_temp, LV_ANIM_OFF);
    }
    if(stage4_time == 0 || stage4_time > 300)
    {
      stage4_time = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextTimeFour, "%dS", stage4_time);
      lv_slider_set_value(ui_SliderTimeFour, stage4_time, LV_ANIM_OFF);
    }

    if(stage5_temp == 0 || stage5_temp > 400)
    {
      stage5_temp = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextProfileFive, "%d℃", stage5_temp);
      lv_slider_set_value(ui_SliderProfileFive, stage5_temp, LV_ANIM_OFF);
    }
    if(stage5_time == 0 || stage5_time > 300)
    {
      stage5_time = 100;
    }
    else
    {
      lv_label_set_text_fmt(ui_TextTimeFive, "%dS", stage5_time);
      lv_slider_set_value(ui_SliderTimeFive, stage5_time, LV_ANIM_OFF);
    }
}

void lvgl_run()
{
    lv_task_handler();
    delay(5);
}

void lvgl_tmp102_refresh()
{
  if(room_temperature > 100)
  {
    lv_label_set_text(ui_RoomTemp, "ERROR");
  }
  else
  {
    char temp_str[8] = {0};  // 缓冲区足够存储 XX.XX 格式
    snprintf(temp_str, sizeof(temp_str), "%.2f", room_temperature);
    lv_label_set_text(ui_RoomTemp, temp_str);
  }
}

void lvgl_max6675_refresh()
{
    if(heater_status == 0)
    {
        // 提取整数部分
        int temp_int = (int)heater_temperature;
        
        char temp_str[4] = {0};
        if(temp_int < 100) {
            // 小于100时补零，确保显示3位
            snprintf(temp_str, sizeof(temp_str), "%03d", temp_int);
        } else {
            // 大于等于100时正常显示
            snprintf(temp_str, sizeof(temp_str), "%d", temp_int);
        }
        lv_label_set_text(ui_HeaterTemp, temp_str);
        lv_label_set_text(ui_ChartTemp, temp_str);
        lv_label_set_text(ui_CurrentTemp, temp_str);
        lv_bar_set_value(ui_BarHeaterTemp, heater_temperature, LV_ANIM_ON);
    }
    else
    {
        lv_label_set_text(ui_HeaterTemp, "ERR");
        lv_label_set_text(ui_ChartTemp, "ERR");
        lv_label_set_text(ui_CurrentTemp, "ERR");
        lv_bar_set_value(ui_BarHeaterTemp, 0, LV_ANIM_ON);
    }
}

void lvgl_clock_refresh()
{
    if(clock_status == 1)
    {
        char second_str[3] = {0};
        char minute_str[3] = {0};
        if(timer_second < 10)
        {
            snprintf(second_str, sizeof(second_str), "0%d", timer_second);
        }
        else
        {
            snprintf(second_str, sizeof(second_str), "%d", timer_second);
        }
        lv_label_set_text(ui_Second, second_str);
        lv_label_set_text(ui_ChartSecond, second_str);
        if(timer_minute < 10)
        {
            snprintf(minute_str, sizeof(minute_str), "0%d", timer_minute);
        }
        else
        {
            snprintf(minute_str, sizeof(minute_str), "%d", timer_minute);
        }
        lv_label_set_text(ui_Minute, minute_str);
        lv_label_set_text(ui_ChartMinute, minute_str);
    }
    else
    {
        lv_label_set_text(ui_Second, "00");
        lv_label_set_text(ui_Minute, "00");
        lv_label_set_text(ui_ChartSecond, "00");
        lv_label_set_text(ui_ChartMinute, "00");
    }
}

void update_chart_init()
{
      ui_TempChart_TempSeries = lv_chart_add_series(ui_TempChart, lv_color_hex(0XFF0000), LV_CHART_AXIS_PRIMARY_Y);
      ui_TempChart_DutySeries = lv_chart_add_series(ui_TempChart, lv_color_hex(0X00FFFF), LV_CHART_AXIS_SECONDARY_Y);
      lv_chart_set_point_count(ui_TempChart, 128);
      chart_update_timer = lv_timer_create(update_chart_data, 200, NULL);
      lv_timer_pause(chart_update_timer);
}

void update_chart_data(lv_timer_t * timer)
{
  if(heater_status == 0)
  {
      lv_chart_set_next_value(ui_TempChart, ui_TempChart_TempSeries, heater_temperature);
  }
  else
  {
      lv_chart_set_next_value(ui_TempChart, ui_TempChart_TempSeries, 0);
  }
}