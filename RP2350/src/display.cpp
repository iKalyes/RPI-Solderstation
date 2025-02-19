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
    pinMode(6, OUTPUT);
    analogWriteFreq(1000);
    analogWriteRange(100);
    if(brightness == 0)
    {
      analogWrite(6, 50);
    }
    else
    {
      analogWrite(6, brightness);
    }
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

    if(buzzer_state == 1 || buzzer_state > 1){
        buzzer_state = 1;
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
    }
    else
    {
      lv_label_set_text_fmt(ui_TargetTemp, "%d", SetTemp);
    }
}