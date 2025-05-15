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
    analogWriteFreq(5000);
    analogWriteRange(100);
    analogWrite(6, 100);
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
    backlight_init();
}

void lvgl_task_handler()
{
  lv_task_handler();
  handle_encoder_parameters_edit();
}