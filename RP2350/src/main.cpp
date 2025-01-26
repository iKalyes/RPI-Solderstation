/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include <TFT_eSPI.h>
#include <lvgl.h>

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

#include "ui/ui.h"
#include <RAK14014_FT6336U.h>
#include <RotaryEncoder.h>

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

FT6336U touch_6336;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf_1[ screenWidth * screenHeight / 10 ];
static lv_color_t buf_2[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

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

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
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
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(5);
}


RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup1()
{

  encoder = new RotaryEncoder(16, 17, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(16), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), checkPosition, CHANGE);

}

void loop1()
{
  static int pos = 0;

  encoder->tick(); // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  }
  }