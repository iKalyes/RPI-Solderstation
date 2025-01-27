/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */

#include <TFT_eSPI.h>

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

#include <SparkFunTMP102.h>
#include "ui/ui.h"
#include <RAK14014_FT6336U.h>
#include <MAX6675.h>
#include <RotaryEncoder.h>

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

FT6336U touch_6336;
TMP102 sensor0;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf_1[ screenWidth * screenHeight / 10 ];
static lv_color_t buf_2[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif


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

    pinMode(20, OUTPUT);
    digitalWrite(20, HIGH);
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);

    touch_6336.begin(Wire);
    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/

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

    /* Try an example. See all the examples 
     * online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples */
     //lv_example_btn_1();
   
     /*Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMOS_WIDGETS*/
     ui_init();
    // lv_demo_widgets();               
    // lv_demo_benchmark();          
    // lv_demo_keypad_encoder();     
    // lv_demo_music();              
    // lv_demo_printer();
    // lv_demo_stress();
    
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    vTaskDelay(5);
}

void TMP102_USE(void *param)
{
  (void)param;
    Wire1.setSDA(18);
    Wire1.setSCL(19);
    Wire1.begin();
    
    sensor0.begin();

    // set the number of consecutive faults before triggering alarm.
    // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
    sensor0.setFault(0);  // Trigger alarm immediately
    // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
    sensor0.setAlertPolarity(1); // Active HIGH
    // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
    sensor0.setAlertMode(0); // Comparator Mode.
    // set the Conversion Rate (how quickly the sensor gets a new reading)
    //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
    sensor0.setConversionRate(2);
    //set Extended Mode.
    //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
    sensor0.setExtendedMode(0);
    //set T_HIGH, the upper limit to trigger the alert on
    sensor0.setHighTempC(29.4); // set T_HIGH in C
    //set T_LOW, the lower limit to shut turn off the alert
    sensor0.setLowTempC(26.67); // set T_LOW in C
    sensor0.wakeup();
    while(true)
    {
      float temperature;
      temperature = sensor0.readTempC();
      Serial.print("Temperature: ");
      Serial.println(temperature);
      delay(250);
    }
}

MAX6675 sensor1(24, 25, 26, &SPI1, 1000000);

RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

void setup1()
{
  //sensor1.begin();
  //xTaskCreate(TMP102_USE, "TMP102_USE", 128, NULL, 1, NULL);
  encoder = new RotaryEncoder(14, 15, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(14), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), checkPosition, CHANGE);

}

void loop1()
{
  //int status = sensor1.read();
  //float temp = sensor1.getTemperature();

  //Serial.print("\tstatus: ");
  //Serial.print(status);
  //Serial.print("\ttemp: ");
  //Serial.println(temp);

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

  //delay(250);
}