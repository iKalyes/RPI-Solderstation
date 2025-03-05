/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
#include <main.h>

const uint8_t relayPin = 16;

// user settings
uint32_t settleTimeSec = 5;   //稳定时间
uint32_t testTimeSec = 100;   //（采样间隔）sample interval = testTimeSec / samples
const uint16_t samples = 500; //采样数量
const float inputSpan = 450;  //温度输入范围
const float outputSpan = 100; //控制输出范围（暂无用）
float outputStart = 0;        //稳定时间输出值
float outputStep = 50;        //采样阶段输出值
float tempLimit = 400;        //温度上限

// variables
float Setpoint = 150;         //设定温度
float Input, Output, Kp, Ki, Kd;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
QuickPID myPID(&Input, &Output, &Setpoint);

void vApplicationTickHook()
{
  encoder_tick();
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    ReadFlash();
    backlight_init();
    display_init();
    update_chart_init();
}

void loop()
{
    lvgl_tmp102_refresh();
    lvgl_max6675_refresh();
    lvgl_clock_refresh();
    lvgl_run();
}

void setup1()
{
  TMP102_init();
  MAX6675_init();
  xTaskCreate(clock_run, "clock_run", 128, NULL, 4, NULL);
  xTaskCreate(TMP102_Read, "TMP102_Read", 128, NULL, 3, NULL);
  xTaskCreate(MAX6675_Read, "MAX6675_Read", 128, NULL, 5, NULL);

  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
  tuner.initHardwarePwm(relayPin);
}

void loop1()
{
  if(heating_status == 1)
  {
  float optimumOutput = tuner.hardwarePwm(relayPin, Input, Output, Setpoint);

  switch (tuner.Run()) {

    case tuner.sample: // active once per sample during test
      Input = heater_temperature;
      tuner.plotter(Input, Output, Setpoint, 1.0f, 1); // output scale 0.5, plot every 3rd sample
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.EnablePredictControl(true, 2.0, 5.0); // 启用预测控制
      myPID.SetSampleTimeUs(testTimeSec / samples);
      myPID.SetMode(QuickPID::Control::automatic);
      myPID.SetProportionalMode(QuickPID::pMode::pOnMeas);
      myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
      myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      Input = heater_temperature;
      myPID.Compute();
      tuner.plotter(Input, optimumOutput, Setpoint, 1.0f, 1);
      break;
   }
  }
  else
  {
    tuner.Reset(relayPin);
  }
}