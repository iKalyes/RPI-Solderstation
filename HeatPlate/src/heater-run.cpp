#include <heater-run.h>

const uint8_t relayPin = 16;
const uint8_t fanPin = 20;

// user settings
uint32_t settleTimeSec = 20;  //稳定时间
uint32_t testTimeSec = 100;   //（采样间隔）sample interval = testTimeSec / samples
const uint16_t samples = 500; //采样数量
const float inputSpan = 450;  //温度输入范围
const float outputSpan = 100; //控制输出范围（暂无用）
float outputStart = 0;        //稳定时间输出值
float outputStep = 50;        //采样阶段输出值
float tempLimit;              //温度上限
bool PID_flag = false;      //PID标志位

// variables
float Setpoint;         
float Input, Output, Kp, Ki, Kd;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
QuickPID myPID(&Input, &Output, &Setpoint);

void heater_tempset()
{
    Setpoint = SetTemp;
}

void heater_templimit()
{
    tempLimit = temp_limited;
}

void heater_init()
{
  tempLimit = temp_limited;
  Setpoint = SetTemp;
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
  tuner.initHardwarePwm(relayPin);
  if(all_Kp != 0 || all_Ki != 0 || all_Kd != 0)
  {
    myPID.SetOutputLimits(0, 100); // 限制输出范围
    myPID.SetSampleTimeUs(testTimeSec / samples);
    myPID.SetMode(QuickPID::Control::automatic);
    myPID.SetProportionalMode(QuickPID::pMode::pOnMeas);
    myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
    myPID.SetTunings(all_Kp, all_Ki, all_Kd); // update PID with the new tunings
    pid_setting();
    tuner.SetStausRunPid();
    PID_flag = true;
  }
  else
  {
    PID_flag = false;
  }
  
}

void PID_flag_True()
{
  PID_flag = true;
}

void PID_flag_False()
{
  PID_flag = false;
}

void heater_run()
{
  if(heating_status == 1)
  {
  float optimumOutput = tuner.hardwarePwm(relayPin, Input, Output, Setpoint);

  if(PID_flag == false)
  {
    switch (tuner.Run()) {

      case tuner.sample: // active once per sample during test
        Input = heater_temperature;
        tuner.plotter(Input, Output, Setpoint, 1.0f, 1); // output scale 0.5, plot every 3rd sample
        heater_duty = Output;
        break;
  
      case tuner.tunings: // active just once when sTune is done
        tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
        myPID.EnablePredictControl(true, 2.0, 6.0); // 启用预测控制
        myPID.SetSampleTimeUs(testTimeSec / samples);
        myPID.SetMode(QuickPID::Control::automatic);
        myPID.SetProportionalMode(QuickPID::pMode::pOnMeas);
        myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
        myPID.SetTunings(Kp, Ki, Kd); // update PID with the new tunings
        all_Kp = Kp;
        all_Ki = Ki;
        all_Kd = Kd;
        pid_setting();
        WritePID();
        PID_flag = true;
        break;
     }
  }
  else
  {
    switch (tuner.Run()) {
      case tuner.runPid: // active once per sample during test
        Input = heater_temperature;
        myPID.Compute();
        heater_duty = optimumOutput;
        tuner.plotter(Input, optimumOutput, Setpoint, 1.0f, 1);
        break;
    }
  }

  }
  else
  {
    heater_duty = 0;
    tuner.StopPwm(relayPin);
  }
}

void heater_stop()
{
  heating_status = 0;
  if(PID_flag == true)
  {
    tuner.Reset(relayPin);
    tuner.SetStausRunPid();
  }
  else
  {
    tuner.Reset(relayPin);
  }
}

void SetStatusRunPid()
{
  tuner.SetStausRunPid();
}

void SetStatusSample()
{
  tuner.SetStausSample();
}

void fan_init()
{
  gpio_set_function(20, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(20);
  uint channel = pwm_gpio_to_channel(20);
  
  pwm_set_clkdiv(slice_num, 230.0);  // 分频器
  pwm_set_wrap(slice_num, 1000);     // 最大计数值 (分辨率)
  pwm_set_chan_level(slice_num, channel, 0);  // 占空比
  pwm_set_enabled(slice_num, true);
}

void fan_on()
{
  uint slice_num = pwm_gpio_to_slice_num(20);
  uint channel = pwm_gpio_to_channel(20);
  pwm_set_chan_level(slice_num, channel, 1000);
}

void fan_off()
{
  uint slice_num = pwm_gpio_to_slice_num(20);
  uint channel = pwm_gpio_to_channel(20);
  pwm_set_chan_level(slice_num, channel, 0);
}