/****************************************************************************************
   sTune Library for Arduino - Version 2.4.0
   by dlloydev https://github.com/Dlloydev/sTune
   Licensed under the MIT License.

  This is an open loop PID autotuner using a novel s-curve inflection point test method.
  Tuning parameters are determined in about ½Tau on a first-order system with time delay.
  Full 5Tau testing and multiple serial output options are provided.
 ****************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "sTan.h"
#include "sTune.h"
#include "hardware/pwm.h"

sTan::sTan() {}
sTan tangent = sTan();

// sTune类的构造函数 - 默认空初始化
sTune::sTune() {
  _input = nullptr;      // 输入值指针初始化为空
  _output = nullptr;     // 输出值指针初始化为空
  sTune::Reset(0);       // 调用Reset方法重置所有参数
}

// 带参数的构造函数 - 完整初始化
sTune::sTune(float *input, float *output, TuningMethod tuningMethod, Action action, SerialMode serialMode) {
  _input = input;                // 设置输入值指针
  _output = output;              // 设置输出值指针
  _tuningMethod = tuningMethod;  // 设置调谐方法
  _action = action;              // 设置控制动作类型
  _serialMode = serialMode;      // 设置串口输出模式
  sTune::Reset(0);               // 重置所有参数
}

// 重置所有状态变量和计算参数
void sTune::Reset(const uint8_t pwmPin) {
  uint slice_num = pwm_gpio_to_slice_num(pwmPin);  // 获取PWM片选号
  uint channel = pwm_gpio_to_channel(pwmPin);      // 获取PWM通道号
  _tunerStatus = test;                             // 将状态设为测试模式
  *_output = _outputStart;                         // 将输出设为起始值
  
  // 重置时间计数和状态变量
  usPrev = micros();
  settlePrev = usPrev;
  ipUs = 0;
  us = 0;
  
  // 重置PID参数
  _Ku = 0.0f;     // 过程增益
  _Tu = 0.0f;     // 时间常数
  _td = 0.0f;     // 死区时间
  _kp = 0.0f;     // 比例增益
  _ki = 0.0f;     // 积分增益
  _kd = 0.0f;     // 微分增益
  
  // 重置过程变量
  pvIp = 0.0f;
  pvMax = 0.0f;
  pvPk = 0.0f;
  slopeIp = 0.0f;
  pvTangent = 0.0f;
  pvTangentPrev = 0.0f;
  pvAvg = pvInst;
  pvStart = pvInst;
  pvInstRes = pvInst;
  
  // 重置计数器
  ipCount = 0;
  plotCount = 0;
  sampleCount = 0;
  pvPkCount = 0;

  pwm_set_chan_level(slice_num, channel, 0);  // 将PWM输出置零
}

// 配置自动调谐器的参数
void sTune::Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
                    uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples) {
  sTune::Reset(0);                                 // 重置系统
  _inputSpan = inputSpan;                          // 输入量程
  eStop = inputSpan;                               // 紧急停止阈值
  _outputSpan = outputSpan;                        // 输出量程
  _outputStart = outputStart;                      // 输出起始值
  _outputStep = outputStep;                        // 输出步进值
  _testTimeSec = testTimeSec;                      // 测试时长(秒)
  _settleTimeSec = settleTimeSec;                  // 稳定时长(秒)
  _samples = samples;                              // 采样数量
  _bufferSize = (uint16_t)(_samples * 0.06);       // 缓冲区大小(约6%的样本数)
  _samplePeriodUs = (float)(_testTimeSec * 1000000.0f) / _samples;  // 采样周期(微秒)
  _tangentPeriodUs = _samplePeriodUs * (_bufferSize - 1);           // 切线计算周期
  _settlePeriodUs = (float)(_settleTimeSec * 1000000.0f);           // 稳定周期(微秒)
  tangent.begin(_bufferSize);                      // 初始化切线计算器
}

// 自动调谐器的主运行函数 - 状态机实现
uint8_t sTune::Run() {
  uint32_t usNow = micros();                       // 当前时间(微秒)
  uint32_t usElapsed = usNow - usPrev;             // 自上次采样后经过的时间
  uint32_t settleElapsed = usNow - settlePrev;     // 自稳定期开始后经过的时间
  us = usNow - usStart;                            // 自测试开始后总经过时间

  switch (_tunerStatus) {
    case sample:  // 采样状态 - 短暂中间状态
      _tunerStatus = test;
      return test;
      
    case test:    // 测试状态 - 执行变曲点测试方法
      // 紧急停止检查
      if (pvInst > eStop && !eStopAbort) {
        // 过程值超过紧急停止阈值，中止测试
        sTune::Reset(0);
        sampleCount = _samples + 1;
        eStopAbort = 1;
        Serial.println(F(" ABORT: pvInst > eStop"));
        break;
      }
      
      if (settleElapsed >= _settlePeriodUs) {  // 稳定期结束
        if (sampleCount == 1) *_output = _outputStep;  // 第一个样本时应用输出步进
        
        if (usElapsed >= _samplePeriodUs) {  // 到达采样时间
          usPrev = usNow;
          
          if (sampleCount <= _samples) {  // 测试未完成
            // 处理当前样本
            // 计算过程变量及其分辨率
            float lastPvInst = pvInst;
            float lastPvAvg = pvAvg;
            pvInst = *_input;                 // 读取当前输入值
            pvAvg = tangent.avgVal(pvInst);   // 计算滑动平均
            
            // 计算输入分辨率
            float pvInstResolution = fabs(pvInst - lastPvInst);
            float pvAvgResolution = fabs(pvAvg - lastPvAvg);
            if (pvInstResolution > epsilon && pvInstResolution < pvInstRes) pvInstRes = pvInstResolution;
            if (pvAvgResolution > epsilon && pvAvgResolution < pvAvgRes) pvAvgRes = pvAvgResolution;

            // 第一个样本初始化
            if (sampleCount == 0) {
              // 初始化工作
              tangent.init(pvInst);
              pvAvg = pvInst;
              pvInstResolution = 0.0f;
              pvAvgResolution = 0.0f;
              pvInstRes = pvInst;
              pvAvgRes = pvInst;
              pvStart = pvInst;
              usStart = usNow;
              us = 0;
            }

            // 计算切线(斜率)以检测变曲点
            // 使用滑动切线法识别S曲线的变曲点
            pvTangent = pvAvg - tangent.startVal();

            // 检测死区时间(过程开始响应的延迟)
            bool dt = false;
            if (_action == directIP || _action == direct5T) {
              (pvAvg > pvStart + pvInstRes + epsilon) ?  dt = true : dt = false;
            } else { // 反向动作
              (pvAvg < pvStart - pvInstRes - epsilon) ?  dt = true : dt = false;
            }
            if (!_td && dt) _td = us * 0.000001f;  // 记录死区时间(秒)

            // 检测变曲点 - 曲线斜率最大的点
            bool ipcount = false;
            if (_action == directIP || _action == direct5T) {
              if (pvTangent > slopeIp + epsilon) ipcount = true;   // 正向：斜率增加
              if (pvTangent < 0 + epsilon) ipCount = 0;            // 平坦或负斜率，重置计数器
            } else { // 反向动作
              if (pvTangent < slopeIp - epsilon) ipcount = true;   // 反向：斜率减小(更负)
              if (pvTangent > 0 - epsilon) ipCount = 0;            // 平坦或正斜率，重置计数器
            }
            
            if (ipcount) {
              ipCount = 0;         // 重置计数器
              slopeIp = pvTangent; // 记录新的最大斜率
            }
            ipCount++;  // 统计连续记录的样本数
            
            // 变曲点测试方法
            if ((_action == directIP || _action == reverseIP) && 
                (ipCount == ((uint16_t)(_samples / 16)))) { // 达到变曲点
              sampleCount = _samples;  // 设置为测试完成
              ipUs = us;               // 记录变曲点时间
              pvIp = pvAvg;            // 记录变曲点值
              
              // 估计最大值 - 基于时间常数理论
              pvMax = pvIp + (slopeIp * kexp);
              
              // 计算时间常数Tu
              _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001f) - _td;
            }

            // 5Tau测试方法 - 持续测试直到系统接近最终值
            if (_action == direct5T || _action == reverse5T) {
              if (sampleCount >= _samples - 1) sampleCount = _samples - 2;
              if (us > _testTimeSec * 100000) {  // 测试时间超过10%
                if (pvAvg > pvPk) {
                  // 设置新的最大值
                  pvPk = pvAvg + (_bufferSize * 0.2f * pvAvgRes);
                  pvPkCount = 0;  // 重置计数器
                } else {
                  pvPkCount++;  // 统计低于峰值的连续样本
                }
                
                // 测试完成条件
                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize))) {
                  pvPkCount++;
                  sampleCount = _samples;
                  pvMax = pvAvg + (pvInst - pvStart) * 0.05f;  // 假设3τ,增加5%
                  _Tu = (us * 1.6667 * 0.000001f * 0.286f) - _td; // 估算时间常数
                }
              }
            }

            // 测试完成，计算PID参数
            if (sampleCount == _samples) {
              _R = _td / _Tu;  // 计算时间比率
              
              // 计算过程增益
              _Ku =  fabs(((pvMax - pvStart) / _inputSpan) / ((_outputStep - _outputStart) / _outputSpan));
              
              // 计算PID参数
              _kp = sTune::GetKp();
              _ki = sTune::GetKi();
              _kd = sTune::GetKd();
              
              // 输出结果
              sTune::printResults();
              _tunerStatus = tunings;
              return tunings;
            }
            
            sTune::printTestRun();  // 打印当前测试状态
            pvTangentPrev = pvTangent;  // 保存当前斜率作为上一次值
            
          } else _tunerStatus = tunings;
          
          sampleCount++;  // 增加样本计数
          _tunerStatus = sample;
          return sample;
        }
      } else {  // 稳定期内
        if (usElapsed >= _samplePeriodUs && !eStopAbort) {
          *_output = _outputStart;  // 保持起始输出
          usPrev = usNow;
          pvInst = *_input;  // 读取当前输入
          
          // 打印稳定期信息
          if (_serialMode == printALL || _serialMode == printDEBUG) {
            Serial.print(F(" sec: "));     Serial.print((float)((_settlePeriodUs - settleElapsed) * 0.000001f), 4);
            Serial.print(F("  out: ")); Serial.print(*_output);
            Serial.print(F("  pv: "));     Serial.print(pvInst, 3);
            Serial.println(F("  settling  ⤳⤳"));
          }
          
          _tunerStatus = sample;
          return sample;
        }
      }
      break;
      
    // 其他状态处理...
    case tunings:  // 调谐参数计算完成
      _tunerStatus = timerPid;
      return timerPid;
    
    case runPid:   // 运行PID控制器
      if (pvInst > eStop && !eStopAbort) {  // 安全检查
        sTune::Reset(0);
        sampleCount = _samples + 1;
        eStopAbort = 1;
        Serial.println(F(" ABORT: pvInst > eStop"));
      }
      _tunerStatus = timerPid;
      return timerPid;
      
    case timerPid: // 等待下一个PID计算周期
      if (usElapsed >= _samplePeriodUs) {
        usPrev = usNow;
        _tunerStatus = runPid;
        return runPid;
      } else {
        _tunerStatus = timerPid;
        return timerPid;
      }
      
    default:
      _tunerStatus = timerPid;
      return timerPid;
  }
  return timerPid;
}

void sTune::SetEmergencyStop(float e_Stop) {
  eStop = e_Stop;
}

void sTune::SetControllerAction(Action Action) {
  _action = Action;
}

void sTune::SetSerialMode(SerialMode SerialMode) {
  _serialMode = SerialMode;
}

void sTune::SetTuningMethod(TuningMethod TuningMethod) {
  _tuningMethod = TuningMethod;
}

void sTune::printPidTuner(uint8_t everyNth) {
  if (sampleCount < _samples) {
    if (plotCount == 0 || plotCount >= everyNth) {
      plotCount = 1;
      Serial.print(us * 0.000001f, 4);  Serial.print(F(", "));
      Serial.print(*_output);           Serial.print(F(", "));
      Serial.println(pvAvg);
    } else plotCount++;
  }
}

void sTune::plotter(float input, float output, float setpoint, float outputScale, uint8_t everyNth) {
  if (plotCount >= everyNth) {
    plotCount = 1;
    Serial.print(F("Setpoint:"));  Serial.print(setpoint);              Serial.print(F(", "));
    Serial.print(F("Input:"));     Serial.print(input);                 Serial.print(F(", "));
    Serial.print(F("Output:"));    Serial.print(output * outputScale);  Serial.print(F(","));
    Serial.println();
  } else plotCount++;
}

void sTune::printTestRun() {

  if (sampleCount < _samples) {
    if (_serialMode == printALL || _serialMode == printDEBUG) {
      Serial.print(F(" sec: "));           Serial.print(us * 0.000001f, 4);
      Serial.print(F("  out: "));          Serial.print(*_output);
      Serial.print(F("  pv: "));           Serial.print(pvInst, 3);
      //Serial.print(F("  pvAvg: "));      Serial.print(pvAvg, 3);
      if (_serialMode == printDEBUG && (_action == direct5T || _action == reverse5T)) {
        Serial.print(F("  pvPk: "));       Serial.print(pvPk, 3);
        Serial.print(F("  pvPkCount: "));  Serial.print(pvPkCount);
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
        Serial.print(F("  ipCount: "));    Serial.print(ipCount);
      }
      Serial.print(F("  tan: "));                       Serial.print(pvTangent, 3);
      if (pvInst > 0.9f * eStop)                        Serial.print(F(" ⚠"));
      if (pvTangent - pvTangentPrev > 0 + epsilon)      Serial.println(F(" ↗"));
      else if (pvTangent - pvTangentPrev < 0 - epsilon) Serial.println(F(" ↘"));
      else                                              Serial.println(F(" →"));
    }
  }
}

void sTune::printTunings() {
  Serial.print(F(" Tuning Method: "));
  if (_tuningMethod == ZN_PID) Serial.println(F("ZN_PID"));
  else if (_tuningMethod == DampedOsc_PID) Serial.println(F("Damped_PID"));
  else if (_tuningMethod == NoOvershoot_PID) Serial.println(F("NoOvershoot_PID"));
  else if (_tuningMethod == CohenCoon_PID) Serial.println(F("CohenCoon_PID"));
  else if (_tuningMethod == Mixed_PID) Serial.println(F("Mixed_PID"));
  else if (_tuningMethod == ZN_PI) Serial.println(F("ZN_PI"));
  else if (_tuningMethod == DampedOsc_PI) Serial.println(F("Damped_PI"));
  else if (_tuningMethod == NoOvershoot_PI) Serial.println(F("NoOvershoot_PI"));
  else if (_tuningMethod == CohenCoon_PI) Serial.println(F("CohenCoon_PI"));
  else Serial.println(F("Mixed_PI"));
  Serial.print(F("  Kp: ")); Serial.println(sTune::GetKp(), 3);
  Serial.print(F("  Ki: ")); Serial.print(sTune::GetKi(), 3); Serial.print(F("  Ti: ")); Serial.println(sTune::GetTi(), 3);
  Serial.print(F("  Kd: ")); Serial.print(sTune::GetKd(), 3); Serial.print(F("  Td: ")); Serial.println(sTune::GetTd(), 3);
  Serial.println();
}

void sTune::printResults() {
  if (_serialMode == printALL || _serialMode == printDEBUG || _serialMode == printSUMMARY) {
    Serial.println();
    Serial.print(F(" Controller Action: "));
    if (_action == directIP) Serial.println(F("directIP"));
    else if (_action == direct5T) Serial.println(F("direct5T"));
    else if (_action == reverseIP) Serial.println(F("reverseIP"));
    else Serial.println(F("reverse5T"));
    Serial.println();
    Serial.print(F(" Output Start:      "));  Serial.println(_outputStart);
    Serial.print(F(" Output Step:       "));  Serial.println(_outputStep);
    Serial.print(F(" Sample Sec:        "));  Serial.println(_samplePeriodUs * 0.000001f, 4);
    Serial.println();
    if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP)) {
      Serial.print(F(" Ip Sec:            "));  Serial.println(ipUs * 0.000001f, 4);
      Serial.print(F(" Ip Slope:          "));  Serial.print(slopeIp, 3);
      if (_action == directIP || _action == direct5T) Serial.println(F(" ↑"));
      else  Serial.println(F(" ↓"));
      Serial.print(F(" Ip Pv:             "));  Serial.println(pvIp, 3);
    }
    Serial.print(F(" Pv Start:          "));  Serial.println(pvStart, 3);
    if (_action == directIP || _action == direct5T) Serial.print(F(" Pv Max:            "));
    else Serial.print(F(" Pv Min:            "));
    Serial.println(pvMax, 3);
    Serial.print(F(" Pv Diff:           "));  Serial.println(pvMax - pvStart, 3);
    Serial.println();
    Serial.print(F(" Process Gain:      "));  Serial.println(_Ku, 3);
    Serial.print(F(" Dead Time Sec:     "));  Serial.println(_td, 3);
    Serial.print(F(" Tau Sec:           "));  Serial.println(_Tu, 3);
    Serial.println();

    // Controllability https://blog.opticontrols.com/wp-content/uploads/2011/06/td-versus-tau.png
    float controllability = _Tu / _td + epsilon;
    if (controllability > 99.9) controllability = 99.9;
    Serial.print(F(" Tau/Dead Time:     "));  Serial.print(controllability, 1);
    if (controllability > 0.75) Serial.println(F(" (easy to control)"));
    else if (controllability > 0.25) Serial.println(F(" (average controllability)"));
    else Serial.println(F(" (difficult to control)"));

    // check “best practice” rule that sample time should be ≥ 10 times per process time constant
    // https://controlguru.com/sample-time-is-a-fundamental-design-and-tuning-specification/
    float sampleTimeCheck = _Tu / (_samplePeriodUs * 0.000001f);
    Serial.print(F(" Tau/Sample Period: "));  Serial.print(sampleTimeCheck, 1);
    if (sampleTimeCheck >= 10) Serial.println(F(" (good sample rate)"));
    else Serial.println(F(" (low sample rate)"));
    Serial.println();
    sTune::printTunings();
    sampleCount++;
  }
}

// Query functions

void sTune::GetAutoTunings(float * kp, float * ki, float * kd) {
  *kp = _kp;
  *ki = _ki;
  *kd = _kd;
}

// https://blog.opticontrols.com/archives/477

float sTune::GetKp() {
  float znPid = ((1.2f * _Tu) / (_Ku * _td)) / 2;
  float doPid = (0.66f * _Tu) / (_Ku * _td);
  float noPid = (0.6f / _Ku) * (_Tu / _td);
  float ccPid = _Ku * (1.33f + (_R / 4.0f));
  float znPi = ((0.9f * _Tu) / (_Ku * _td)) / 2;
  float doPi = (0.495f * _Tu) / (_Ku * _td);
  float noPi = (0.35f / _Ku) * (_Tu / _td);
  float ccPi = _Ku * (0.9f + (_R / 12.0f));
  if (_tuningMethod == ZN_PID)                _kp = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _kp = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _kp = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _kp = ccPid;
  else if (_tuningMethod == Mixed_PID)        _kp = 0.25f * (znPid + doPid + noPid + ccPid);
  else if (_tuningMethod == ZN_PI)            _kp = znPi;
  else if (_tuningMethod == DampedOsc_PI)     _kp = doPi;
  else if (_tuningMethod == NoOvershoot_PI)   _kp = noPi;
  else if (_tuningMethod == CohenCoon_PI)     _kp = ccPi;
  else                                        _kp = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI
  return _kp;
}

float sTune::GetKi() {
  float znPid = 1 / (2.0f * _td);
  float doPid = 1 / (_Tu / 3.6f);
  float noPid = 1 / (_Tu);
  float ccPid = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));
  float znPi = 1 / (3.3333f * _td);
  float doPi = 1 / (_Tu / 2.6f);
  float noPi = 1 / (1.2f * _Tu);
  float ccPi = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));

  if (_tuningMethod == ZN_PID)                _ki = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _ki = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _ki = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _ki = ccPid;
  else if (_tuningMethod == Mixed_PID)        _ki = 0.25f * (znPid + doPid + noPid + ccPid);
  else if (_tuningMethod == ZN_PI)            _ki = znPi;
  else if (_tuningMethod == DampedOsc_PI)     _ki = doPi;
  else if (_tuningMethod == NoOvershoot_PI)   _ki = noPi;
  else if (_tuningMethod == CohenCoon_PI)     _ki = ccPi;
  else                                        _ki = 0.25f * (znPi + doPi + noPi + ccPi); // Mixed_PI
  return _ki;
}

float sTune::GetKd() {
  float znPid = 1 / (0.5f * _td);
  float doPid = 1 / (_Tu / 9.0f);
  float noPid = 1 / (0.5f * _td);
  float ccPid = 1 / ((4.0f * _td) / (11.0f + (2.0f * _R)));
  if (_tuningMethod == ZN_PID)                _kd = znPid;
  else if (_tuningMethod == DampedOsc_PID)    _kd = doPid;
  else if (_tuningMethod == NoOvershoot_PID)  _kd = noPid;
  else if (_tuningMethod == CohenCoon_PID)    _kd = ccPid;
  else if (_tuningMethod == Mixed_PID)        _kd = 0.25f * (znPid + doPid + noPid + ccPid);
  else                                        _kd = 0.0f; // PI controller
  return _kd;
}

float sTune::GetTi() {
  return _kp / _ki;
}

float sTune::GetTd() {
  if (_tuningMethod == ZN_PID ||
      _tuningMethod == DampedOsc_PID ||
      _tuningMethod == NoOvershoot_PID ||
      _tuningMethod == CohenCoon_PID ||
      _tuningMethod == Mixed_PID) {
    return _kp / _kd;
  }
  else return 0;
}

float sTune::GetProcessGain() {
  return _Ku;
}

float sTune::GetDeadTime() {
  return _td;
}

float sTune::GetTau() {
  return _Tu;
}

uint8_t sTune::GetControllerAction() {
  return static_cast<uint8_t>(_action);
}

uint8_t sTune::GetSerialMode() {
  return static_cast<uint8_t>(_serialMode);
}

uint8_t sTune::GetTuningMethod() {
  return static_cast<uint8_t>(_tuningMethod);
}

// PWM初始化函数
void sTune::initHardwarePwm(const uint8_t pwmPin) {
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(pwmPin);
  uint channel = pwm_gpio_to_channel(pwmPin);
  
  pwm_set_clkdiv(slice_num, 230.0);  // 分频器
  pwm_set_wrap(slice_num, 10000);     // 最大计数值 (分辨率)
  pwm_set_chan_level(slice_num, channel, 0);  // 占空比
  pwm_set_enabled(slice_num, true);
}

float sTune::hardwarePwm(const uint8_t pwmPin, float input, float output, float setpoint) {

  static float optimumOutput;
  static bool reachedSetpoint;
  uint slice_num = pwm_gpio_to_slice_num(pwmPin);
  uint channel = pwm_gpio_to_channel(pwmPin);

  // 优化控制逻辑
  if (input > setpoint)
  {
    reachedSetpoint = true;
  }
  if (reachedSetpoint && setpoint > 0 && input > setpoint)
  {
    optimumOutput = output - 8;
  }
  else if (reachedSetpoint && setpoint > 0 && input < setpoint)
  {
     optimumOutput = output + 8;
  }
  else optimumOutput = output;
  
  // 限制输出范围
  if (optimumOutput < 0) optimumOutput = 0;
  if (optimumOutput > 100) optimumOutput = 100;

  // 直接输出到硬件PWM
  pwm_set_chan_level(slice_num, channel, (int)optimumOutput * 100);

  return optimumOutput;
}
