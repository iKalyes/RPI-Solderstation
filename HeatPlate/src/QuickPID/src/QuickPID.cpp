/**********************************************************************************
   QuickPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QuickPID
   基于Arduino PID_v1库。使用MIT许可证。
 **********************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QuickPID.h"

QuickPID::QuickPID() {}

/* 构造函数 ********************************************************************
   这里指定的参数是那些我们无法设置可靠默认值的参数，
   因此需要用户进行设置。
 **********************************************************************************/

QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp = 0, float Ki = 0, float Kd = 0,
                   pMode pMode = pMode::pOnError,
                   dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode = iAwMode::iAwCondition,
                   Action Action = Action::direct) {

  myOutput = Output;       // 输出变量指针
  myInput = Input;         // 输入变量指针
  mySetpoint = Setpoint;   // 设定点变量指针
  mode = Control::manual;  // 默认为手动模式

  QuickPID::SetOutputLimits(0, 100);  // 与Arduino PWM范围相同的默认限制
  sampleTimeUs = 100000;              // 默认0.1秒采样时间
  QuickPID::SetControllerDirection(Action);  // 设置控制器方向
  QuickPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode);  // 设置PID参数

  lastTime = micros() - sampleTimeUs;  // 初始化时间戳
}

/* 构造函数 *********************************************************************
   允许使用pOnError、dOnMeas和iAwCondition，而无需显式说明。
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint,
                   float Kp, float Ki, float Kd, Action Action)
  : QuickPID::QuickPID(Input, Output, Setpoint, Kp, Ki, Kd,
                       pmode = pMode::pOnError,
                       dmode = dMode::dOnMeas,
                       iawmode = iAwMode::iAwCondition,
                       action = Action) {
}

/* 构造函数 *********************************************************************
   简化的构造函数，为剩余参数使用默认值。
 **********************************************************************************/
QuickPID::QuickPID(float* Input, float* Output, float* Setpoint)
  : QuickPID::QuickPID(Input, Output, Setpoint,
                       dispKp = 0,
                       dispKi = 0,
                       dispKd = 0,
                       pmode = pMode::pOnError,
                       dmode = dMode::dOnMeas,
                       iawmode = iAwMode::iAwCondition,
                       action = Action::direct) {
}

/* Compute() ***********************************************************************
   此函数应在每次"void loop()"执行时调用。函数将决定
   是否需要计算新的PID输出值。当输出被计算时返回true，
   当没有执行任何操作时返回false。
 **********************************************************************************/
bool QuickPID::Compute() {
  if (mode == Control::manual) return false;  // 手动模式下不计算
  uint32_t now = micros();                    // 获取当前时间
  uint32_t timeChange = (now - lastTime);     // 计算时间变化
  if (mode == Control::timer || timeChange >= sampleTimeUs) {  // 定时器模式或达到采样时间

    float input = *myInput;                   // 获取当前输入值
    float dInput = input - lastInput;         // 计算输入变化量
    if (action == Action::reverse) dInput = -dInput;  // 如果是反向动作，反转输入变化
    
    // 标准误差计算
    float originalError = *mySetpoint - input;
    error = originalError;
    if (action == Action::reverse) error = -error;  // 如果是反向动作，反转误差
    float dError = error - lastError;         // 计算误差变化量
    
    // 如果误差很大(远低于设定点)，直接提供最大输出
    if (originalError > approachRange * 1.5 && dInput >= 0) {
      *myOutput = outMax;  // 当温度远低于目标且没有快速上升时，提供最大输出
      lastInput = input;
      lastError = error;
      lastTime = now;
      return true;
    }
    
    // 预测控制处理 - 只有在接近设定点且温度仍在上升时才使用
    bool usePrediction = false;
    float predictedInput = input;
    
    if (usePredictControl && 
        abs(originalError) < approachRange && // 在接近设定点范围内
        dInput > 0) {  // 温度仍在上升
      
      // 预测未来温度
      predictedInput = input + predictGain * dInput;
      
      // 如果预测温度会超过设定点，则启用预测控制
      if ((action == Action::direct && predictedInput > *mySetpoint) || 
          (action == Action::reverse && predictedInput < *mySetpoint)) {
        usePrediction = true;
        error = *mySetpoint - predictedInput; // 使用预测误差
        if (action == Action::reverse) error = -error;
      }
    }

    // 计算比例项：可以基于误差、测量值或两者的组合
    float peTerm = kp * error;                // 误差比例项
    float pmTerm = kp * dInput;               // 测量值比例项
    if (pmode == pMode::pOnError) pmTerm = 0;  // 仅使用误差
    else if (pmode == pMode::pOnMeas) peTerm = 0;  // 仅使用测量值
    else { //pOnErrorMeas
      peTerm *= 0.5f;                         // 两者各占一半
      pmTerm *= 0.5f;
    }
    pTerm = peTerm - pmTerm;                  // 最终比例项，由GetDterm()使用
    iTerm =  ki  * error;                     // 积分项
    
    // 计算微分项：可以基于误差或测量值
    if (dmode == dMode::dOnError) {
      dTerm = kd * dError;  // 基于误差的微分
    } else {
      dTerm = -kd * dInput;  // 基于测量值的微分(dOnMeas)
      
      // 预测控制时增强微分作用抑制过冲
      if (usePrediction) {
        // 根据预测增加微分增益
        float tempKd = kd * (1.0 + abs(dInput) * predictGain * 0.5);
        dTerm = -tempKd * dInput;
      }
    }

    // 条件抗积分饱和(默认)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);  // 预测输出
      // 判断是否需要抗积分饱和
      if (iTermOut > outMax && dError > 0) aw = true;       // 输出将超过上限且误差增大
      else if (iTermOut < outMin && dError < 0) aw = true;  // 输出将低于下限且误差减小
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);  // 限制积分项
    }

    // 默认情况下，按照PID_v1方式计算输出
    outputSum += iTerm;                       // 包含积分量
    if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;  // 包含pmTerm (无抗饱和)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);  // 包含pmTerm并限幅
    
    // 计算PID输出
    float baseOutput = constrain(outputSum + pTerm + dTerm, outMin, outMax);
    
    // 如果预测控制被触发且仅当在接近设定点时调整输出
    if (usePrediction && dInput > 0 && abs(originalError) < approachRange) {
      float overShootAmount = predictedInput - *mySetpoint;
      if (overShootAmount > 0) {  // 只有确实预测会超调时才减小输出
        float adjustFactor = 1.0 - (overShootAmount / approachRange);
        adjustFactor = constrain(adjustFactor, 0.2, 1.0);
        *myOutput = baseOutput * adjustFactor;
      } else {
        // 不会超调，使用正常输出
        *myOutput = baseOutput;
      }
    } else {
      // 标准输出计算
      *myOutput = baseOutput;
    }

    lastError = error;    // 保存当前误差
    lastInput = input;    // 保存当前输入
    lastTime = now;       // 更新时间戳
    return true;          // 返回计算成功
  }
  else return false;      // 未达到计算条件
}

/* EnablePredictControl *********************************************************
   启用预测控制功能，用于处理高热惯性系统，防止温度过冲
******************************************************************************/
void QuickPID::EnablePredictControl(bool enable, float predictGain, float approachRange) {
    usePredictControl = enable;
    this->predictGain = predictGain;
    this->approachRange = approachRange;
}

/* SetTunings(....) ************************************************************
  此函数允许调整控制器的动态性能。
  它在构造函数中自动调用，但也可以在正常运行期间动态调整。
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd,
                          pMode pMode = pMode::pOnError,
                          dMode dMode = dMode::dOnMeas,
                          iAwMode iAwMode = iAwMode::iAwCondition) {

  if (Kp < 0 || Ki < 0 || Kd < 0) return;  // 参数必须为非负数
  if (Ki == 0) outputSum = 0;              // Ki为0时重置积分和
  pmode = pMode; dmode = dMode; iawmode = iAwMode;  // 设置各种模式
  dispKp = Kp; dispKi = Ki; dispKd = Kd;   // 存储显示用参数
  float SampleTimeSec = (float)sampleTimeUs / 1000000;  // 转换为秒
  kp = Kp;
  ki = Ki * SampleTimeSec;                 // 积分项需要考虑采样时间
  kd = Kd / SampleTimeSec;                 // 微分项需要考虑采样时间
}

/* SetTunings(...) ************************************************************
  使用之前记住的pMode、dMode和iAwMode设置设置参数。
******************************************************************************/
void QuickPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pmode, dmode, iawmode);
}

/* SetSampleTime(.) ***********************************************************
  设置执行计算的时间间隔，单位为微秒。
******************************************************************************/
void QuickPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;  // 计算时间比率
    ki *= ratio;     // 调整ki以适应新的采样时间
    kd /= ratio;     // 调整kd以适应新的采样时间
    sampleTimeUs = NewSampleTimeUs;  // 更新采样时间
  }
}

/* SetOutputLimits(..) ********************************************************
  PID控制器设计为在给定范围内变化其输出。
  默认情况下，此范围为0-100，即Arduino PWM范围。
******************************************************************************/
void QuickPID::SetOutputLimits(float Min, float Max) {
  if (Min >= Max) return;  // 最小值必须小于最大值
  outMin = Min;            // 设置最小输出限制
  outMax = Max;            // 设置最大输出限制

  if (mode != Control::manual) {  // 如果不是手动模式
    *myOutput = constrain(*myOutput, outMin, outMax);     // 限制当前输出
    outputSum = constrain(outputSum, outMin, outMax);     // 限制积分和
  }
}

/* SetMode(.) *****************************************************************
  将控制器模式设置为手动(0)、自动(1)或定时器(2)
  当从手动模式转换到自动或定时器模式时，控制器会自动初始化。
******************************************************************************/
void QuickPID::SetMode(Control Mode) {
  if (mode == Control::manual && Mode != Control::manual) { // 刚从手动变为自动、定时器或切换
    QuickPID::Initialize();  // 初始化控制器
  }
  if (Mode == Control::toggle) {  // 切换模式
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = Mode;  // 直接设置模式
}
void QuickPID::SetMode(uint8_t Mode) {
  if (mode == Control::manual && Mode != 0) { // 刚从手动变为自动或定时器
    QuickPID::Initialize();  // 初始化控制器
  }
  if (Mode == 3) { // 切换
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = (Control)Mode;  // 设置为指定模式
}

/* Initialize() ****************************************************************
  执行所有需要进行的操作，以确保从手动模式到自动模式的平滑过渡。
******************************************************************************/
void QuickPID::Initialize() {
  outputSum = *myOutput;     // 使用当前输出作为起点
  lastInput = *myInput;      // 记录当前输入
  outputSum = constrain(outputSum, outMin, outMax);  // 限制积分和在输出范围内
}

/* SetControllerDirection(.) **************************************************
  PID将连接到正向过程(+输出导致+输入)或
  反向过程(+输出导致-输入)。
******************************************************************************/
void QuickPID::SetControllerDirection(Action Action) {
  action = Action;  // 设置控制动作
}
void QuickPID::SetControllerDirection(uint8_t Direction) {
  action = (Action)Direction;  // 使用数值设置控制动作
}

/* SetProportionalMode(.) *****************************************************
  设置比例项的计算方法，可以基于误差(默认)、
  测量值，或两者的平均值计算。
******************************************************************************/
void QuickPID::SetProportionalMode(pMode pMode) {
  pmode = pMode;  // 设置比例模式
}
void QuickPID::SetProportionalMode(uint8_t Pmode) {
  pmode = (pMode)Pmode;  // 使用数值设置比例模式
}

/* SetDerivativeMode(.) *******************************************************
  设置微分项的计算方法，可以基于误差或
  测量值(默认)计算。
******************************************************************************/
void QuickPID::SetDerivativeMode(dMode dMode) {
  dmode = dMode;  // 设置微分模式
}
void QuickPID::SetDerivativeMode(uint8_t Dmode) {
  dmode = (dMode)Dmode;  // 使用数值设置微分模式
}

/* SetAntiWindupMode(.) *******************************************************
  设置积分抗饱和模式为iAwClamp，它在添加积分和比例项(基于测量)后
  限制输出；或iAwCondition(默认)，它提供一些积分校正，
  防止深度饱和并减少超调。
  选项iAwOff完全禁用抗饱和。
******************************************************************************/
void QuickPID::SetAntiWindupMode(iAwMode iAwMode) {
  iawmode = iAwMode;  // 设置抗饱和模式
}
void QuickPID::SetAntiWindupMode(uint8_t IawMode) {
  iawmode = (iAwMode)IawMode;  // 使用数值设置抗饱和模式
}

void QuickPID::Reset() {
  lastTime = micros() - sampleTimeUs;  // 重置时间戳
  lastInput = 0;      // 重置上一次输入
  outputSum = 0;      // 重置积分和
  pTerm = 0;          // 重置比例项
  iTerm = 0;          // 重置积分项
  dTerm = 0;          // 重置微分项
}

// 设置输出累加值
void QuickPID::SetOutputSum(float sum) {
  outputSum = sum;    // 直接设置积分和
}

/* 状态查询函数 ************************************************************
  这些函数查询PID的内部状态。
******************************************************************************/
float QuickPID::GetKp() {
  return dispKp;      // 获取比例系数
}
float QuickPID::GetKi() {
  return dispKi;      // 获取积分系数
}
float QuickPID::GetKd() {
  return dispKd;      // 获取微分系数
}
float QuickPID::GetPterm() {
  return pTerm;       // 获取当前比例项
}
float QuickPID::GetIterm() {
  return iTerm;       // 获取当前积分项
}
float QuickPID::GetDterm() {
  return dTerm;       // 获取当前微分项
}
float QuickPID::GetOutputSum() {
  return outputSum;   // 获取输出累加值
}
uint8_t QuickPID::GetMode() {
  return static_cast<uint8_t>(mode);    // 获取当前模式
}
uint8_t QuickPID::GetDirection() {
  return static_cast<uint8_t>(action);  // 获取控制方向
}
uint8_t QuickPID::GetPmode() {
  return static_cast<uint8_t>(pmode);   // 获取比例模式
}
uint8_t QuickPID::GetDmode() {
  return static_cast<uint8_t>(dmode);   // 获取微分模式
}
uint8_t QuickPID::GetAwMode() {
  return static_cast<uint8_t>(iawmode); // 获取抗饱和模式
}
