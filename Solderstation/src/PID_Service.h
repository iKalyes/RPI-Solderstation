#ifndef _PID_SERVICE_H
#define _PID_SERVICE_H

#include <Arduino.h>
#include <QuickPID.h>
#include <GPIO_Service.h>
#include <MAX6675_Service.h>
#include "variables.h"
#include <lvgl.h>
#include "ui/ui.h"

// 创建float类型的中间变量用于PID计算
static float soldering_temp_float = 0.0f;
static float soldering_target_temp_float = 0.0f;

// 待机休眠相关变量
static struct {
    unsigned long standby_start_time;   // 待机开始时间
    bool ui_color_changed;             // UI颜色是否已改变
    bool in_standby_mode;              // 当前是否处于待机模式 (由防抖后的sleep信号决定)
    unsigned long sleep_read_time;     // 休眠状态下的读取时间控制

    // 新增防抖相关成员
    bool prev_raw_sleep_signal;         // 上一次读取的原始 sleep 信号
    bool debounced_sleep_signal;        // 防抖处理后的 sleep 信号
    unsigned long last_raw_signal_change_time; // 原始信号上一次发生变化的时间戳
    bool last_processed_debounced_signal; // 上一次处理过的（用于边沿检测）防抖信号
} standby_state = {
    0,      // standby_start_time
    false,  // ui_color_changed
    false,  // in_standby_mode
    0,      // sleep_read_time
    false,  // prev_raw_sleep_signal (将在Init中正确设置)
    false,  // debounced_sleep_signal (将在Init中正确设置)
    0,      // last_raw_signal_change_time
    false   // last_processed_debounced_signal (将在Init中正确设置)
};

// 添加状态机枚举和静态变量
enum SolderingPIDState {
    PID_POWER_OFF,
    PID_WAIT_STABLE,
    PID_READ_TEMP,
    PID_HEATING,
    PID_SLEEP           // 休眠状态
};

static SolderingPIDState pid_state = PID_POWER_OFF;
static unsigned long state_start_time = 0;

// 蜂鸣器控制变量 - 重新组织以提高缓存效率
static struct {
    bool temperature_reached_played;    // 是否已鸣叫过到达温度
    bool short_active;                  // 短鸣叫是否激活
    bool long_active;                   // 长鸣叫是否激活
    unsigned long start_time;           // 蜂鸣器开始时间
} buzzer_state = {false, false, false, 0};

// 状态跟踪变量
static struct {
    uint16_t last_target_temp;          // 上次的目标温度
    bool was_disabled;                  // 电烙铁之前是否被禁用
} soldering_state = {0, true};

void Soldering_PID_Compute_Init();
void Soldering_PID_Update_Tunings(float Kp, float Ki, float Kd);
void Soldering_PID_Compute();


void Heatgun_PID_Compute_Init();
void HeatgunPID_Update_Tunings(float Kp, float Ki, float Kd);
void Heatgun_PID_Compute();

#endif