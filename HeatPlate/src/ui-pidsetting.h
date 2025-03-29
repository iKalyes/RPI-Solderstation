#ifndef _UI_PIDSETTING_H_
#define _UI_PIDSETTING_H_

#include <lvgl.h>
#include "ui/ui.h"
#include "ui/ui_helpers.h"
#include "ui/ui_events.h"
#include "ui/ui_theme_manager.h"
#include "ui/ui_themes.h"

#include <Arduino.h>
#include <variables.h>
#include "QuickPID/src/QuickPID.h"
#include <lvgl_group.h>

void pid_setting();
void pid_update();

#endif