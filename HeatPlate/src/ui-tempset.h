#ifndef _UI_TEMPSET_H_
#define _UI_TEMPSET_H_

#include <lvgl.h>
#include "ui/ui.h"
#include "ui/ui_helpers.h"
#include "ui/ui_events.h"
#include "ui/ui_theme_manager.h"
#include "ui/ui_themes.h"

#include <Arduino.h>
#include <variables.h>

void updateDisplay();
void handleNumberInput(char num);

#endif