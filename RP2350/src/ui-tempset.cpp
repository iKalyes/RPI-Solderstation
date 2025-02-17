#include <ui-tempset.h>

uint16_t SetTemp = 0;
char tempDisplay[4] = "000";  // 初始化为"000"而不是空格
uint8_t inputPos = 0;        // 输入位置指针

// 更新显示函数
void updateDisplay() {
    tempDisplay[3] = '\0';
    lv_label_set_text(ui_SetTemp, tempDisplay);
}

// 数字输入处理函数
void handleNumberInput(char num) {
    if (inputPos < 3) {
        // 输入新数字
        tempDisplay[inputPos] = num;
        // 确保后面的位置保持为'0'
        for (int i = inputPos + 1; i < 3; i++) {
            tempDisplay[i] = '0';
        }
        inputPos++;
        updateDisplay();
    }
}

void ui_event_Confirm(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_RELEASED) {
        // 将显示的字符串转换为数字
        sscanf(tempDisplay, "%d", &SetTemp);

        if (SetTemp == 0)
        {
            lv_label_set_text(ui_TargetTemp, "000");
        }
        else
        {
            // 创建临时字符数组来存储转换后的数字字符串
            char temp_str[8];
            sprintf(temp_str, "%d", SetTemp);
            // 更新目标温度显示
            lv_label_set_text(ui_TargetTemp, temp_str);
        }
        
        // 切换回主屏幕
        _ui_screen_change(&ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 50, 0, &ui_MainScreen_screen_init);
    }
}

void ui_event_Delete(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        for(int i = 0; i < 3; i++) {
            tempDisplay[i] = '0';
        }
        inputPos = 0;
        SetTemp = 0;
        updateDisplay();
    }
}

void ui_event_Num0(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('0');
    }
}

void ui_event_Num1(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('1');
    }
}

void ui_event_Num2(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('2');
    }
}

void ui_event_Num3(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('3');
    }
}

void ui_event_Num4(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('4');
    }
}

void ui_event_Num5(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('5');
    }
}

void ui_event_Num6(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('6');
    }
}

void ui_event_Num7(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('7');
    }
}

void ui_event_Num8(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('8');
    }
}

void ui_event_Num9(lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_PRESSED) {
        handleNumberInput('9');
    }
}