#include <ui-tempset.h>

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
    if (event_code == LV_EVENT_PRESSED) {
        if(tempset_status == 1)
        {
        // 添加输入验证
        if (!tempDisplay || strlen(tempDisplay) > 3) {
            return;
        }
        // 使用更安全的转换方式
        char *endptr;
        long temp_val = strtol(tempDisplay, &endptr, 10);
        // 验证转换结果
        if (endptr == tempDisplay || *endptr != '\0') {
            return;
        }
        SetTemp = (int)temp_val;
        // 缓冲区增大为5以确保安全
        char temp_str[5] = {0};
        if (SetTemp == 0) {
            lv_label_set_text(ui_TargetTemp, "000");
        }
        else if(SetTemp > temp_limited) {
            SetTemp = temp_limited;
            // 使用snprintf防止缓冲区溢出
            snprintf(temp_str, sizeof(temp_str), "%d", temp_limited);
            lv_label_set_text(ui_TargetTemp, temp_str);
        }
        else {
            // 使用snprintf防止缓冲区溢出
            snprintf(temp_str, sizeof(temp_str), "%d", SetTemp);
            lv_label_set_text(ui_TargetTemp, temp_str);
        }
        // 切换回主屏幕
        _ui_screen_change(&ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_MainScreen_screen_init);
        tempset_status = 0;
    }
/////////////////////////////////////////////////////////////////////////////
        if(tempset_status == 2)
        {
        // 添加输入验证
        if (!tempDisplay || strlen(tempDisplay) > 3) {
            return;
        }
        // 使用更安全的转换方式
        char *endptr;
        long temp_val = strtol(tempDisplay, &endptr, 10);
        // 验证转换结果
        if (endptr == tempDisplay || *endptr != '\0') {
            return;
        }
        PIDSetTemp = (int)temp_val;
        // 缓冲区增大为5以确保安全
        char temp_str[5] = {0};
        if (PIDSetTemp == 0) {
            lv_label_set_text(ui_PIDTargetTemp, "000");
        }
        else if(PIDSetTemp > temp_limited) {
            PIDSetTemp = temp_limited;
            // 使用snprintf防止缓冲区溢出
            snprintf(temp_str, sizeof(temp_str), "%d", temp_limited);
            lv_label_set_text(ui_PIDTargetTemp, temp_str);
        }
        else {
            // 使用snprintf防止缓冲区溢出
            snprintf(temp_str, sizeof(temp_str), "%d", PIDSetTemp);
            lv_label_set_text(ui_PIDTargetTemp, temp_str);
        }
        // 切换回主屏幕
        _ui_screen_change(&ui_PIDCalibrationScreen, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_PIDCalibrationScreen_screen_init);
        tempset_status = 0;
    }
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
        PIDSetTemp = 0;
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