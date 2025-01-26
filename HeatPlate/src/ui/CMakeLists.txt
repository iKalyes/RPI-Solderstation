SET(SOURCES ui_Screen1.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_setting_png.c
    fonts/ui_font_SarasaMonoASCII18.c)

add_library(ui ${SOURCES})
