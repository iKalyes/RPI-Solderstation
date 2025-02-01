SET(SOURCES screens/ui_MainScreen.c
    screens/ui_TemperatureSetting.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_740489818.c
    images/ui_img_1699618864.c
    images/ui_img_290686881.c
    images/ui_img_1956478733.c
    images/ui_img_155116563.c
    images/ui_img_1162706898.c
    images/ui_img_1345483443.c
    images/ui_img_817906860.c
    images/ui_img_1577463476.c
    images/ui_img_1006688281.c
    images/ui_img_1428031618.c
    images/ui_img_1680627741.c
    images/ui_img_1671917170.c
    images/ui_img_1347724781.c
    images/ui_img_413634367.c
    images/ui_img_938671949.c
    images/ui_img_13634893.c
    images/ui_img_1543630939.c
    images/ui_img_761595314.c
    images/ui_img_154154061.c
    images/ui_img_2056179636.c
    images/ui_img_2011795637.c
    images/ui_img_72837962.c
    images/ui_img_988587337.c
    images/ui_img_1221746360.c
    images/ui_img_427413075.c
    images/ui_img_2131435434.c
    fonts/ui_font_ASCII.c
    fonts/ui_font_ASCII48.c)

add_library(ui ${SOURCES})
