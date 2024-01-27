#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "PowerModule.hpp"

/*

    Klasa opakowywująca LVGL, sterownik ekranu i dotyku
    Wykorzystywana przez UI

*/
class Display{

public :
    void begin();
    void loop(int64_t encoderAbsCnt, ButtonState_t state);

    void printEvent(String Event, lv_event_t event);

    // Stałe określające wymiary ekranu
    static const int screenWidth = 320;
    static const int screenHeight = 240;

    static bool enable_button_state_change;

    lv_indev_t * indev_touch;
    lv_indev_t * indev_encoder;

private:

    // Enkoder
    static int16_t getEncoderDiff();
    static int64_t encoderCnt;
    static int64_t encoderPrevCnt;

    // Przycisk
    static ButtonState_t   buttonState;

    // Implementacja sterownika
    static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
    static bool touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);
    static bool encoder_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);

    // Obiekt sterownika ekranu i dotyku
    static TFT_eSPI    tft;

    // Bufory biblioteki LVGL
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf1[LV_HOR_RES_MAX * 30];
    static lv_color_t buf2[LV_HOR_RES_MAX * 30];

};

#endif