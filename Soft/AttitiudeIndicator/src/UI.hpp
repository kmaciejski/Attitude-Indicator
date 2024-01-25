#ifndef UI_HPP
#define UI_HPP

#include "Display.hpp"
#include "Sensors.hpp"

typedef struct{
    uint16_t    voltage_mV;
    ChargingState_t state;
}BatteryInfo_t;

typedef enum {
  SCREEN_NULL           = 0,
  SCREEN_DIAGNOSTIC     = 1,
  SCREEN_MAIN           = 2,
  SCREEN_SELECTION      = 3,
  SCREEN_SHUTDOWN       = 4
}Screen_t;

typedef enum{
    ENTRY   = 2,
    UPDATE  = 3,
    EXIT    = 4
}ScreenState_t;

// Main screen
    typedef struct{
        lv_obj_t * ui_Screen;
        lv_obj_t * ui_BarBat;
        lv_obj_t * ui_LabelBatStatus;
        lv_obj_t * ui_LabelBatVoltage;

        lv_obj_t * ui_LabelHdg;
        lv_obj_t * ui_LabelMid;

        lv_obj_t * ui_LabelAltitiude;
        lv_obj_t * ui_LabelQnh;
        lv_obj_t * ui_LabelFpm;
        lv_obj_t * ui_LabelTemp;

        lv_obj_t * ui_Om;
        lv_objmask_mask_t * ui_ObjMask;

        BatteryInfo_t battery;
        AttitiudeInfo_t attitiude;
    }MainScreen_t;


// Diag screen
    typedef struct{
        lv_obj_t * ui_Screen;
        lv_obj_t * ui_SliderH;
        lv_obj_t * ui_SliderV;
        lv_obj_t * ui_Arc;
        lv_obj_t * ui_LabelHdg;
        lv_obj_t * ui_LabelPitch;
        lv_obj_t * ui_LabelRoll;
        lv_obj_t * ui_SwitchCal;
        lv_obj_t * ui_LabelCal;

        lv_group_t* ui_Group;

        AttitiudeInfo_t attitiude;
    }DiagScreen_t;


// Shutdown screen
    typedef struct{
        lv_obj_t * ui_Screen;

        lv_obj_t * ui_Label;

    }ShutdownScreen_t;


// Selection screen
    typedef struct{
        lv_obj_t * ui_Screen;

        lv_obj_t * ui_Label;
        lv_obj_t * ui_ButtonMain;
        lv_obj_t * ui_ButtonDiag;
        lv_group_t * ui_Group;

    }SelectionScreen_t;

/*
    Interfejs użytkownika
*/
class UI{

public :
    void begin();
    void loop(int64_t encoderValue, PowerModule &pm, Sensors &gyro);

    static void setScreen(Screen_t scr){currentScreen = scr;}

private:

    // Ekrany
    void createDiagnosticScreen();
    void createSelectionScreen();
    void createShutdownScreen();
    void createMainScreen();


    void updateSelectionScreen(ScreenState_t screenState, ButtonState_t buttonState);
    void updateDiagnosticScreen(ScreenState_t screenState, AttitiudeInfo_t attitiude);
    void updateShutdownScreen(ScreenState_t screenState, PowerModule &pm);
    void updateMainScreen(ScreenState_t screenState, PowerModule &pm, Sensors &gyro, int64_t encoderValue);

    void updateScreen(ScreenState_t screenState, Screen_t screen, PowerModule &pm, Sensors &gyro, int64_t encoderValue);

    // Timer selection
    TimerHandle_t timer;
    static void vTimerCallback( TimerHandle_t xTimer );
    
    // Event handler
    static void event_handler(lv_obj_t * obj, lv_event_t event);

    // Zmienne do przelaczania ekranow
    static Screen_t currentScreen, prevScreen, prevMenuScreen;
    Display display;


    static DiagScreen_t diag;
    static SelectionScreen_t selection;
    static ShutdownScreen_t shutdown;
    static MainScreen_t main;

};

#endif