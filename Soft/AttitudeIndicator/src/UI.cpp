#include "UI.hpp"

#define SELECTION_MENU_TIMEOUT_MS 5000

#define QNH_HPA_MIN 950
#define QNH_HPA_MAX 1050

Screen_t UI::currentScreen;
Screen_t UI::prevScreen;
Screen_t UI::prevMenuScreen;

DiagScreen_t            UI::diag;
SelectionScreen_t       UI::selection;
ShutdownScreen_t        UI::shutdown;
MainScreen_t            UI::main;

void    UI::begin(){

    display.begin();

    timer = xTimerCreate("selectionTimer", pdMS_TO_TICKS( SELECTION_MENU_TIMEOUT_MS ), pdFALSE, (void*) 1, UI::vTimerCallback);

    // Tworzymy ekrany i kontrolki
    createMainScreen();
    createDiagnosticScreen();
    createSelectionScreen();
    createShutdownScreen();

    // Przełączamy ekran
    prevScreen = Screen_t::SCREEN_NULL;
    currentScreen = prevMenuScreen = Screen_t::SCREEN_MAIN;
}

void UI::loop(int64_t encoderValue, PowerModule &pm, Sensors &sensors){

    if(currentScreen != prevScreen){    // Zmiana ekranu
        updateScreen(ScreenState_t::EXIT, prevScreen, pm, sensors, encoderValue);
        updateScreen(ScreenState_t::ENTRY, currentScreen, pm, sensors, encoderValue);
        prevScreen = currentScreen;

        if( (currentScreen != Screen_t::SCREEN_SELECTION) && (currentScreen != Screen_t::SCREEN_SHUTDOWN) )
            prevMenuScreen = currentScreen;

    }else{
        updateScreen(ScreenState_t::UPDATE, currentScreen, pm, sensors, encoderValue);
    }

    display.loop(encoderValue, pm.buttonState);
}

void UI::updateScreen(ScreenState_t screenState,
    Screen_t screen,
    PowerModule &pm,
    Sensors &sensors,
    int64_t encoderValue){

    switch(screen){
        case Screen_t::SCREEN_DIAGNOSTIC:
            updateDiagnosticScreen(screenState, sensors.attitiude);
        break;

        case Screen_t::SCREEN_MAIN:
            updateMainScreen(screenState, pm, sensors, encoderValue);
        break;

        case Screen_t::SCREEN_SELECTION:
            updateSelectionScreen(screenState, pm.buttonState);
        break;

        case Screen_t::SCREEN_SHUTDOWN:
            updateShutdownScreen(screenState, pm);
        break;
    }
}

void UI::vTimerCallback( TimerHandle_t xTimer ){
    currentScreen = prevMenuScreen; // Wyjście z menu głównego
}

void UI::event_handler(lv_obj_t * obj, lv_event_t event)
{
    switch (event)
    {
    case LV_EVENT_CLICKED:
        
        if(obj == selection.ui_ButtonMain){
            setScreen(Screen_t::SCREEN_MAIN);
        }

        else if(obj == selection.ui_ButtonDiag){
            setScreen(Screen_t::SCREEN_DIAGNOSTIC); 
        }

        break;
    
    default:
        break;
    }
}

void UI::createMainScreen(){

    // ui_ScreenMain
    main.ui_Screen = lv_cont_create(NULL, NULL);

    static lv_style_t style_screen;
    lv_style_init(&style_screen);
    lv_style_set_bg_color(&style_screen, LV_STATE_DEFAULT, lv_color_hex(0x0a2fff));
    lv_obj_add_style(main.ui_Screen, LV_OBJ_PART_MAIN, &style_screen);  //turn the screen white

    // ui_BarBat

    main.ui_BarBat = lv_bar_create(main.ui_Screen, NULL);
    lv_bar_set_range(main.ui_BarBat, 0, 100);
    lv_bar_set_value(main.ui_BarBat, 25, LV_ANIM_OFF);

    lv_obj_set_width(main.ui_BarBat, 50);
    lv_obj_set_height(main.ui_BarBat, 15);

    lv_obj_align(main.ui_BarBat, NULL, LV_ALIGN_CENTER, 133, -112);
    lv_obj_set_style_local_bg_color(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, lv_color_hex(0xFF2300));
    lv_obj_set_style_local_bg_opa(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, 150);

    // ui_LabelBatStatus

    main.ui_LabelBatStatus = lv_label_create(main.ui_Screen, NULL);
    lv_obj_align(main.ui_LabelBatStatus, NULL, LV_ALIGN_CENTER, 135, -95);
    lv_label_set_text(main.ui_LabelBatStatus, "CHRG");
    lv_obj_set_style_local_text_font(main.ui_LabelBatStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_obj_set_style_local_text_color(main.ui_LabelBatStatus, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));


    // ui_LabelBatVoltage

    main.ui_LabelBatVoltage = lv_label_create(main.ui_Screen, NULL);
    lv_obj_align(main.ui_LabelBatVoltage, NULL, LV_ALIGN_CENTER, 138, -110);
    lv_label_set_text(main.ui_LabelBatVoltage, "3.6V");
    lv_obj_set_style_local_text_font(main.ui_LabelBatVoltage, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);


    // ui_LabelAltitiude

    main.ui_LabelAltitiude = lv_label_create(main.ui_Screen, NULL);
    lv_label_set_text(main.ui_LabelAltitiude, "30000ft");
    lv_obj_align(main.ui_LabelAltitiude, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);
    lv_obj_set_style_local_text_color(main.ui_LabelAltitiude, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));

    // ui_LabelFpm

    main.ui_LabelFpm = lv_label_create(main.ui_Screen, NULL);
    lv_label_set_text(main.ui_LabelFpm, "200");
    lv_obj_set_style_local_text_font(main.ui_LabelFpm, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_obj_align(main.ui_LabelFpm, NULL, LV_ALIGN_IN_RIGHT_MID, -5, -50);
    lv_obj_set_style_local_text_color(main.ui_LabelFpm, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));

    // ui_LabelQnh

    main.ui_LabelQnh = lv_label_create(main.ui_Screen, NULL);
    lv_label_set_text(main.ui_LabelQnh, "1025hPa");
    lv_obj_align(main.ui_LabelQnh, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_local_text_color(main.ui_LabelQnh, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));

    // ui_LabelHdg
    main.ui_LabelHdg = lv_label_create(main.ui_Screen, NULL);
    lv_label_set_text(main.ui_LabelHdg, "290");
    lv_obj_align(main.ui_LabelHdg, NULL, LV_ALIGN_IN_TOP_MID, 10, 0);
    lv_obj_set_style_local_text_font(main.ui_LabelHdg, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_24);
    lv_obj_set_style_local_text_color(main.ui_LabelHdg, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));


    main.ui_Om = lv_objmask_create(main.ui_Screen, NULL);
    lv_obj_set_size(main.ui_Om, 320, 240);
    lv_obj_align(main.ui_Om, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);

    lv_draw_mask_line_param_t drawMask;
    lv_draw_mask_line_angle_init(&drawMask, 170, 120, 30, LV_DRAW_MASK_LINE_SIDE_BOTTOM);
    main.ui_ObjMask = lv_objmask_add_mask(main.ui_Om, &drawMask);

    static lv_obj_t * cont = lv_cont_create(main.ui_Om, NULL);
    lv_obj_set_size(cont, 340, 260);
    lv_obj_align(cont, NULL, LV_ALIGN_IN_BOTTOM_LEFT, -10, 10);
    lv_obj_set_style_local_bg_color(cont, LV_CONT_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0x855225));
    lv_obj_move_background(main.ui_Om);


    //ui_LabelMid
    main.ui_LabelMid = lv_label_create(main.ui_Screen, NULL);
    lv_label_set_text(main.ui_LabelMid, "-----^-----");
    lv_obj_align(main.ui_LabelMid, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_text_color(main.ui_LabelMid, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));

    //ui_LabelTemp
    main.ui_LabelTemp = lv_label_create(main.ui_Screen, NULL);
    lv_obj_align(main.ui_LabelTemp, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_label_set_text(main.ui_LabelTemp, "25.5C");
    lv_obj_set_style_local_text_font(main.ui_LabelTemp, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_14);
    lv_obj_set_style_local_text_color(main.ui_LabelTemp, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0xFFFFFF));


}

void UI::createShutdownScreen(){

    shutdown.ui_Screen = lv_cont_create(NULL, NULL);
    shutdown.ui_Label = lv_label_create(shutdown.ui_Screen, NULL);

    lv_obj_align(shutdown.ui_Label, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(shutdown.ui_Label, "Turning off. . .");

}

void UI::createSelectionScreen(){

    selection.ui_Screen = lv_cont_create(NULL, NULL);
    selection.ui_Label = lv_label_create(selection.ui_Screen, NULL);

    
    lv_label_set_text(selection.ui_Label, "Main Menu");
    lv_obj_align(selection.ui_Label, NULL, LV_ALIGN_CENTER, 0, -100);

    selection.ui_ButtonMain = lv_btn_create(selection.ui_Screen, NULL);
    selection.ui_ButtonDiag = lv_btn_create(selection.ui_Screen, NULL);

    lv_obj_set_width(selection.ui_ButtonMain, 150);
    lv_obj_set_width(selection.ui_ButtonDiag, 150);

    lv_obj_set_height(selection.ui_ButtonMain, 40);
    lv_obj_set_height(selection.ui_ButtonDiag, 40);

    lv_obj_t * label;
    label = lv_label_create(selection.ui_ButtonMain, NULL);
    lv_label_set_text(label, "Main");
    label = lv_label_create(selection.ui_ButtonDiag, NULL);
    lv_label_set_text(label, "Diag");

    lv_obj_align(selection.ui_ButtonMain, NULL, LV_ALIGN_CENTER, 0, -20);
    lv_obj_align(selection.ui_ButtonDiag, NULL, LV_ALIGN_CENTER, 0, 20);

    lv_obj_set_event_cb(selection.ui_ButtonMain, event_handler);
    lv_obj_set_event_cb(selection.ui_ButtonDiag, event_handler);

    selection.ui_Group = lv_group_create();
    lv_group_add_obj(selection.ui_Group, selection.ui_ButtonMain);
    lv_group_add_obj(selection.ui_Group, selection.ui_ButtonDiag);
    
}

void UI::createDiagnosticScreen(){

// Tworzymy obiekt ekranu
    diag.ui_Screen = lv_cont_create(NULL, NULL);

    diag.ui_SliderH = lv_slider_create(diag.ui_Screen, NULL);
    lv_slider_set_range(diag.ui_SliderH, -90, 90);
    lv_slider_set_type(diag.ui_SliderH, LV_SLIDER_TYPE_NORMAL);
    lv_obj_set_adv_hittest(diag.ui_SliderH, true);
    lv_obj_set_style_local_bg_opa(diag.ui_SliderH, LV_SLIDER_PART_INDIC, NULL, LV_OPA_TRANSP);
    lv_obj_set_width(diag.ui_SliderH, 200);
    lv_obj_set_height(diag.ui_SliderH, 12);
    lv_obj_align(diag.ui_SliderH, NULL, LV_ALIGN_CENTER, 56, 103);
    lv_obj_set_state(diag.ui_SliderH, LV_STATE_DISABLED);

    diag.ui_SliderV = lv_slider_create(diag.ui_Screen, NULL);
    lv_slider_set_range(diag.ui_SliderV, -90, 90);
    lv_slider_set_type(diag.ui_SliderV, LV_SLIDER_TYPE_NORMAL);
    lv_obj_set_adv_hittest(diag.ui_SliderV, true);
    lv_obj_set_style_local_bg_opa(diag.ui_SliderV, LV_SLIDER_PART_INDIC, NULL, LV_OPA_TRANSP);
    lv_obj_set_width(diag.ui_SliderV, 12);
    lv_obj_set_height(diag.ui_SliderV, 200);
    lv_obj_align(diag.ui_SliderV, NULL, LV_ALIGN_CENTER, -64, -10);
    lv_obj_set_state(diag.ui_SliderV, LV_STATE_DISABLED);

    diag.ui_Arc = lv_arc_create(diag.ui_Screen, NULL);
    lv_obj_set_width(diag.ui_Arc, 200);
    lv_obj_set_height(diag.ui_Arc, 200);
    lv_obj_set_style_local_bg_opa(diag.ui_SliderH, LV_SLIDER_PART_INDIC, NULL, LV_OPA_TRANSP);
    lv_obj_align(diag.ui_Arc, NULL, LV_ALIGN_CENTER, 55, -15);
    lv_arc_set_rotation(diag.ui_Arc, 270);
    lv_arc_set_bg_angles(diag.ui_Arc, 0, 360);
    lv_obj_set_style_local_bg_opa(diag.ui_Arc, LV_ARC_PART_INDIC, NULL, LV_OPA_TRANSP);
    lv_obj_set_state(diag.ui_Arc, LV_STATE_DISABLED);

    diag.ui_LabelHdg = lv_label_create(diag.ui_Screen, NULL);

    lv_obj_align(diag.ui_LabelHdg, NULL, LV_ALIGN_CENTER, 37, -20);
    lv_label_set_text(diag.ui_LabelHdg, "Heading \n270");


    diag.ui_LabelPitch = lv_label_create(diag.ui_Screen, NULL);

    lv_obj_align(diag.ui_LabelPitch, NULL, LV_ALIGN_CENTER, -135, -10);
    lv_label_set_text(diag.ui_LabelPitch, "Pitch: -15");


    diag.ui_LabelRoll = lv_label_create(diag.ui_Screen, NULL);

    lv_obj_align(diag.ui_LabelRoll, NULL, LV_ALIGN_CENTER, -117, 103);
    lv_label_set_text(diag.ui_LabelRoll, "Roll: +15");


    // diag.ui_SwitchCal = lv_switch_create(diag.ui_Screen, NULL);

    // lv_obj_set_width(diag.ui_SwitchCal, 50);
    // lv_obj_set_height(diag.ui_SwitchCal, 25);

    // lv_obj_align(diag.ui_SwitchCal, NULL, LV_ALIGN_CENTER, -114, -78);


    // diag.ui_LabelCal = lv_label_create(diag.ui_Screen, NULL);

    // lv_obj_align(diag.ui_LabelCal, NULL, LV_ALIGN_CENTER, -138, -101);
    // lv_label_set_text(diag.ui_LabelCal, "Kalibracja");

    selection.ui_Screen = lv_cont_create(NULL, NULL);
    selection.ui_Label = lv_label_create(selection.ui_Screen, NULL);

    lv_obj_align(selection.ui_Label, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(selection.ui_Label, "Selection");

}


void UI::updateDiagnosticScreen(ScreenState_t screenState, AttitiudeInfo_t attitiude){
    if(screenState == ENTRY)
        lv_scr_load(diag.ui_Screen);
    else if(screenState == UPDATE){
        lv_slider_set_value(diag.ui_SliderV, attitiude.pitch, LV_ANIM_OFF);
        //lv_slider_set_left_value(diag.ui_SliderV, attitiude.pitch, LV_ANIM_OFF);

        lv_slider_set_value(diag.ui_SliderH, attitiude.roll, LV_ANIM_OFF);
        //lv_slider_set_left_value(diag.ui_SliderH, attitiude.roll, LV_ANIM_OFF);

        lv_arc_set_angles(diag.ui_Arc, attitiude.hdg, attitiude.hdg +1);


        static char cRoll[10];
        snprintf(cRoll, 10, "Roll: %+d", attitiude.roll);
        lv_label_set_text(diag.ui_LabelRoll, cRoll);

        static char cPitch[11];
        snprintf(cPitch, 11, "Pitch: %+d", attitiude.pitch);
        lv_label_set_text(diag.ui_LabelPitch, cPitch);

        static char cHeading[20];
        snprintf(cHeading, 20, "Heading\n\t  %.3u", attitiude.hdg);
        lv_label_set_text(diag.ui_LabelHdg, cHeading);
    }
}

void UI::updateSelectionScreen(ScreenState_t screenState, ButtonState_t buttonState){
    
    static bool btnReleased;
    
    if(screenState == ENTRY){
        lv_scr_load(selection.ui_Screen);

        display.enable_button_state_change = false;
        lv_indev_enable(display.indev_encoder, false);

        lv_indev_set_group(display.indev_encoder, selection.ui_Group);
        btnReleased = false;
        xTimerStart(timer, 0);
    }
    else if(screenState == UPDATE){

        if (!btnReleased && buttonState == RELEASED){
            Serial.println("RELEASED");
            btnReleased = true;

            display.enable_button_state_change = true;
            lv_indev_reset_long_press(display.indev_encoder);
            lv_indev_reset(display.indev_encoder, NULL);
            lv_indev_enable(display.indev_encoder, true);
        }

    }
    else if(screenState == EXIT){
        lv_indev_set_group(display.indev_encoder, NULL);

        display.enable_button_state_change = false;
        lv_indev_enable(display.indev_encoder, false);
    }
}

void UI::updateMainScreen(ScreenState_t screenState, PowerModule &pm, Sensors &sensors, int64_t encoderValue){
    static int64_t prevEncoderValue = encoderValue;
    
    if(screenState == ENTRY){
        lv_scr_load(main.ui_Screen);
        prevEncoderValue = encoderValue;
    }
    else{

        // Zmiana QNH enkoderem
        
        int encoderDiff = encoderValue - prevEncoderValue;
        prevEncoderValue = encoderValue;

        sensors.qnh_hpa += encoderDiff;

        if(sensors.qnh_hpa < QNH_HPA_MIN)
            sensors.qnh_hpa = QNH_HPA_MIN;

        if(sensors.qnh_hpa > QNH_HPA_MAX)
            sensors.qnh_hpa = QNH_HPA_MAX;


        // (Label) Stan ładowarki Li-Ion
        switch (pm.readChargingState()){
            case ChargingState_t::BATTERY_CHARGING:
                lv_label_set_text(main.ui_LabelBatStatus, "CHRG");
            break;
            case ChargingState_t::BATTERY_FULL:
                lv_label_set_text(main.ui_LabelBatStatus, "FULL");
            break;
            case ChargingState_t::BATTERY_ERROR:
                lv_label_set_text(main.ui_LabelBatStatus, "DISC");
            break;
        }
        lv_obj_align(main.ui_LabelBatStatus, NULL, LV_ALIGN_CENTER, 135, -95);

        // (Label) Napięcie Li-Ion
        static char cVoltage[5];
        snprintf(cVoltage, 5, "%d.%dV", (pm.batteryVoltage_mV / 1000), ((pm.batteryVoltage_mV % 1000) / 100));
        lv_label_set_text(main.ui_LabelBatVoltage, cVoltage);

        // (Bar)
        long percent = map(pm.batteryVoltage_mV, 2700, 4200, 0, 100);
        lv_bar_set_value(main.ui_BarBat, percent, LV_ANIM_OFF);

        // Kolor w zaleznosci od poziomu naladowania baterii
        if (percent < 25)
            lv_obj_set_style_local_bg_color(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, lv_color_hex(0xFF0000));
        else if (percent < 75)
            lv_obj_set_style_local_bg_color(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, lv_color_hex(0xFFFF00));
        else
            lv_obj_set_style_local_bg_color(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, lv_color_hex(0x00FF00));

        lv_obj_set_style_local_bg_opa(main.ui_BarBat, LV_BAR_PART_INDIC, LV_STATE_DEFAULT, 150);


        // Wysokość
        static char cAlt[16];
        snprintf(cAlt, 16, "%.0fft", sensors.altitude_ft);
        lv_label_set_text(main.ui_LabelAltitiude, cAlt);
        lv_obj_align(main.ui_LabelAltitiude, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

        // Wznoszenie/Opadanie

        int32_t fpm = (((int32_t) round(sensors.vertical_speed_fpm)) * 10) / 10;

        static char cFpm[16];
        snprintf(cFpm, 16, "%d", fpm);
        lv_label_set_text(main.ui_LabelFpm, cFpm);
        
        if (fpm > 10){     // Wznoszenie
            lv_obj_set_hidden(main.ui_LabelFpm, false);

            if (fpm >= 500)
                lv_obj_align(main.ui_LabelFpm, NULL, LV_ALIGN_IN_RIGHT_MID, -5, -70);
            else
                lv_obj_align(main.ui_LabelFpm, NULL, LV_ALIGN_IN_RIGHT_MID, -5, map((fpm/10)*10, 10, 500, -20, -70));
        }
        else if (fpm < -10){    // Opadanie
            lv_obj_set_hidden(main.ui_LabelFpm, false);
            
            if (fpm <= -500)
                lv_obj_align(main.ui_LabelFpm, NULL, LV_ALIGN_IN_RIGHT_MID, -5, 70);
            else
                lv_obj_align(main.ui_LabelFpm, NULL, LV_ALIGN_IN_RIGHT_MID, -5, map(fpm, -10, -500, 20, 70));

        }else{
            lv_obj_set_hidden(main.ui_LabelFpm, true);
        }
        
        // cisnienie ustawinone (QNH)
        static char cQnh[16];
        snprintf(cQnh, 16, "%dhPa", sensors.qnh_hpa);
        lv_label_set_text(main.ui_LabelQnh, cQnh);
        lv_obj_set_style_local_bg_color(main.ui_LabelQnh, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(0x00c8ff));    
        lv_obj_align(main.ui_LabelQnh, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);

        // HDG
        static char cHdg[16];
        snprintf(cHdg, 16, "%03d", sensors.attitiude.hdg);
        lv_label_set_text(main.ui_LabelHdg, cHdg);
        lv_obj_align(main.ui_LabelHdg, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);

        // Temp
        static char cTemp[16];
        snprintf(cTemp, 16, "%.1fC", sensors.temperature);
        lv_label_set_text(main.ui_LabelTemp, cTemp);
        lv_obj_align(main.ui_LabelTemp, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);


        // Tło
        lv_draw_mask_line_param_t drawMask;
        lv_draw_mask_line_angle_init(&drawMask, 170, 120 + map(sensors.attitiude.pitch, -30, 30, -70, 70), sensors.attitiude.roll, LV_DRAW_MASK_LINE_SIDE_BOTTOM);
        lv_objmask_update_mask(main.ui_Om, main.ui_ObjMask, &drawMask);

    }
}

void UI::updateShutdownScreen(ScreenState_t screenState, PowerModule &pm){
    if(screenState == ENTRY)
        lv_scr_load(shutdown.ui_Screen);
    else{
        pm.turnOff();
    }
}