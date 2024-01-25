#include "Display.hpp"
#include "freertos/atomic.h"

TFT_eSPI            Display::tft = TFT_eSPI();
lv_disp_buf_t       Display::disp_buf;
lv_color_t          Display::buf1[LV_HOR_RES_MAX * 30];
lv_color_t          Display::buf2[LV_HOR_RES_MAX * 30];

int64_t             Display::encoderCnt;
int64_t             Display::encoderPrevCnt;

ButtonState_t       Display::buttonState;
bool                Display::enable_button_state_change;

void Display::begin(){

    Serial.begin(9600);
    Serial.println("ILI9341 Test!"); 
 
    // Backlight ON
    pinMode(GPIO_NUM_13, OUTPUT);
    digitalWrite(GPIO_NUM_13, HIGH);

    lv_init();

    tft.init();
    tft.setRotation(3);           // Rotacja = 3
    tft.fillScreen(TFT_BLUE);

    lv_disp_buf_init(&disp_buf, buf1, buf2, LV_HOR_RES_MAX * 30);

    /*Initialize the display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = this->disp_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_t indev;
    lv_indev_drv_init(&indev);             /*Descriptor of a input device driver*/
    indev.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev.read_cb = this->touchpad_read;      /*Set your driver function*/
    indev_touch = lv_indev_drv_register(&indev);         /*Finally register the driver*/

    lv_indev_drv_init(&indev);             /*Descriptor of a input device driver*/
    indev.type = LV_INDEV_TYPE_ENCODER;
    indev.read_cb = this->encoder_read;
    indev_encoder = lv_indev_drv_register(&indev);         /*Finally register the driver*/

    encoderCnt = encoderPrevCnt = 0;
    enable_button_state_change = false;
}

void Display::loop(int64_t encoderAbsCnt, ButtonState_t state){
    encoderCnt = encoderAbsCnt;
    buttonState = state;

    lv_task_handler();
}

/*
    Implementacja sterownika ekranu
*/
void Display::disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) 
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*
    Implementacja sterownika panelu dotykowego
*/
bool Display::touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
	uint16_t touchX, touchY;
	bool touched = tft.getTouch(&touchX, &touchY);  //  bool TFT_eTouch<T>::getXY(int16_t& x, int16_t& y)

    if(!touched)
      return false;

    touchX = screenWidth - touchX;      // Rotacja = 3
    touchY = screenHeight - touchY;     // Rotacja = 3

    if(touchX>screenWidth || touchY > screenHeight)
    {
    //   Serial.println("Y or y outside of expected parameters..");
    //   Serial.print("y:");
    //   Serial.print(touchX);
    //   Serial.print(" x:");
    //   Serial.print(touchY);
    }
    else
    {
		
      data->state = touched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL; 
  
      /*Save the state and save the pressed coordinate*/
      //if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&last_x, &last_y);
     
      /*Set the coordinates (if released use the last pressed coordinates)*/
      data->point.x = touchX;
      data->point.y = touchY;
  
    //   Serial.print("Data x");
    //   Serial.println(touchX);
      
    //   Serial.print("Data y");
    //   Serial.println(touchY);

    }

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

/*
    Implementacja sterownika enkodera
*/
int16_t Display::getEncoderDiff(){
    ATOMIC_ENTER_CRITICAL();
    int64_t diff = encoderCnt - encoderPrevCnt;
    encoderPrevCnt = encoderCnt;
    ATOMIC_EXIT_CRITICAL();
    return (int16_t)(diff & 0xFFFF);
}

bool Display::encoder_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data){

    data->enc_diff = getEncoderDiff();

    if(enable_button_state_change){
        if(buttonState) 
            data->state = LV_INDEV_STATE_PR;
        else 
            data->state = LV_INDEV_STATE_REL;
    }
    return false; /*No buffering now so no more data read*/

}

void Display::printEvent(String Event, lv_event_t event)
{
  
  Serial.print(Event);
  Serial.printf(" ");

  switch(event) {
      case LV_EVENT_PRESSED:
          Serial.printf("Pressed\n");
          break;

      case LV_EVENT_SHORT_CLICKED:
          Serial.printf("Short clicked\n");
          break;

      case LV_EVENT_CLICKED:
          Serial.printf("Clicked\n");
          break;

      case LV_EVENT_LONG_PRESSED:
          Serial.printf("Long press\n");
          break;

      case LV_EVENT_LONG_PRESSED_REPEAT:
          Serial.printf("Long press repeat\n");
          break;

      case LV_EVENT_RELEASED:
          Serial.printf("Released\n");
          break;
  }
}