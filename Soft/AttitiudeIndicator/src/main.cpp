#include <Arduino.h>
#include "PowerModule.hpp"
#include "Sensors.hpp"
#include "UI.hpp"
#include <ESP32Encoder.h>

// Definicje modułów
PowerModule pm;
Sensors sensors;
UI ui;
ESP32Encoder encoder;

/*
    Callback obsługujący zdarzenie przycisku enkodera
*/
void onButtonAction(ClickType_t ct){

  switch(ct){

    case HOLD:
      ui.setScreen(Screen_t::SCREEN_SELECTION); // Przełącz ekran na ekran wyboru (menu główne)
      break;

    case SHUTDOWN :
      ui.setScreen(Screen_t::SCREEN_SHUTDOWN);  // Przełącz ekran na ekran wyłączenia
      break;

  }

}

void setup() {

  Serial.begin(9600);
  Serial.println("Starting . . .");

  sensors.begin();

  pm.begin(GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_34, GPIO_NUM_35);

  encoder.attachSingleEdge ( GPIO_NUM_25, GPIO_NUM_26 );
  encoder.useInternalWeakPullResistors = UP;
  encoder.setCount (0);
  encoder.setFilter(1000);

  ui.begin();
  pm.setPowerClickListener(onButtonAction);
}

// Aktualizacje modułów w wątku głównym (M.T.)
void loop() {
  sensors.loop();
  pm.loop();
  ui.loop(encoder.getCount(), pm, sensors);
}

