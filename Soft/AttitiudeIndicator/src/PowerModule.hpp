#ifndef POWER_MODULE_HPP
#define POWER_MODULE_HPP

#include "driver/gpio.h"
#include <inttypes.h>

// Rodzaj kliknięcia
typedef enum{
    CLICK = 1,              // Kliknięcie w przycisk
    HOLD = 2,               // Przytrzymanie
    SHUTDOWN = 3            // Przytrzymanie do wyłączenia
} ClickType_t;

// Stan przycisku zasilania
typedef enum{
    RELEASED = 0,          
    PRESSED = 1          
} ButtonState_t;

// Stan ładowarki LiIon
typedef enum {
    BATTERY_CHARGING = 1,       // Ładowanie
    BATTERY_FULL = 2,           // Bateria naładowana
    BATTERY_ERROR = 3           // Błąd ładowania
} ChargingState_t;

typedef void (*ClickListener_t)(ClickType_t type);

/*
Moduł odpowiedzialny za : 
    - przycisk zasilania 'PowerButton' (wykrywanie przytrzymania i kliknięcia)
    - Wyłączanie urządzena pinem 'PowerButton'
    - Pomiar napięcia LiIon (aktualizowany co 10s)
    -
*/

class PowerModule{

public :
    void begin (gpio_num_t power_control, gpio_num_t adc_battery, gpio_num_t tp_stdby, gpio_num_t tp_chrg);   // Inicjalizacja

    void setPowerClickListener (ClickListener_t cbPowerClick){PowerModule::cbPowerClick = cbPowerClick;}; 
    
    void turnOff (void);

    void loop (void);   // Wykonywanie w głównym wątku

    void setAdcCoeff(uint32_t a, uint32_t b){batteryVoltage_coeffA=a;batteryVoltage_coeffB=b;};

    ChargingState_t readChargingState();   // Odczytuje stan ładowania

    uint32_t        batteryVoltage_mV;  // Odswiezane w głównym wątku
    ButtonState_t   buttonState;        // Odswiezane w głównym wątku

private:

    // kalibracja odczytu napiecia
    static uint32_t batteryVoltage_coeffA;    // wsp. A (wartość w 1/1000 całości [mV])
    static uint32_t batteryVoltage_coeffB;    // wsp. B (wartość w 1/1000 całości [mV])

    // wykrycie klikniecia
    static ClickListener_t cbPowerClick;
    
    // używane Piny GPIO
    static gpio_num_t power_control_gpio;
    static gpio_num_t adc_battery_gpio;
    static gpio_num_t tp_stdby_gpio;
    static gpio_num_t tp_chrg_gpio;

    static QueueHandle_t power_module_queue;

    static void power_module_task(void* arg);

    void adc_init();
    void gpio_init();

    static uint32_t adc_read();

};

#endif