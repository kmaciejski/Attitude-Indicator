#include <Arduino.h>
#include "PowerModule.hpp"
#include <esp_intr_alloc.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "include/soc/adc_channel.h"


#define HOLD_TIME_MS        500
#define SHUTDOWN_TIME_MS    1000

gpio_num_t PowerModule::power_control_gpio  = GPIO_NUM_NC;
gpio_num_t PowerModule::adc_battery_gpio    = GPIO_NUM_NC;
gpio_num_t PowerModule::tp_stdby_gpio       = GPIO_NUM_NC;
gpio_num_t PowerModule::tp_chrg_gpio        = GPIO_NUM_NC;

uint32_t PowerModule::batteryVoltage_coeffA;    // wsp. A (wartość w 1/1000 całości [mV])
uint32_t PowerModule::batteryVoltage_coeffB;    // wsp. B (wartość w 1/1000 całości [mV])

ClickListener_t PowerModule::cbPowerClick   = NULL;
QueueHandle_t PowerModule::power_module_queue         = NULL;

static esp_adc_cal_characteristics_t * adc_chars;

static const int DEFAULT_VREF    = 1100;        
static const int NO_OF_SAMPLES   = 64;          // Multisampling

static const adc1_channel_t channel     = ADC1_GPIO32_CHANNEL;
static const adc_bits_width_t width     = ADC_WIDTH_BIT_12;
static const adc_atten_t atten          = ADC_ATTEN_DB_11;

// Rodzaj elementu kolejki sluzacej do komunikacji pomiedzy zadaniem P.M. a watkiem glownym
typedef enum{
    ITEM_BATTERY_VOLTAGE,
    ITEM_BUTTON_ACTION,
    ITEM_BUTTON_STATE_CHANGE
}QueueItemType_t;

// Format elementu kolejki
typedef struct{
    QueueItemType_t action;
    union{
        uint32_t        adc;
        ClickType_t     clickType;
        ButtonState_t   buttonState;
    } data;
}QueueItem_t;


/*
    Zadanie P.M.
        - odczytuje napiecie baterii poprzez ADC
        - debouncing i wykrywanie stanu przycisku
*/
void PowerModule::power_module_task(void* arg)
{
    bool prevState = gpio_get_level(power_control_gpio);    // stan przycisku sprzed poprzedniego cyklu
    uint32_t cntHold = 0;           // Licznik cykli 20ms dla przycisku, gdy = 0 to wtedy zatrzymany
    uint32_t cntShutdown = 0;       // Licznik cykli 20ms dla przycisku, gdy = 0 to wtedy zatrzymany
    
    uint32_t cntAdc = 0;            // Licznik cykli 20ms dla adc

    QueueItem_t item;       // Element kolejki

    for(;;) {

        bool state = gpio_get_level(power_control_gpio);

        if (state != prevState){    // Wykryto zmiane stanu

            item.action = ITEM_BUTTON_STATE_CHANGE;
            item.data.buttonState = (state) ? ButtonState_t::RELEASED : ButtonState_t::PRESSED;
            xQueueSend(power_module_queue, &item, portMAX_DELAY);   // Wysyłamy do M.T. inormacje o zmianie stanu

            if (state){ // przycisk teraz puszczony
                
                if (cntHold){    // Wykryto klikniecie
                    item.action = ITEM_BUTTON_ACTION;
                    item.data.clickType = CLICK;
                    xQueueSend(power_module_queue, &item, portMAX_DELAY);
                }

                cntHold = 0;
                cntShutdown = 0;
            }else{      // przycisk teraz wciśnięty
                cntHold = 1;
                cntShutdown = 1;
            }

        }

        if (cntHold > (HOLD_TIME_MS / 20)){         // Spr. czy dluzej niz HOLD_TIME_MS
            
            if (!state){    // Wykryto przytrzymanie
                item.action = ITEM_BUTTON_ACTION;
                item.data.clickType = HOLD;
                xQueueSend(power_module_queue, &item, portMAX_DELAY);
            }
            cntHold = 0;
        }

        if (cntShutdown > (SHUTDOWN_TIME_MS / 20)){ // Spr. czy dluzej niz SHUTDOWN_TIME_MS
            
            if (!state){    // Wykryto przytrzymanie
                item.action = ITEM_BUTTON_ACTION;
                item.data.clickType = SHUTDOWN;
                xQueueSend(power_module_queue, &item, portMAX_DELAY);
            }
            cntShutdown = 0;
        }

        prevState = state;

        if(cntHold)             // Odliczaj gdy cnt != 0
            cntHold++;

        if(cntShutdown)         // Odliczaj gdy cntShutdown != 0
            cntShutdown++;

        // Pomiar ADC
        cntAdc++;
        if(cntAdc > (1000 / 20)){   // pomiar co sekunde
            cntAdc = 0;

            item.action = ITEM_BATTERY_VOLTAGE;
            item.data.adc = adc_read();
            xQueueSend(power_module_queue, &item, portMAX_DELAY);
        }

        vTaskDelay(20/portTICK_PERIOD_MS);  // Próbkowanie stanu pinu co 20ms
    }
}

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

// Odczyt ADC
uint32_t PowerModule::adc_read(){

    //Multisampling
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) 
        adc_reading += adc1_get_raw((adc1_channel_t)channel);    
    
    adc_reading /= NO_OF_SAMPLES;

    uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    //Convert adc_reading to voltage in mV
    return ( ( (voltage_mV * batteryVoltage_coeffA) / 1000 ) + batteryVoltage_coeffB );
}

// Inicjalizacja ADC
void PowerModule::adc_init(){

    check_efuse();

    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t *) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    // Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

}

// Inicjalizacja GPIO
void PowerModule::gpio_init(){

// tp_chrg_gpio & tp_stdby_gpio

    //zero-initialize the config structure.
    gpio_config_t tp_io_conf = {};
    //disable interrupt
    tp_io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    tp_io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins
    tp_io_conf.pin_bit_mask = (1ULL << tp_chrg_gpio) | (1ULL << tp_stdby_gpio);
    //disable pull-down mode
    tp_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    tp_io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&tp_io_conf);

// power_control_gpio
    gpio_config_t pc_io_conf = {};
    //disable interrupt
    pc_io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    pc_io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    pc_io_conf.pin_bit_mask = (1ULL << power_control_gpio);
    //disable pull-down mode
    pc_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    pc_io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&pc_io_conf);

    gpio_set_level(power_control_gpio, HIGH);

}

/*
    Inicjalizacja
*/
void PowerModule::begin(gpio_num_t power_control, gpio_num_t adc_battery, gpio_num_t tp_stdby, gpio_num_t tp_chrg){

    power_control_gpio = power_control;
    adc_battery_gpio = adc_battery;
    tp_chrg_gpio = tp_chrg;
    tp_stdby_gpio = tp_stdby;

    batteryVoltage_coeffA = 2000;
    batteryVoltage_coeffB = 0;

    gpio_init();
    adc_init();

    //create a queue
    power_module_queue = xQueueCreate(32, sizeof(QueueItem_t));
    
    //start task
    xTaskCreate(power_module_task, "power_module_task", 2048, NULL, 10, NULL);

    Serial.println("PowerModule init OK.");
}

/*
    Odczyt stanu ładowania
*/
ChargingState_t PowerModule::readChargingState(){

    bool stdby = gpio_get_level(tp_stdby_gpio);
    bool chrg = gpio_get_level(tp_chrg_gpio);

    if (!stdby && chrg)
        return BATTERY_FULL;

    else if (stdby && !chrg)
        return BATTERY_CHARGING;

    else
        return BATTERY_ERROR;

}

/*
    execute in MainThread
*/
void PowerModule::loop(void){

    QueueItem_t queueItem;

    while(xQueueReceive(power_module_queue, &queueItem, 0) == pdTRUE) {

        switch (queueItem.action){

            case ITEM_BUTTON_STATE_CHANGE:  // Zadanie P.M. wysyła zmiane stanu przycisku
                this->buttonState = queueItem.data.buttonState;
            break;

            case ITEM_BUTTON_ACTION:        // Zadanie P.M. wysyła akcje przycisniecia przycisku
                Serial.println(queueItem.data.clickType);
                if( this->cbPowerClick )
                    this->cbPowerClick(queueItem.data.clickType);
            break;

            case ITEM_BATTERY_VOLTAGE:      // Zadanie P.M. wysyła pomiar baterii
                this->batteryVoltage_mV = queueItem.data.adc;   // aktualizacja napiecia
            break;

        }

    }
}

/*
    Wyłączanie urządzenia
*/
void PowerModule::turnOff (){

    gpio_set_level(power_control_gpio, LOW);
    while(1)
        delay(1000);

}
