#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Adafruit_MPU6050.h>
#include <DFRobot_QMC5883.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Fusion/Fusion.h>

typedef struct{
    int8_t pitch;
    int8_t roll;
    uint16_t hdg;
}AttitiudeInfo_t;

class Sensors{

public :

    void begin();
    void loop();    // Synchronizacja zmiennych

    // Zmienne używane tylko w wątku głównym
    static AttitiudeInfo_t attitiude;
    static uint32_t qnh_hpa;   // QNH

    static float temperature;
    static float pressureHPa;
    static float altitude_ft;
    static float vertical_speed_fpm;

private:

    void initMPU6050();
    void initCompas();
    void initBme();
    
    // Kopia zmiennych tylko dla wątku czujników (sensor thread, st)
    static AttitiudeInfo_t st_attitiude;    // Położenie przestrzenne
    static uint32_t st_qnh_hpa;             // QNH
    static float st_temperature;
    static float st_pressureHPa;
    static float st_altitude_ft;
    static float st_vertical_speed_fpm;

    static Adafruit_BMP280 bmp;
    static Adafruit_MPU6050 mpu;
    static DFRobot_QMC5883 compass;

    static Adafruit_Sensor *bmp_temp;
    static Adafruit_Sensor *bmp_pressure;

    static FusionAhrs ahrs;

    static void sensors_module_task(void* arg);

    // Debugowanie
    void wireScan();
    void simulateAttitiudeChanges();
    void simulateAltitiudeChanges();

    // Semafor do synchronizacji zmiennych pomiędzy wątkami
    static SemaphoreHandle_t semaphore;

};

#endif