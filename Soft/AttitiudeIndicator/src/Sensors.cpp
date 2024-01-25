#include "Sensors.hpp"
#include <Arduino.h>
#include <Wire.h>

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

FusionAhrs        Sensors::ahrs;
Adafruit_MPU6050  Sensors::mpu;
Adafruit_BMP280   Sensors::bmp(&Wire);
DFRobot_QMC5883   Sensors::compass(&Wire, QMC5883_ADDRESS);
SemaphoreHandle_t Sensors::semaphore;
Adafruit_Sensor *Sensors::bmp_temp      = bmp.getTemperatureSensor();
Adafruit_Sensor *Sensors::bmp_pressure  = bmp.getPressureSensor();

AttitiudeInfo_t   Sensors::attitiude{0};
AttitiudeInfo_t   Sensors::st_attitiude{0};

uint32_t Sensors::qnh_hpa;   // QNH
float Sensors::temperature = 25.0f;
float Sensors::pressureHPa = 1013.25f;
float Sensors::altitude_ft = 400.0f;
float Sensors::vertical_speed_fpm = 0.0f;

uint32_t Sensors::st_qnh_hpa;   // QNH
float Sensors::st_temperature = 25.0f;
float Sensors::st_pressureHPa = 1013.25f;
float Sensors::st_vertical_speed_fpm = 0.0f;
float Sensors::st_altitude_ft = 400.0f;

/*
  Funkcja obliczająca i filtrująca prędkość pionową na podstawie zmian ciśnienia w czasie
*/
float calculateNFilterFpm(float pressure_hPa){

  static float prev_fpm = 0.0;                            // Poprzednia wartość wznoszenia [ft/min]
  static float prev_pressure_hPa = 1013.25f;              // Poprzednia wartość ciśnienia statycznego [hPa]
  static volatile unsigned long prev_millis = millis();   // Poprzedni czas od uruchomienia [ms]

  unsigned volatile long curr_millis = millis();                          // Aktualny czas od uruchomienia [ms]
  unsigned volatile long delta_ms = curr_millis - prev_millis;            // delta t [ms]
  
  if (delta_ms >= 1000){ // 1s sampling

    // Przeliczamy wartości ciśnienia statycznego na wysokości w stopach [ft]
    float prev_abs_altitude_ft = ( (44330 * 3.280839895) * (1.0 - pow(prev_pressure_hPa / 1013.25f, 0.1903f)) );
    float abs_altitude_ft = ( (44330 * 3.280839895) * (1.0 - pow(pressure_hPa / 1013.25f, 0.1903f)) );

    float delta_altitude_ft = abs_altitude_ft - prev_abs_altitude_ft;     // delta H [ft]
    float fpm = (delta_altitude_ft * 1000.0f * 60.0f) / (float)delta_ms;  // obliczanie fpm [ft/min]
  
    // Próg 100 ft/min (poniżej próbki są odrzucane i predkosc pionowa wynosi 0)
    if (fpm > 100.0f || fpm < -100.0f)
      prev_fpm = fpm;
    else
      prev_fpm = 0.0f;

    prev_pressure_hPa = pressure_hPa;   // Aktualizujemy poprzednią wartość ciśnienia
    prev_millis = curr_millis;          // Aktualizujemy poprzedni czas od uruchomienia
  }

  return prev_fpm;    // Zwracamy obliczone wznoszenie [ft/min]
}

/*
    Zadanie S.M.
        - odczytuje napiecie baterii poprzez ADC
        - debouncing i wykrywanie stanu przycisku
*/
void Sensors::sensors_module_task(void* arg)
{

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 75 / portTICK_PERIOD_MS;
  BaseType_t xWasDelayed;
  xLastWakeTime = xTaskGetTickCount ();

  unsigned long millisPrev = millis();
  unsigned long delta = 0;
  sensors_event_t a, g, mpu_temp, bmp_temp_event, bmp_pressure_event;

  while(1){

    // Odczyt z 3 czujników IMU :
    mpu.getEvent(&a, &g, &mpu_temp);
    sVector_t mag = compass.readRaw();

    // Obliczanie położenia przestrzennego
    const FusionVector gyroscope = {g.gyro.x, g.gyro.y, g.gyro.z};
    const FusionVector accelerometer = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    const FusionVector magnetometer = {mag.XAxis, mag.YAxis, mag.ZAxis};

    delta = millis() - millisPrev;
    millisPrev = millis();

    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, ((float)delta) / 1000);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    bmp_temp->getEvent(&bmp_temp_event);
    bmp_pressure->getEvent(&bmp_pressure_event);

    if ( xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE ){  // Aktualizacja zmiennych

      st_attitiude.pitch   = std::roundf(euler.angle.pitch);
      st_attitiude.roll    = std::roundf(euler.angle.roll);
      st_attitiude.hdg     = std::roundf(euler.angle.yaw + 180.0f);
      
      st_temperature = bmp_temp_event.temperature;
      st_pressureHPa = bmp_pressure_event.pressure;
      st_altitude_ft = (44330 * 3.280839895) * (1.0 - pow(st_pressureHPa / st_qnh_hpa, 0.1903));
      st_vertical_speed_fpm = calculateNFilterFpm(st_pressureHPa);

      xSemaphoreGive(semaphore);
    }
    
    xTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void Sensors::begin(){

  qnh_hpa = 1025;

  Wire.begin(21, 22, 10000);

 // wireScan();

  initBme();
  initMPU6050();
  initCompas();

  FusionAhrsInitialise(&ahrs);

  semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(semaphore);

  //start task
  xTaskCreate(sensors_module_task, "sensors_module_task", 2048, NULL, 20, NULL);

}

void Sensors::loop(){

  if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE){
    memcpy(&attitiude, &st_attitiude, sizeof(AttitiudeInfo_t));

    temperature = st_temperature;                 // Temperatura
    pressureHPa = st_pressureHPa;                 // Ciśnienie BMP
    altitude_ft = st_altitude_ft;                 // Wysokość z uwzględnieniem QNH
    vertical_speed_fpm = st_vertical_speed_fpm;   // Prędkość pionowa
    st_qnh_hpa = qnh_hpa;                         // QNH

    xSemaphoreGive(semaphore);
  }

}

void Sensors::initMPU6050(){

if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }


}

void Sensors::initCompas(){

while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    while (1) delay(10);
  }

  if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_8GA);
    Serial.print("compass range is:");
    Serial.println(compass.getRange());

    compass.setMeasurementMode(QMC5883_CONTINOUS);
    Serial.print("compass measurement mode is:");
    Serial.println(compass.getMeasurementMode());

    compass.setSamples(QMC5883_SAMPLES_8);
    Serial.print("compass samples is:");
    Serial.println(compass.getSamples());
  }

}

void Sensors::initBme(){

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,       /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,       /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,      /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,        /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);     /* Standby time. */

  bmp_temp->printSensorDetails();

}

/*
        Funkcje wykorzystywane przy debugowaniu
*/
void Sensors::wireScan(){
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}

void Sensors::simulateAttitiudeChanges(){

  static int8_t dirP = 1;
  static int8_t dirR = 1;

  attitiude.pitch += dirP;
  if(attitiude.pitch >= 90){
    attitiude.pitch = 90;
    dirP = -1;
  }
  if(attitiude.pitch <= -90){
    attitiude.pitch = -90;
    dirP = 1;
  }

  attitiude.roll += dirR;
  if(attitiude.roll >= 90){
    attitiude.roll = 90;
    dirR = -1;
  }
  if(attitiude.roll <= -90){
    attitiude.roll = -90;
    dirR = 1;
  }


  attitiude.hdg += 1;
  if(attitiude.hdg >= 360)
    attitiude.hdg = 0;

}