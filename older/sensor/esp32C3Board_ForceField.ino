#include "LSM6DSOXSensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

//plot the movement graphs live, will need to calibrate sensors, but this is a good start
#define LED_PIN1 5
#define LED_PIN2 6

#define SCL_PIN 1
#define SDA_PIN 0

#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define ADXL375_CS 10

LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

void scanI2C() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
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
    Serial.println("I2C scan done\n");
}

void setup() {
  Serial.begin(115200);
  

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  scanI2C();

  lsm6dsoxSensor.begin();

  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
    Serial.println("Success enabling LSM6DSOX accelero and gyro");
  } else {
    Serial.println("Error enabling LSM6DSOX accelero and gyro");
  }

  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
  } else {
    Serial.println("Received correct ID for LSM6DSOX sensor");
  }

  lsm6dsoxSensor.Set_X_FS(2);
  lsm6dsoxSensor.Set_G_FS(125);
  lsm6dsoxSensor.Set_X_ODR(208.0f);
  lsm6dsoxSensor.Set_G_ODR(208.0f);

  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL375 detected ... Check my wiring silly!");
    while (1);
  }

  displayDataRate();
}

void displayDataRate() {
  Serial.print("ADXL375 Data Rate: ");
  switch (accel.getDataRate()) {
    case ADXL343_DATARATE_3200_HZ: Serial.print("3200 "); break;
    case ADXL343_DATARATE_1600_HZ: Serial.print("1600 "); break;
    case ADXL343_DATARATE_800_HZ: Serial.print("800 "); break;
    case ADXL343_DATARATE_400_HZ: Serial.print("400 "); break;
    case ADXL343_DATARATE_200_HZ: Serial.print("200 "); break;
    case ADXL343_DATARATE_100_HZ: Serial.print("100 "); break;
    case ADXL343_DATARATE_50_HZ: Serial.print("50 "); break;
    case ADXL343_DATARATE_25_HZ: Serial.print("25 "); break;
    case ADXL343_DATARATE_12_5_HZ: Serial.print("12.5 "); break;
    case ADXL343_DATARATE_6_25HZ: Serial.print("6.25 "); break;
    case ADXL343_DATARATE_3_13_HZ: Serial.print("3.13 "); break;
    case ADXL343_DATARATE_1_56_HZ: Serial.print("1.56 "); break;
    case ADXL343_DATARATE_0_78_HZ: Serial.print("0.78 "); break;
    case ADXL343_DATARATE_0_39_HZ: Serial.print("0.39 "); break;
    case ADXL343_DATARATE_0_20_HZ: Serial.print("0.20 "); break;
    case ADXL343_DATARATE_0_10_HZ: Serial.print("0.10 "); break;
    default: Serial.print("???? "); break;
  }
  Serial.println("Hz");
}

void loop() {
  // Turn the LEDs on
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);

  // read LSM6DSOX accelerometer
  uint8_t acceleroStatus;
  lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
  int32_t lsmAcceleration[3] = {0, 0, 0};
  if (acceleroStatus == 1) {
    lsm6dsoxSensor.Get_X_Axes(lsmAcceleration);
  }

  // read LSM6DSOX gyroscope
  uint8_t gyroStatus;
  lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
  int32_t lsmRotation[3] = {0, 0, 0};
  if (gyroStatus == 1) {
    lsm6dsoxSensor.Get_G_Axes(lsmRotation);
  }

  // read ADXL375 accelerometer
  sensors_event_t adxlEvent;
  accel.getEvent(&adxlEvent);

  // print data for the Serial Plotter
  Serial.print("LSM_A_X:"); Serial.print(lsmAcceleration[0]); Serial.print(" ");
  Serial.print("LSM_A_Y:"); Serial.print(lsmAcceleration[1]); Serial.print(" ");
  Serial.print("LSM_A_Z:"); Serial.print(lsmAcceleration[2]); Serial.print(" ");
  Serial.print("LSM_G_X:"); Serial.print(lsmRotation[0]); Serial.print(" ");
  Serial.print("LSM_G_Y:"); Serial.print(lsmRotation[1]); Serial.print(" ");
  Serial.print("LSM_G_Z:"); Serial.print(lsmRotation[2]); Serial.print(" ");
  Serial.print("ADXL_A_X:"); Serial.print(adxlEvent.acceleration.x); Serial.print(" ");
  Serial.print("ADXL_A_Y:"); Serial.print(adxlEvent.acceleration.y); Serial.print(" ");
  Serial.print("ADXL_A_Z:"); Serial.println(adxlEvent.acceleration.z);

  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);

  delay(10);
}
