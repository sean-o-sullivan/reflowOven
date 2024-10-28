#include <Arduino_GFX_Library.h>
#include <max6675.h>
#include <QuickPID.h>

#include "bitmap_image.h"  
#include "flame_image.h" 

// celsius
float PREHEAT_START_TEMP = 25.0; // Room temperature
float PREHEAT_END_TEMP = 150.0; // Common preheat temperature to avoid thermal shock
float SOAK_TEMP = 150.0; // Soak temperature to ensure even heating
float REFLOW_START_TEMP = 150.0; // Start of reflow phase
float PEAK_TEMP = 240.0; // Peak temperature for solder reflow
float COOLING_END_TEMP = 150.0; // End temperature for cooling phase

// seconds
int preheat_duration = 90;  
int soak_duration = 60;     
int reflow_duration = 70;  
int peak_duration = 30;   
int cooling_duration = 50; 

#define LED_PREHEAT_PIN   33   
#define LED_SOAK_PIN      14 
#define LED_REFLOW_PIN    12  

// display pins
#define TFT_SCK    18
#define TFT_MOSI   23
#define TFT_MISO   19
#define TFT_CS     22
#define TFT_DC     21
#define TFT_RESET  17

#define BLUE      0x001F
#define GREEN     0x07E0
#define RED       0xF800
#define YELLOW    0xFFE0
#define WHITE     0xFFFF
#define BLACK     0x0000

static int count = 0;
static String temp_str = "0";

// MAX6675 pins for software SPI
#define MAX6675_SOFT_SCK_PIN 25
#define MAX6675_SOFT_CS_PIN  26
#define MAX6675_SOFT_MISO_PIN 27

#define buttonPin 2 
#define relayPin 32

const unsigned long windowSize = 250; 
const byte debounce = 50;
float Input, Output, Setpoint = 10, Kp = 1, Ki = 0.1, Kd = 0.1;
unsigned long windowStartTime, nextSwitchTime;
boolean relayStatus = false;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

MAX6675 thermocouple(MAX6675_SOFT_SCK_PIN, MAX6675_SOFT_CS_PIN, MAX6675_SOFT_MISO_PIN);

Arduino_ESP32SPI bus(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_ILI9341 display(&bus, TFT_RESET);

int x_pos;
float temp, temp_old = 120;
float last_desired_temp = 0;
byte x_scale = 1;
byte y_scale = 1;
bool reflow_complete = false;
int state = 0;

const unsigned long debounceDelay = 50;
unsigned long lastDebounceTime = 0;
int lastButtonState = HIGH;
int buttonState = HIGH;

// Define reflow profile
const int profile_length = 300; // 300 seconds (5 minutes)
float reflow_profile[profile_length];

// PID and display update intervals
const unsigned long pidInterval = 500; // milliseconds
const unsigned long displayInterval = 1000; // milliseconds

unsigned long reflowStartTime; 
unsigned long timeReflowing = 0;
unsigned int countfix = 1;


void setup() {
  Serial.begin(57600);

  display.begin();
  display.setRotation(3);
  display.fillScreen(BLACK);

  display.draw16bitRGBBitmap(0, 0, Untitled_design_18, 320, 250);


  pinMode(LED_PREHEAT_PIN, OUTPUT);
  pinMode(LED_SOAK_PIN, OUTPUT);
  pinMode(LED_REFLOW_PIN, OUTPUT);

  pinMode(TFT_CS, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(MAX6675_SOFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX6675_SOFT_CS_PIN, HIGH);

  createReflowProfile();


  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(pidInterval * 1000); 
  myPID.SetMode(myPID.Control::automatic);
}



void loop() {
  static unsigned long lastPIDTime = 0;
  static unsigned long lastDisplayTime = 0;
  unsigned long msNow = millis();

  checkButton();

  if (state == 1 && !reflow_complete) {
    reflow();
  }

  if (reflow_complete) {
    handleReflowComplete(msNow);
  }

  Serial.println(count);
}

void handleReflowComplete(unsigned long msNow) {
  static unsigned long lastUpdateTime = 0;
  static bool showingCompleteMessage = false;

  if (!showingCompleteMessage) {
    display.fillScreen(BLACK);
    display.setTextColor(YELLOW);
    display.setTextSize(3);
    display.setCursor(50, 100);
    display.print("Reflow Complete!");
    showingCompleteMessage = true;
    lastUpdateTime = msNow;
  } else if (msNow - lastUpdateTime >= 1000) {
  display.fillScreen(BLACK);
  // Display the bitmap on the screen
  display.draw16bitRGBBitmap(0, 0, Untitled_design_18, 320, 250);

    while (reflow_complete) {
      checkButton();
      if (state == 1) {
        reflow_complete = false;
        showingCompleteMessage = false;
        break;
      }
      delay(10);  
    }
  }
}

void checkButton() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        display.drawFastVLine(9, 100, 130, BLACK);  // Left border rectangle, 1px to the left of the white bar

        Serial.println("Push button pressed");
        if (state == 1) {
          state = 0;
          Serial.println("Reflow aborted");
        } else if (state == 0) {
          

          state = 1;
          display.drawFastVLine(9, 100, 130, BLACK);  // Left border rectangle, 1px to the left of the white bar

          Serial.println("Reflow started");

        }
      }
    }
  }

  lastButtonState = reading;
}

void reflow() {
  reflowStartTime = millis();
  display.drawFastVLine(9, 100, 130, BLACK);  // Left border rectangle

  static unsigned long lastPIDTime = 0;
  static unsigned long lastDisplayTime = 0;
  x_pos = 10;
  clearOldPlot();
  drawGraphAxes();

  display.draw16bitRGBBitmap(20, 20, Untitled_design_20, 50, 50);

  drawReflowProfile(); // Redraw the profile
  drawLegend();
  drawVerticalBars();


  digitalWrite(LED_PREHEAT_PIN, LOW);
  digitalWrite(LED_SOAK_PIN, LOW);
  digitalWrite(LED_REFLOW_PIN, LOW);


  while (state == 1) {
    unsigned long msNow = millis();
    timeReflowing = msNow - reflowStartTime; 
    
    if (msNow - lastPIDTime >= pidInterval) {
      computePID(msNow);
      lastPIDTime = msNow;
    }

    if (msNow - lastDisplayTime >= displayInterval) {
      updateDisplay(msNow, reflowStartTime);
      lastDisplayTime = msNow;
    }

    if ((msNow - reflowStartTime) < (preheat_duration * 1000)) {
      Serial.println("preheat pin should be on");
      digitalWrite(LED_PREHEAT_PIN, HIGH);
      digitalWrite(LED_SOAK_PIN, LOW);
      digitalWrite(LED_REFLOW_PIN, LOW);
    } else if ((msNow - reflowStartTime) < ((preheat_duration + soak_duration) * 1000)) {
      digitalWrite(LED_PREHEAT_PIN, HIGH);
      digitalWrite(LED_SOAK_PIN, HIGH);
      digitalWrite(LED_REFLOW_PIN, LOW);
    } else if ((msNow - reflowStartTime) < ((preheat_duration + soak_duration + reflow_duration) * 1000)) {
      digitalWrite(LED_PREHEAT_PIN, HIGH);
      digitalWrite(LED_SOAK_PIN, HIGH);
      digitalWrite(LED_REFLOW_PIN, HIGH);
    } else {
      digitalWrite(LED_PREHEAT_PIN, LOW);
      digitalWrite(LED_SOAK_PIN, LOW);
      digitalWrite(LED_REFLOW_PIN, LOW);
    }


    if ((msNow - reflowStartTime) >= 300000) {
      reflow_complete = true;
      state = 0;
      break;
    }

    checkButton();
    if (state == 0) break;
  }
}
void computePID(unsigned long msNow) {
  float desired_temp = getDesiredTemperature((msNow - reflowStartTime) / 1000);
  temp = readTemperature();

  Setpoint = desired_temp;
  Input = temp;

  if (myPID.Compute()) {
    windowStartTime = msNow;
  }

  Output = constrain(Output, 0, windowSize);

  // Control the relay based on the PID output
  if (Output > (msNow - windowStartTime)) {
    if (!relayStatus && msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(relayPin, HIGH);
    }
  } else {
    if (relayStatus && msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(relayPin, LOW);
    }
  }

  // Debug information
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(", Input: ");
  Serial.print(Input);
  Serial.print(", Output: ");
  Serial.print(Output);
  Serial.print(", Relay Status: ");
  Serial.println(relayStatus ? "ON" : "OFF");
}


void updateDisplay(unsigned long msNow, unsigned long reflowStartTime) {
  float desired_temp = getDesiredTemperature((msNow - reflowStartTime) / 1000);
  temp = readTemperature();

  temp_str = String(temp, 1);
  String desired_temp_str = String(desired_temp, 1);


float y_pos = map(temp, 0, 250, 230, 100);


display.drawFastVLine(9, 100, 130, BLACK);

  display.drawLine(x_pos - x_scale, temp_old, x_pos, y_pos, GREEN);

    if(countfix !=0){
    display.drawFastVLine(9, 100, 130, BLACK);
    countfix-=1;
  }


  temp_old = y_pos;
  display.drawFastVLine(10, 100, 130, WHITE);  // Left vertical axis

 // display.fillRect(0, 0, 320, 250, BLACK);

  // Create a black box to cover the previous text
  display.fillRect(113, 170, 65, 60, BLACK); // Adjust the dimensions if necessary

  display.setCursor(113, 170);
  display.setTextColor(GREEN);
  display.setTextSize(2);
  display.print(temp_str);
  display.print("C");

  display.setCursor(113, 195);
  display.setTextColor(RED);
  display.setTextSize(2);
  display.print(desired_temp_str);
  display.print("C");

  display.fillRect(25, 110, 60, 30, BLACK); 

  unsigned long timeReflowing = (msNow - reflowStartTime) / 1000; // Time in seconds

  display.setCursor(25, 110);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.print(timeReflowing);
  display.print("S");

   //      reflow indicators 

int screenWidth = 320;
int numPhases = 3;
int spacing = screenWidth / (numPhases + 1); // Calculate spacing between each phase

for (int i = 0; i < numPhases; i++) {
    int x_offset = spacing * (i + 1); // Calculate the x position for each phase

    switch (i) {
      case 0:
        // preheat
        display.setCursor(x_offset, 15);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print("pre");

        display.setCursor(x_offset, 40);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(preheat_duration);
        display.print("S");

        display.setCursor(x_offset, 65);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(int(PREHEAT_END_TEMP-PREHEAT_START_TEMP));
        display.print("C");
        break;

      case 1:
        // soak
        display.setCursor(x_offset, 15);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print("soak");

        display.setCursor(x_offset, 40);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(soak_duration);
        display.print("S");

        display.setCursor(x_offset, 65);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(int(REFLOW_START_TEMP));


        display.print("C");
        break;

      case 2:
        // peak 
        display.setCursor(x_offset, 15);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print("peak");

        display.setCursor(x_offset, 40);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(peak_duration);
        display.print("S");

        display.setCursor(x_offset, 65);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print(int(PEAK_TEMP));
        display.print("C");
        break;
    }
}



  x_pos += x_scale;
}

float readTemperature() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX6675_SOFT_CS_PIN, LOW);
  delay(1);

  float tempC = thermocouple.readCelsius();

  digitalWrite(MAX6675_SOFT_CS_PIN, HIGH);
  digitalWrite(TFT_CS, LOW);

  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println("C");

  return tempC;
}

float getDesiredTemperature(int elapsedSeconds) {
  int index = map(elapsedSeconds, 0, profile_length, 0, profile_length - 1);
  return reflow_profile[index];
}

void createReflowProfile() {
  // preheat stage
  for (int i = 0; i < preheat_duration; i++) {
    reflow_profile[i] = PREHEAT_START_TEMP + ((PREHEAT_END_TEMP - PREHEAT_START_TEMP) * i / preheat_duration);
  }

  // Soak stage
  for (int i = preheat_duration; i < preheat_duration + soak_duration; i++) {
    reflow_profile[i] = SOAK_TEMP;
  }

  // Reflow stage
  for (int i = preheat_duration + soak_duration; i < preheat_duration + soak_duration + reflow_duration; i++) {
    reflow_profile[i] = REFLOW_START_TEMP + ((PEAK_TEMP - REFLOW_START_TEMP) * (i - (preheat_duration + soak_duration)) / reflow_duration);
  }

  // Peak stage
  for (int i = preheat_duration + soak_duration + reflow_duration; i < preheat_duration + soak_duration + reflow_duration + peak_duration; i++) {
    reflow_profile[i] = PEAK_TEMP;
  }

  // Cooling stage
  for (int i = preheat_duration + soak_duration + reflow_duration + peak_duration; i < profile_length; i++) {
    reflow_profile[i] = PEAK_TEMP - ((PEAK_TEMP - COOLING_END_TEMP) * (i - (preheat_duration + soak_duration + reflow_duration + peak_duration)) / cooling_duration);
  }
}

void drawReflowProfile() {
  int total_duration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;

  for (int i = 1; i < profile_length; i++) {
    int elapsed_time = i;

    int x1 = map(elapsed_time - 1, 0, total_duration, 10, 310);
    int x2 = map(elapsed_time, 0, total_duration, 10, 310);

    int y1 = map(reflow_profile[i - 1], 0, 250, 230, 100);
    int y2 = map(reflow_profile[i], 0, 250, 230, 100);

    display.drawLine(x1, y1, x2, y2, RED);
  }
}


void drawGraphAxes() {
display.drawFastVLine(9, 100, 130, BLACK);

  // vertical and horizontal axes
  display.drawFastVLine(10, 10, 220, WHITE);
  display.drawFastHLine(10, 230, 301, WHITE);  
  display.drawFastHLine(10, 90, 301, WHITE);
  display.drawFastHLine(10, 10, 301, WHITE);  

  display.drawFastVLine(310, 10, 220, WHITE);
}



void drawVerticalBars() {

  int total_duration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;

  drawDottedLine(map(preheat_duration, 0, total_duration, 10, 310), 153, 230, WHITE);
  drawDottedLine(map(preheat_duration + soak_duration, 0, total_duration, 10, 310), 153, 230, WHITE);
  drawDottedLine(map(preheat_duration + soak_duration + reflow_duration, 0, total_duration, 10, 310), 108, 230, WHITE);
  drawDottedLine(map(preheat_duration + soak_duration + reflow_duration + peak_duration, 0, total_duration, 10, 310), 108, 230, WHITE);
}




void drawDottedLine(int x, int y_start, int y_end, uint16_t color) {
  for (int y = y_start; y < y_end; y += 4) {
    display.drawPixel(x, y, color);
  }
}

void drawLegend() {
  display.setTextColor(RED);
  display.setTextSize(1);
  display.setCursor(50, 240);
  display.print("Optimal Reflow Profile");

  display.setTextColor(GREEN);
  display.setCursor(200, 240);
  display.print("Measured Temp");
}

void clearOldPlot() {
  display.fillRect(0, 0, 320, 250, BLACK);
}
