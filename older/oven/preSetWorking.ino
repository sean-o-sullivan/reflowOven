#include <Arduino_GFX_Library.h>
#include <max6675.h>
#include <QuickPID.h>

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


const int NUM_PROFILE_POINTS = 5;
int profileTimes[NUM_PROFILE_POINTS];
float profileTemps[NUM_PROFILE_POINTS];

setupProfilePoints();

void setupProfilePoints() {
  profileTimes[0] = 0;
  profileTimes[1] = PREHEAT_DURATION;
  profileTimes[2] = PREHEAT_DURATION + SOAK_DURATION;
  profileTimes[3] = PREHEAT_DURATION + SOAK_DURATION + REFLOW_DURATION;
  profileTimes[4] = PREHEAT_DURATION + SOAK_DURATION + REFLOW_DURATION + PEAK_DURATION;

  profileTemps[0] = 25.0;   // Start temperature
  profileTemps[1] = 150.0;  // End of preheat
  profileTemps[2] = 180.0;  // End of soak
  profileTemps[3] = 240.0;  // Peak temperature
  profileTemps[4] = 240.0;  // Start of cooling
}

// Replace the existing getDesiredTemperature function with this:
float getDesiredTemperature(unsigned long elapsedSeconds) {
  if (elapsedSeconds >= profile_length) {
    return profileTemps[NUM_PROFILE_POINTS - 1];
  }

  for (int i = 1; i < NUM_PROFILE_POINTS; i++) {
    if (elapsedSeconds < profileTimes[i]) {
      float t = (float)(elapsedSeconds - profileTimes[i-1]) / (profileTimes[i] - profileTimes[i-1]);
      return profileTemps[i-1] + t * (profileTemps[i] - profileTemps[i-1]);
    }
  }

  // For cooling phase
  float coolingRate = (profileTemps[NUM_PROFILE_POINTS-1] - 150.0) / COOLING_DURATION;
  return profileTemps[NUM_PROFILE_POINTS-1] - coolingRate * (elapsedSeconds - profileTimes[NUM_PROFILE_POINTS-1]);
}

// Update the createReflowProfile function:
void createReflowProfile() {
  for (int i = 0; i < profile_length; i++) {
    reflow_profile[i] = getDesiredTemperature(i);
  }
}



// Reflow profile durations (in seconds)
const int PREHEAT_DURATION = 90;
const int SOAK_DURATION = 90;
const int REFLOW_DURATION = 60;
const int PEAK_DURATION = 15;
const int COOLING_DURATION = 90;

// Calculate total profile length
const int profile_length = PREHEAT_DURATION + SOAK_DURATION + REFLOW_DURATION + PEAK_DURATION + COOLING_DURATION;

static int count = 0;
static String temp_str = "0";

// MAX6675 pins for software SPI
#define MAX6675_SOFT_SCK_PIN 25
#define MAX6675_SOFT_CS_PIN  26
#define MAX6675_SOFT_MISO_PIN 27

#define buttonPin 2 
#define relayPin 32

const unsigned long windowSize = 250; // 0.5 seconds
const byte debounce = 50;
float Input, Output, Setpoint = 10, Kp = 2, Ki = 2, Kd = 4;

// status
unsigned long windowStartTime, nextSwitchTime;
boolean relayStatus = false;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

// Use software SPI for MAX6675
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

float reflow_profile[profile_length];

const unsigned long pidInterval = 500; // PID computation interval in milliseconds
const unsigned long displayInterval = 1000; // Display update interval in milliseconds

unsigned long reflowStartTime; // Declare reflowStartTime globally

void setup() {
  Serial.begin(57600);

  display.begin();
  display.setRotation(3);
  display.fillScreen(BLACK);

  pinMode(TFT_CS, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(MAX6675_SOFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX6675_SOFT_CS_PIN, HIGH);

  createReflowProfile();
  drawGraphAxes();
  drawLegend();
  drawVerticalBars();

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
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(100, 120);
    display.print("Press Button");
    display.setCursor(100, 80);
    display.print("Start Reflow");
    
    while (reflow_complete) {
      checkButton();
      if (state == 1) {
        reflow_complete = false;
        showingCompleteMessage = false;
        break;
      }
      delay(10);  // Short delay to prevent excessive CPU usage
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
        Serial.println("Push button pressed");
        if (state == 1) {
          state = 0;
          Serial.println("Reflow aborted");
        } else if (state == 0) {
          state = 1;
          Serial.println("Reflow started");
        }
      }
    }
  }

  lastButtonState = reading;
}

void reflow() {
  reflowStartTime = millis();
  static unsigned long lastPIDTime = 0;
  static unsigned long lastDisplayTime = 0;
  x_pos = 11;
  clearOldPlot();
  drawGraphAxes();
  drawReflowProfile();
  drawLegend();
  drawVerticalBars();

  while (state == 1) {
    unsigned long msNow = millis();
    unsigned long elapsedSeconds = (msNow - reflowStartTime) / 1000;

    if (msNow - lastPIDTime >= pidInterval) {
      computePID(msNow, elapsedSeconds);
      lastPIDTime = msNow;
    }

    if (msNow - lastDisplayTime >= displayInterval) {
      updateDisplay(msNow, elapsedSeconds);
      lastDisplayTime = msNow;
    }

    if (isReflowComplete(elapsedSeconds)) {
      reflow_complete = true;
      state = 0;
      break;
    }

    checkButton();
    if (state == 0) break;
  }
}

bool isReflowComplete(unsigned long elapsedSeconds) {
  return elapsedSeconds >= profile_length;
}

void computePID(unsigned long msNow, unsigned long elapsedSeconds) {
  float desired_temp = getDesiredTemperature(elapsedSeconds);
  temp = readTemperature();

  Setpoint = desired_temp;
  Input = temp;

  if (myPID.Compute()) {
    windowStartTime = msNow;
  }

  if (!relayStatus && Output > (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(relayPin, HIGH);
    }
  } else if (relayStatus && Output < (msNow - windowStartTime)) {
    if (msNow > nextSwitchTime) {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(relayPin, LOW);
    }
  }
}

void updateDisplay(unsigned long msNow, unsigned long elapsedSeconds) {
  float desired_temp = getDesiredTemperature(elapsedSeconds);
  temp = readTemperature();

  temp_str = String(temp, 1);
  String desired_temp_str = String(desired_temp, 1);

  float y_pos = map(temp, 0, 250, 230, 10);

  display.drawLine(x_pos - x_scale, temp_old, x_pos, y_pos, GREEN);
  temp_old = y_pos;

  display.fillRect(90, 170, 230, 60, BLACK);

  display.setCursor(100, 170);
  display.setTextColor(GREEN);
  display.setTextSize(2);
  display.print("Measured: ");
  display.print(temp_str);
  display.print(" C");

  display.setCursor(100, 195);
  display.setTextColor(RED);
  display.setTextSize(2);
  display.print("Desired: ");
  display.print(desired_temp_str);
  display.print(" C");

  x_pos += x_scale;
}



void drawReflowProfile() {
  for (int i = 1; i < profile_length; i++) {
    int x1 = map(i - 1, 0, profile_length, 10, 310);
    int y1 = map(reflow_profile[i - 1], 0, 250, 230, 10);
    int x2 = map(i, 0, profile_length, 10, 310);
    int y2 = map(reflow_profile[i], 0, 250, 230, 10);
    display.drawLine(x1, y1, x2, y2, RED);
  }
}

void drawGraphAxes() {
  display.drawFastVLine(10, 10, 230, WHITE);
  display.drawFastHLine(10, 230, 300, WHITE);

  display.setTextColor(YELLOW);
  display.setTextSize(1);
  display.setCursor(3, 226);
  display.print("0");

  for (int i = 50; i <= 250; i += 50) {
    int y = map(i, 0, 250, 230, 10);
    display.setCursor(3, y - 4);
    display.print(i);
    display.drawFastHLine(10, y, 5, WHITE);
  }
}

void drawVerticalBars() {
  int total_duration = profile_length;
  int preheat_end = PREHEAT_DURATION;
  int soak_end = preheat_end + SOAK_DURATION;
  int reflow_end = soak_end + REFLOW_DURATION;
  int peak_end = reflow_end + PEAK_DURATION;

  drawDottedLine(map(preheat_end, 0, total_duration, 10, 310), 10, 230, WHITE);
  drawDottedLine(map(soak_end, 0, total_duration, 10, 310), 10, 230, WHITE);
  drawDottedLine(map(reflow_end, 0, total_duration, 10, 310), 10, 230, WHITE);
  drawDottedLine(map(peak_end, 0, total_duration, 10, 310), 10, 230, WHITE);
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
  display.fillRect(10, 10, 310, 220, BLACK);
}