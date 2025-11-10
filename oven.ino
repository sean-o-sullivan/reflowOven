// Reflow Oven OS v2.3.2 - One-sweep graphs for Reflow + Anneal
// - Reflow: single-sweep time-mapped plot
// - Anneal: single-sweep time-mapped plot, full target curve pre-rendered

#include <Arduino_GFX_Library.h>
#include <max6675.h>
#include <QuickPID.h>
#include <AiEsp32RotaryEncoder.h>

// Optional bitmaps
#include "bitmap_image.h"
#include "flame_image.h"

// ====================================================================================
// TUNABLE PARAMETERS
// ====================================================================================

// TIMING (ms)
const unsigned long GLOBAL_PID_INTERVAL       = 10;   // PID compute cadence
const unsigned long GLOBAL_TEMP_READ_INTERVAL = 175;  // Thermocouple read cadence
const unsigned long GLOBAL_DISPLAY_INTERVAL   = 300;  // Display update cadence

// PID GAINS
float GLOBAL_KP = 6;
float GLOBAL_KI = 0.01;
float GLOBAL_KD = 0.0;

// Anneal Profile
float ANNEAL_TEMP = 150.0;
int   anneal_duration = 1200;           // seconds (10 min)
int   anneal_ramp_seconds = 60;         // ramp up time to protect muscle wire

// TEMPERATURE SMOOTHING
float GLOBAL_TEMP_SMOOTHING = 0.0;      // 0..1 (0 = fast)

// RELAY CONTROL
const unsigned long GLOBAL_WINDOW_SIZE = 5000; // SSR time-proportioning window (ms)
const byte GLOBAL_RELAY_DEBOUNCE       = 10;   // (ms)

// ====================================================================================
// END OF TUNABLES
// ====================================================================================

// --- PIN DEFINITIONS ---
#define TFT_SCK    18
#define TFT_MOSI   23
#define TFT_MISO   19
#define TFT_CS     22
#define TFT_DC     21
#define TFT_RESET  17

// Thermocouple (Software SPI)
#define MAX6675_SOFT_SCK_PIN 25
#define MAX6675_SOFT_CS_PIN  26
#define MAX6675_SOFT_MISO_PIN 27

// Rotary Encoder
#define ROTARY_ENCODER_A_PIN      4
#define ROTARY_ENCODER_B_PIN      0
#define ROTARY_ENCODER_BUTTON_PIN 13
#define ROTARY_ENCODER_VCC_PIN    -1
#define ROTARY_ENCODER_STEPS      4

// Outputs
#define RELAY_PIN         32
#define LED_PREHEAT_PIN   33
#define LED_SOAK_PIN      14
#define LED_REFLOW_PIN    12

// --- COLORS ---
#define BLUE        0x001F
#define GREEN       0x07E0
#define RED         0xF800
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define BLACK       0x0000
#define GREY        0x8410
#define DARK_GREY   0x4208
#define CYAN        0x07FF
#define ORANGE      0xFD20
#define PURPLE      0x801F
#define LIGHT_GREY  0xC618
#define DARK_BLUE   0x0011

// --- GLOBAL OBJECTS ---
Arduino_ESP32SPI bus(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_ILI9341 display(&bus, TFT_RESET);
MAX6675 thermocouple(MAX6675_SOFT_SCK_PIN, MAX6675_SOFT_CS_PIN, MAX6675_SOFT_MISO_PIN);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// --- STATE MACHINE ---
enum ScreenState {
  SPLASH_SCREEN,
  MAIN_MENU,
  SETTINGS_MENU,
  REFLOW_SETTINGS,
  ANNEAL_SETTINGS,
  REFLOW_RUNNING,
  ANNEAL_RUNNING,
  PROCESS_COMPLETE
};
ScreenState currentState = SPLASH_SCREEN;


// // PROFILE A: gentle, soak/Lee-style for 0201s on 2-layer
// PREHEAT_END_TEMP = 150;  preheat_duration = 120;   // 25→150 °C in ~120 s  (~1.0 °C/s)
// SOAK_TEMP        = 150;  soak_duration    = 60;    // 60 s soak (tombstone mitigation)
// REFLOW_START_TEMP= 150;  reflow_duration  = 60;    // 150→215 °C in 60 s (~1.1 °C/s)
// PEAK_TEMP        = 215;  peak_duration    = 15;    // short peak
// COOLING_END_TEMP = 150;  cooling_duration = 60;    // controlled cool
// // If you sum durations, set: profile_length = 315;



// // PROFILE B: compact, higher copper stack, keep TAL mid-high, modest peak
// PREHEAT_END_TEMP = 155;  preheat_duration = 135;   // ~0.96 °C/s
// SOAK_TEMP        = 155;  soak_duration    = 60;    // even out inner layers
// REFLOW_START_TEMP= 155;  reflow_duration  = 70;    // 155→228 °C in 70 s (~1.0 °C/s)
// PEAK_TEMP        = 228;  peak_duration    = 15;    // short peak to protect 01005s
// COOLING_END_TEMP = 160;  cooling_duration = 50;    // steady cool
// // Sum = 330 s ⇒ profile_length = 330;


// --- Generic PROFILE SETTINGS (Reflow) ---
float PREHEAT_START_TEMP = 25.0;
float PREHEAT_END_TEMP   = 155.0;
float SOAK_TEMP          = 155.0;
float REFLOW_START_TEMP  = 155.0;
float PEAK_TEMP          = 228.0;
float COOLING_END_TEMP   = 160.0;

int preheat_duration = 135;  // s
int soak_duration    = 60;  // s
int reflow_duration  = 70;  // s
int peak_duration    = 15;  // s
int cooling_duration = 50;  // s


const int profile_length = 330;

float reflow_profile[profile_length];


// --- PID CONTROLLER ---
float Input, Output, Setpoint;
const unsigned long windowSize = GLOBAL_WINDOW_SIZE;
unsigned long windowStartTime;
const byte debounce = GLOBAL_RELAY_DEBOUNCE;
unsigned long nextSwitchTime;
bool relayStatus = false;

QuickPID myPID(&Input, &Output, &Setpoint, GLOBAL_KP, GLOBAL_KI, GLOBAL_KD,
               QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
               QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

// --- MENU & UI GLOBALS ---
int mainMenuSelection = 0;
int settingsMenuSelection = 0;
int paramMenuSelection = 0;
bool isEditingValue = false;
unsigned long processStartTime;

// ========= GRAPHING (REFLOW) =========
// one full run fits the screen width
const int GRAPH_X_MIN = 10;
const int GRAPH_X_MAX = 310;
int  totalDuration = 0;     // seconds, set at reflow start
int  lastPlotX = GRAPH_X_MIN;
float lastPlotY = 120;
int  lastPlotSec = 0;

inline int secToX(int sec) {
  return map(sec, 0, totalDuration, GRAPH_X_MIN, GRAPH_X_MAX);
}

// ========= GRAPHING (ANNEAL) =========
// time-mapped single sweep for anneal
const int AN_GRAPH_X_MIN = 10;
const int AN_GRAPH_X_MAX = 310;
int   anneal_totalDuration = 0;   // seconds (== anneal_duration at start)
int   anneal_lastX = AN_GRAPH_X_MIN;
float anneal_lastY_actual = 120;
float anneal_start_temp   = 25.0;

inline int secToXAnneal(int sec) {
  return map(sec, 0, anneal_totalDuration, AN_GRAPH_X_MIN, AN_GRAPH_X_MAX);
}

// --- PROTOTYPES ---
void drawSplashScreen();
void drawMainMenu(int selection);
void handleMainMenu();
void drawSettingsMenu(int selection);
void handleSettingsMenu();
void drawReflowSettings(int selection, bool editing);
void handleReflowSettings();
void drawAnnealSettings(int selection, bool editing);
void handleAnnealSettings();
void runReflowProfile();
void runAnnealProfile();
void handleProcessComplete();
float readTemperature();
void createReflowProfile();
void drawGraphAxes();
void drawReflowProfileLine();
void drawVerticalBars();
void drawDottedLine(int x, int y_start, int y_end, uint16_t color);
void drawLegend();
void clearOldPlot();
void drawRoundedRect(int x, int y, int w, int h, int r, uint16_t color);
void fillRoundedRect(int x, int y, int w, int h, int r, uint16_t color);
void drawMilestoneInfo();
void setupAnnealGraph();
void updateAnnealGraph(float currentTemp, float targetTemp, unsigned long timeElapsed);
void drawAnnealTargetLine();   // NEW

//====================================================================================
// SETUP
//====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  display.begin();
  display.setRotation(3);
  display.fillScreen(BLACK);

  pinMode(TFT_CS, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PREHEAT_PIN, OUTPUT);
  pinMode(LED_SOAK_PIN, OUTPUT);
  pinMode(LED_REFLOW_PIN, OUTPUT);
  pinMode(MAX6675_SOFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX6675_SOFT_CS_PIN, HIGH);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PREHEAT_PIN, LOW);
  digitalWrite(LED_SOAK_PIN, LOW);
  digitalWrite(LED_REFLOW_PIN, LOW);

  delay(500); // MAX6675 settle

  rotaryEncoder.begin();
  rotaryEncoder.setup(
    [] { rotaryEncoder.readEncoder_ISR(); },
    [] { rotaryEncoder.readButton_ISR(); }
  );
  rotaryEncoder.setAcceleration(250);
  rotaryEncoder.setBoundaries(0, 2, false);
  rotaryEncoder.setEncoderValue(0);

  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(GLOBAL_PID_INTERVAL * 1000);
  myPID.SetMode(myPID.Control::automatic);
  myPID.SetTunings(GLOBAL_KP, GLOBAL_KI, GLOBAL_KD);

  createReflowProfile();

  drawSplashScreen();
  currentState = MAIN_MENU;
  drawMainMenu(mainMenuSelection);
}

//====================================================================================
// MAIN LOOP
//====================================================================================
void loop() {
  switch (currentState) {
    case MAIN_MENU:        handleMainMenu(); break;
    case SETTINGS_MENU:    handleSettingsMenu(); break;
    case REFLOW_SETTINGS:  handleReflowSettings(); break;
    case ANNEAL_SETTINGS:  handleAnnealSettings(); break;
    case REFLOW_RUNNING:   runReflowProfile(); break;
    case ANNEAL_RUNNING:   runAnnealProfile(); break;
    case PROCESS_COMPLETE: handleProcessComplete(); break;
    case SPLASH_SCREEN:    break;
  }
  delay(10);
}

//====================================================================================
// TEMPERATURE READING
//====================================================================================
float readTemperature() {
  static float lastTemp = 25.0;
  static unsigned long lastReadTime = 0;

  if (millis() - lastReadTime < GLOBAL_TEMP_READ_INTERVAL) return lastTemp;
  lastReadTime = millis();

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX6675_SOFT_CS_PIN, LOW);
  delayMicroseconds(200);

  float tempC = thermocouple.readCelsius();

  digitalWrite(MAX6675_SOFT_CS_PIN, HIGH);
  digitalWrite(TFT_CS, LOW);
  delayMicroseconds(200);

  if (!isnan(tempC) && tempC > -100 && tempC < 500) {
    lastTemp = (lastTemp * GLOBAL_TEMP_SMOOTHING) + (tempC * (1.0 - GLOBAL_TEMP_SMOOTHING));
  }
  return lastTemp;
}

//====================================================================================
// REFLOW PROFILE (one-sweep graph)
//====================================================================================
void runReflowProfile() {
  static unsigned long lastTempRead = 0;
  static unsigned long lastPIDCompute = 0;
  static unsigned long lastDisplayUpdate = 0;

  unsigned long currentMillis = millis();

  if (rotaryEncoder.isEncoderButtonClicked()) {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PREHEAT_PIN, LOW);
    digitalWrite(LED_SOAK_PIN, LOW);
    digitalWrite(LED_REFLOW_PIN, LOW);
    currentState = MAIN_MENU;
    rotaryEncoder.setBoundaries(0, 2, false);
    rotaryEncoder.setEncoderValue(mainMenuSelection);
    drawMainMenu(mainMenuSelection);
    return;
  }

  unsigned long timeReflowing = currentMillis - processStartTime;
  int currentSec = (int)(timeReflowing / 1000);
  if (currentSec < 0) currentSec = 0;
  if (totalDuration <= 0) totalDuration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;
  if (currentSec > totalDuration - 1) currentSec = totalDuration - 1;

  if (timeReflowing < (unsigned long)preheat_duration * 1000UL) {
    digitalWrite(LED_PREHEAT_PIN, HIGH); digitalWrite(LED_SOAK_PIN, LOW);  digitalWrite(LED_REFLOW_PIN, LOW);
  } else if (timeReflowing < (unsigned long)(preheat_duration + soak_duration) * 1000UL) {
    digitalWrite(LED_PREHEAT_PIN, HIGH); digitalWrite(LED_SOAK_PIN, HIGH); digitalWrite(LED_REFLOW_PIN, LOW);
  } else if (timeReflowing < (unsigned long)(preheat_duration + soak_duration + reflow_duration) * 1000UL) {
    digitalWrite(LED_PREHEAT_PIN, HIGH); digitalWrite(LED_SOAK_PIN, HIGH); digitalWrite(LED_REFLOW_PIN, HIGH);
  } else {
    digitalWrite(LED_PREHEAT_PIN, LOW);  digitalWrite(LED_SOAK_PIN, LOW);  digitalWrite(LED_REFLOW_PIN, LOW);
  }

  if (timeReflowing >= (unsigned long)totalDuration * 1000UL) {
    digitalWrite(RELAY_PIN, LOW);
    currentState = PROCESS_COMPLETE;
    return;
  }

  if (currentMillis - lastTempRead >= GLOBAL_TEMP_READ_INTERVAL) {
    Input = readTemperature();
    lastTempRead = currentMillis;
  }

  if (currentMillis - lastPIDCompute >= GLOBAL_PID_INTERVAL) {
    Setpoint = reflow_profile[currentSec];

    if (myPID.Compute()) windowStartTime = currentMillis;

    if (Output > (currentMillis - windowStartTime)) {
      if (!relayStatus && currentMillis > nextSwitchTime) {
        nextSwitchTime = currentMillis + debounce;
        relayStatus = true; digitalWrite(RELAY_PIN, HIGH);
      }
    } else {
      if (relayStatus && currentMillis > nextSwitchTime) {
        nextSwitchTime = currentMillis + debounce;
        relayStatus = false; digitalWrite(RELAY_PIN, LOW);
      }
    }
    lastPIDCompute = currentMillis;

    Serial.print("R| SP:");  Serial.print(Setpoint, 1);
    Serial.print(" PV:");     Serial.print(Input, 1);
    Serial.print(" OUT:");    Serial.print(Output, 0);
    Serial.print(" PWM:");    Serial.print((Output / windowSize) * 100, 0);
    Serial.println("%");
  }

  if (currentMillis - lastDisplayUpdate >= GLOBAL_DISPLAY_INTERVAL) {
    int   xCurr = secToX(currentSec);
    float yCurr = map(Input, 0, 250, 210, 50);

    if (xCurr > lastPlotX) {
      display.drawLine(lastPlotX, lastPlotY, xCurr, yCurr, GREEN);
      lastPlotX = xCurr;
      lastPlotY = yCurr;
      lastPlotSec = currentSec;
    }

    display.fillRect(10, 214, 300, 22, BLACK);
    display.setCursor(10, 216);   display.setTextColor(GREEN); display.setTextSize(2); display.printf("%.1fC", Input);
    display.setCursor(100, 216);  display.setTextColor(RED);   display.printf("%.1fC", Setpoint);
    display.setCursor(190, 216);  display.setTextColor(WHITE); display.printf("%ds", currentSec);

    lastDisplayUpdate = currentMillis;
  }
}

//====================================================================================
// ANNEAL PROFILE (single-sweep graph + 60 s ramp to target)
//====================================================================================
void runAnnealProfile() {
  static unsigned long lastPIDTime = 0;
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastTempRead = 0;

  unsigned long currentMillis = millis();
  unsigned long elapsedMs = currentMillis - processStartTime;
  unsigned long currentSec = elapsedMs / 1000;

  if (rotaryEncoder.isEncoderButtonClicked()) {
    digitalWrite(RELAY_PIN, LOW);
    currentState = MAIN_MENU;
    rotaryEncoder.setBoundaries(0, 2, false);
    rotaryEncoder.setEncoderValue(mainMenuSelection);
    drawMainMenu(mainMenuSelection);
    return;
  }

  if (currentSec >= (unsigned long)anneal_duration) {
    digitalWrite(RELAY_PIN, LOW);
    currentState = PROCESS_COMPLETE;
    return;
  }

  if (currentMillis - lastTempRead >= GLOBAL_TEMP_READ_INTERVAL) {
    Input = readTemperature();
    lastTempRead = currentMillis;
  }

  if (currentMillis - lastPIDTime >= GLOBAL_PID_INTERVAL) {
    float target;
    if ((int)currentSec < anneal_ramp_seconds) {
      float frac = (float)currentSec / (float)anneal_ramp_seconds;
      target = anneal_start_temp + (ANNEAL_TEMP - anneal_start_temp) * frac;
    } else {
      target = ANNEAL_TEMP;
    }
    Setpoint = target;

    if (myPID.Compute()) windowStartTime = currentMillis;

    if (Output > (currentMillis - windowStartTime)) {
      if (!relayStatus && currentMillis > nextSwitchTime) {
        nextSwitchTime = currentMillis + debounce;
        relayStatus = true; digitalWrite(RELAY_PIN, HIGH);
      }
    } else {
      if (relayStatus && currentMillis > nextSwitchTime) {
        nextSwitchTime = currentMillis + debounce;
        relayStatus = false; digitalWrite(RELAY_PIN, LOW);
      }
    }

    lastPIDTime = currentMillis;

    Serial.print("A| SP:");  Serial.print(Setpoint, 1);
    Serial.print(" PV:");     Serial.print(Input, 1);
    Serial.print(" OUT:");    Serial.print(Output, 0);
    Serial.print(" PWM:");    Serial.print((Output / windowSize) * 100, 0);
    Serial.println("%");
  }

  if (currentMillis - lastDisplayUpdate >= GLOBAL_DISPLAY_INTERVAL) {
    updateAnnealGraph(Input, Setpoint, currentSec);
    lastDisplayUpdate = currentMillis;
  }
}

//====================================================================================
// STATE HANDLERS
//====================================================================================
void handleMainMenu() {
  if (rotaryEncoder.encoderChanged()) {
    mainMenuSelection = rotaryEncoder.readEncoder();
    drawMainMenu(mainMenuSelection);
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    switch (mainMenuSelection) {
      case 0: { // Anneal Twist
        currentState = ANNEAL_RUNNING;
        processStartTime = millis();

        // Initialize anneal graph + timeline
        anneal_start_temp = readTemperature();
        setupAnnealGraph();
        anneal_totalDuration = anneal_duration;
        anneal_lastX = AN_GRAPH_X_MIN;
        anneal_lastY_actual = map(anneal_start_temp, 0, 250, 210, 50);

        // >>> draw the full desired curve once <<<
        drawAnnealTargetLine();
        break;
      }
      case 1: { // Reflow Board
        currentState = REFLOW_RUNNING;
        processStartTime = millis();

        clearOldPlot();
        drawGraphAxes();
        drawReflowProfileLine();
        drawLegend();
        drawVerticalBars();
        drawMilestoneInfo();

        totalDuration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;
        lastPlotSec = 0;
        lastPlotX = GRAPH_X_MIN;
        lastPlotY = map(readTemperature(), 0, 250, 210, 50);
        break;
      }
      case 2:
        currentState = SETTINGS_MENU;
        settingsMenuSelection = 0;
        rotaryEncoder.setBoundaries(0, 2, false);
        rotaryEncoder.setEncoderValue(0);
        drawSettingsMenu(settingsMenuSelection);
        break;
    }
  }
}

void handleSettingsMenu() {
  if (rotaryEncoder.encoderChanged()) {
    settingsMenuSelection = rotaryEncoder.readEncoder();
    drawSettingsMenu(settingsMenuSelection);
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    paramMenuSelection = 0;
    rotaryEncoder.setEncoderValue(0);
    isEditingValue = false;
    switch (settingsMenuSelection) {
      case 0:
        currentState = REFLOW_SETTINGS;
        rotaryEncoder.setBoundaries(0, 4, false);
        drawReflowSettings(paramMenuSelection, isEditingValue);
        break;
      case 1:
        currentState = ANNEAL_SETTINGS;
        rotaryEncoder.setBoundaries(0, 2, false);
        drawAnnealSettings(paramMenuSelection, isEditingValue);
        break;
      case 2:
        currentState = MAIN_MENU;
        rotaryEncoder.setBoundaries(0, 2, false);
        rotaryEncoder.setEncoderValue(mainMenuSelection);
        drawMainMenu(mainMenuSelection);
        break;
    }
  }
}

void handleReflowSettings() {
  if (isEditingValue) {
    if (rotaryEncoder.encoderChanged()) {
      int change = rotaryEncoder.readEncoder() - paramMenuSelection;
      rotaryEncoder.setEncoderValue(paramMenuSelection);
      switch (paramMenuSelection) {
        case 0: PREHEAT_END_TEMP = constrain(PREHEAT_END_TEMP + change, 100, 200); break;
        case 1: preheat_duration = constrain(preheat_duration + change, 30, 180);   break;
        case 2: PEAK_TEMP = constrain(PEAK_TEMP + change, 200, 260);                break;
        case 3: peak_duration = constrain(peak_duration + change, 10, 60);          break;
      }
      drawReflowSettings(paramMenuSelection, isEditingValue);
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      isEditingValue = false;
      createReflowProfile();
      rotaryEncoder.setBoundaries(0, 4, false);
      drawReflowSettings(paramMenuSelection, isEditingValue);
    }
  } else {
    if (rotaryEncoder.encoderChanged()) {
      paramMenuSelection = rotaryEncoder.readEncoder();
      drawReflowSettings(paramMenuSelection, isEditingValue);
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      if (paramMenuSelection == 4) {
        currentState = SETTINGS_MENU;
        rotaryEncoder.setBoundaries(0, 2, false);
        rotaryEncoder.setEncoderValue(settingsMenuSelection);
        drawSettingsMenu(settingsMenuSelection);
      } else {
        isEditingValue = true;
        rotaryEncoder.setBoundaries(paramMenuSelection - 50, paramMenuSelection + 50, false);
        rotaryEncoder.setEncoderValue(paramMenuSelection);
        drawReflowSettings(paramMenuSelection, isEditingValue);
      }
    }
  }
}

void handleAnnealSettings() {
  if (isEditingValue) {
    if (rotaryEncoder.encoderChanged()) {
      int change = rotaryEncoder.readEncoder() - paramMenuSelection;
      rotaryEncoder.setEncoderValue(paramMenuSelection);
      switch (paramMenuSelection) {
        case 0: ANNEAL_TEMP = constrain(ANNEAL_TEMP + change, 150, 250); break;
        case 1: anneal_duration = constrain(anneal_duration + change * 10, 60, 3600); break;
      }
      drawAnnealSettings(paramMenuSelection, isEditingValue);
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      isEditingValue = false;
      rotaryEncoder.setBoundaries(0, 2, false);
      drawAnnealSettings(paramMenuSelection, isEditingValue);
    }
  } else {
    if (rotaryEncoder.encoderChanged()) {
      paramMenuSelection = rotaryEncoder.readEncoder();
      drawAnnealSettings(paramMenuSelection, isEditingValue);
    }
    if (rotaryEncoder.isEncoderButtonClicked()) {
      if (paramMenuSelection == 2) {
        currentState = SETTINGS_MENU;
        rotaryEncoder.setBoundaries(0, 2, false);
        rotaryEncoder.setEncoderValue(settingsMenuSelection);
        drawSettingsMenu(settingsMenuSelection);
      } else {
        isEditingValue = true;
        rotaryEncoder.setBoundaries(paramMenuSelection - 50, paramMenuSelection + 50, false);
        rotaryEncoder.setEncoderValue(paramMenuSelection);
        drawAnnealSettings(paramMenuSelection, isEditingValue);
      }
    }
  }
}

void handleProcessComplete() {
  display.fillScreen(BLACK);
  display.setTextColor(CYAN);
  display.setTextSize(3);
  display.setCursor(30, 80);  display.println("Process");
  display.setCursor(30, 120); display.println("Complete!");

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(50, 180);
  display.println("Press to continue");

  while (true) {
    if (rotaryEncoder.isEncoderButtonClicked()) {
      currentState = MAIN_MENU;
      rotaryEncoder.setBoundaries(0, 2, false);
      rotaryEncoder.setEncoderValue(mainMenuSelection);
      drawMainMenu(mainMenuSelection);
      break;
    }
    delay(10);
  }
}

//====================================================================================
// UI
//====================================================================================
void drawMainMenu(int selection) {
  display.fillScreen(BLACK);

  display.drawLine(10, 5, 50, 5, ORANGE);
  display.drawLine(270, 5, 310, 5, ORANGE);
  display.drawLine(10, 5, 10, 25, ORANGE);
  display.drawLine(310, 5, 310, 25, ORANGE);

  for (int y = 30; y < 230; y += 15) {
    display.setCursor(5, y);
    display.setTextSize(1);
    display.setTextColor((y % 30 == 0) ? RED : ORANGE);
    display.print("|");
    display.setCursor(308, y);
    display.print("|");
  }

  display.setTextSize(2);
  display.setCursor(20, 20); display.setTextColor(WHITE);  display.print("  ___  _   _ ___  _  _");
  display.setCursor(20, 36); display.setTextColor(ORANGE); display.print(" / _ \\| | | | __|| \\| |");
  display.setCursor(20, 52); display.setTextColor(RED);    display.print("| |_| |\\ V /| _| | .  |");
  display.setCursor(20, 68); display.setTextColor(YELLOW); display.print(" \\___/  \\_/ |___||_|\\_|");

  display.drawLine(20, 95, 300, 95, ORANGE);
  display.drawLine(20, 97, 300, 97, DARK_GREY);

  const char* menuItems[] = {"Anneal Twist", "Reflow Board", "Settings"};
  const uint16_t itemColors[] = {ORANGE, GREEN, CYAN};

  for (int i = 0; i < 3; i++) {
    int y = 115 + (i * 40);
    display.setTextSize(2);
    display.setTextColor(DARK_GREY);
    display.setCursor(30, y + 5); display.print("[");
    display.setCursor(280, y + 5); display.print("]");

    if (i == selection) {
      fillRoundedRect(40, y - 5, 240, 35, 5, DARK_BLUE);
      display.setTextColor(WHITE);
    } else {
      display.setTextColor(itemColors[i]);
    }
    display.setCursor(80, y + 5);
    display.println(menuItems[i]);
  }

  display.drawLine(10, 235, 50, 235, ORANGE);
  display.drawLine(270, 235, 310, 235, ORANGE);
  display.drawLine(10, 215, 10, 235, ORANGE);
  display.drawLine(310, 215, 310, 235, ORANGE);
}

void drawSettingsMenu(int selection) {
  display.fillScreen(BLACK);
  display.setTextColor(CYAN);
  display.setTextSize(3);
  display.setCursor(70, 30); display.println("Settings");
  display.drawLine(70, 58, 250, 58, CYAN);

  const char* menuItems[] = {"Reflow Profile", "Anneal Profile", "<- Back"};

  for (int i = 0; i < 3; i++) {
    int y = 90 + (i * 45);
    if (i == selection) {
      fillRoundedRect(40, y - 5, 240, 35, 5, DARK_BLUE);
      display.setTextColor(WHITE);
    } else {
      display.setTextColor(LIGHT_GREY);
    }
    display.setTextSize(2);
    display.setCursor(60, y + 5);
    display.println(menuItems[i]);
  }
}

void drawReflowSettings(int selection, bool editing) {
  display.fillScreen(BLACK);
  display.setTextColor(CYAN);
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.println("Reflow Parameters:");
  display.drawLine(20, 40, 300, 40, CYAN);

  const char* labels[] = {"Preheat Temp:", "Preheat Time:", "Peak Temp:", "Peak Time:", "<- Back"};
  float values[] = {PREHEAT_END_TEMP, (float)preheat_duration, PEAK_TEMP, (float)peak_duration, 0};
  const char* units[] = {"C", "s", "C", "s", ""};

  for (int i = 0; i < 5; i++) {
    int y = 60 + (i * 30);

    if (i == selection && !editing) {
      fillRoundedRect(15, y - 5, 290, 28, 5, DARK_BLUE);
      display.setTextColor(WHITE);
    } else {
      display.setTextColor(LIGHT_GREY);
    }

    display.setCursor(20, y);
    display.print(labels[i]);

    if (i < 4) {
      if (i == selection && editing) {
        drawRoundedRect(195, y - 5, 80, 28, 5, RED);
        display.setTextColor(YELLOW);
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(200, y);
      display.printf("%.0f%s", values[i], units[i]);
    }
  }
}

void drawAnnealSettings(int selection, bool editing) {
  display.fillScreen(BLACK);
  display.setTextColor(CYAN);
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.println("Anneal Parameters:");
  display.drawLine(20, 40, 300, 40, CYAN);

  const char* labels[] = {"Anneal Temp:", "Anneal Time:", "<- Back"};
  float values[] = {ANNEAL_TEMP, (float)anneal_duration, 0};
  const char* units[] = {"C", "s", ""};

  for (int i = 0; i < 3; i++) {
    int y = 60 + (i * 30);

    if (i == selection && !editing) {
      fillRoundedRect(15, y - 5, 290, 28, 5, DARK_BLUE);
      display.setTextColor(WHITE);
    } else {
      display.setTextColor(LIGHT_GREY);
    }

    display.setCursor(20, y);
    display.print(labels[i]);

    if (i < 2) {
      if (i == selection && editing) {
        drawRoundedRect(195, y - 5, 80, 28, 5, RED);
        display.setTextColor(YELLOW);
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(200, y);
      display.printf("%.0f%s", values[i], units[i]);
    }
  }
}

//====================================================================================
// HELPERS & GRAPH
//====================================================================================
void drawRoundedRect(int x, int y, int w, int h, int r, uint16_t color) {
  display.drawLine(x + r, y, x + w - r, y, color);
  display.drawLine(x + r, y + h, x + w - r, y + h, color);
  display.drawLine(x, y + r, x, y + h - r, color);
  display.drawLine(x + w, y + r, x + w, y + h - r, color);
  display.drawCircle(x + r, y + r, r, color);
  display.drawCircle(x + w - r, y + r, r, color);
  display.drawCircle(x + r, y + h - r, r, color);
  display.drawCircle(x + w - r, y + h - r, r, color);
}

void fillRoundedRect(int x, int y, int w, int h, int r, uint16_t color) {
  display.fillRect(x + r, y, w - 2 * r, h, color);
  display.fillRect(x, y + r, w, h - 2 * r, color);
  display.fillCircle(x + r, y + r, r, color);
  display.fillCircle(x + w - r, y + r, r, color);
  display.fillCircle(x + r, y + h - r, r, color);
  display.fillCircle(x + w - r, y + h - r, r, color);
}

void drawMilestoneInfo() {
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(15, 10);  display.print("PREHEAT");
  display.setCursor(15, 20);  display.printf("%ds", preheat_duration);
  display.setCursor(15, 30);  display.printf("%.0fC", PREHEAT_END_TEMP);

  display.setCursor(100, 10); display.print("SOAK");
  display.setCursor(100, 20); display.printf("%ds", soak_duration);
  display.setCursor(100, 30); display.printf("%.0fC", SOAK_TEMP);

  display.setCursor(170, 10); display.print("PEAK");
  display.setCursor(170, 20); display.printf("%ds", peak_duration);
  display.setCursor(170, 30); display.printf("%.0fC", PEAK_TEMP);

  display.setCursor(240, 10); display.print("COOL");
  display.setCursor(240, 20); display.printf("%ds", cooling_duration);
  display.setCursor(240, 30); display.printf("%.0fC", COOLING_END_TEMP);
}

void setupAnnealGraph() {
  display.fillScreen(BLACK);
  display.setTextColor(ORANGE);
  display.setTextSize(2);
  display.setCursor(20, 10);
  display.print("Annealing Muscle Wire");

  display.drawFastVLine(10, 50, 160, WHITE);
  display.drawFastHLine(10, 210, 300, WHITE);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  for (int i = 0; i <= 250; i += 50) {
    int y = map(i, 0, 250, 210, 50);
    display.setCursor(18, y - 4);
    display.printf("%d", i);
    display.drawFastHLine(8, y, 4, WHITE);
  }

  display.setTextColor(RED);   display.setCursor(15, 45); display.print("Target");
  display.setTextColor(GREEN); display.setCursor(60, 45); display.print("Actual");
}

// NEW: draw full anneal target curve once (ramp -> hold)
void drawAnnealTargetLine() {
  if (anneal_totalDuration <= 0) anneal_totalDuration = anneal_duration;

  float prevTarget = anneal_start_temp;
  int   prevX = secToXAnneal(0);
  int   prevY = map(prevTarget, 0, 250, 210, 50);

  for (int s = 1; s <= anneal_totalDuration; ++s) {
    float target;
    if (s < anneal_ramp_seconds) {
      float frac = (float)s / (float)anneal_ramp_seconds;
      target = anneal_start_temp + (ANNEAL_TEMP - anneal_start_temp) * frac;
    } else {
      target = ANNEAL_TEMP;
    }

    int x = secToXAnneal(s);
    int y = map(target, 0, 250, 210, 50);
    display.drawLine(prevX, prevY, x, y, RED);
    prevX = x; prevY = y; prevTarget = target;
  }
}

// Single-sweep anneal plotting (actual only; target is pre-rendered)
void updateAnnealGraph(float currentTemp, float /*targetTemp*/, unsigned long timeElapsedSec) {
  if (anneal_totalDuration <= 0) anneal_totalDuration = anneal_duration;
  if ((int)timeElapsedSec > anneal_totalDuration) timeElapsedSec = anneal_totalDuration;

  int   xCurr = secToXAnneal((int)timeElapsedSec);
  float yAct  = map(currentTemp, 0, 250, 210, 50);

  if (xCurr > anneal_lastX) {
    display.drawLine(anneal_lastX, anneal_lastY_actual, xCurr, yAct, GREEN);
    anneal_lastX        = xCurr;
    anneal_lastY_actual = yAct;
  }

  // Footer (we still show live target for numbers even though it's pre-drawn)
  float liveTarget;
  if ((int)timeElapsedSec < anneal_ramp_seconds) {
    float frac = (float)timeElapsedSec / (float)anneal_ramp_seconds;
    liveTarget = anneal_start_temp + (ANNEAL_TEMP - anneal_start_temp) * frac;
  } else {
    liveTarget = ANNEAL_TEMP;
  }

  display.fillRect(10, 220, 300, 20, BLACK);
  display.setTextSize(2);
  display.setCursor(10, 220);  display.setTextColor(GREEN); display.printf("%.1fC", currentTemp);
  display.setCursor(100, 220); display.setTextColor(RED);   display.printf("%.1fC", liveTarget);
  display.setCursor(190, 220); display.setTextColor(WHITE); display.printf("%lus/%ds", timeElapsedSec, anneal_totalDuration);
}

void drawSplashScreen() {
  display.fillScreen(BLACK);
  display.draw16bitRGBBitmap(0, 0, Untitled_design_18, 320, 250);
  delay(2000);
}

void createReflowProfile() {
  int total_duration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;
  int L = min(profile_length, total_duration);

  // Preheat
  for (int i = 0; i < preheat_duration && i < L; i++) {
    reflow_profile[i] = PREHEAT_START_TEMP +
      ((PREHEAT_END_TEMP - PREHEAT_START_TEMP) * i / (float)preheat_duration);
  }
  // Soak
  for (int i = preheat_duration; i < preheat_duration + soak_duration && i < L; i++) {
    reflow_profile[i] = SOAK_TEMP;
  }
  // Reflow
  for (int i = preheat_duration + soak_duration;
       i < preheat_duration + soak_duration + reflow_duration && i < L; i++) {
    reflow_profile[i] = REFLOW_START_TEMP +
      ((PEAK_TEMP - REFLOW_START_TEMP) *
       (i - (preheat_duration + soak_duration)) / (float)reflow_duration);
  }
  // Peak
  for (int i = preheat_duration + soak_duration + reflow_duration;
       i < preheat_duration + soak_duration + reflow_duration + peak_duration && i < L; i++) {
    reflow_profile[i] = PEAK_TEMP;
  }
  // Cooling: fill the rest (up to L)
  int coolStart = preheat_duration + soak_duration + reflow_duration + peak_duration;
  for (int i = coolStart; i < L; i++) {
    float frac = (float)(i - coolStart) / (float)max(1, cooling_duration);
    if (frac > 1.0f) frac = 1.0f;
    reflow_profile[i] = PEAK_TEMP - (PEAK_TEMP - COOLING_END_TEMP) * frac;
  }
}

void drawReflowProfileLine() {
  int total_duration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;
  for (int i = 1; i < profile_length; i++) {
    int x1 = map(i - 1, 0, total_duration, 10, 310);
    int x2 = map(i,     0, total_duration, 10, 310);
    int y1 = map(reflow_profile[i - 1], 0, 250, 210, 50);
    int y2 = map(reflow_profile[i],     0, 250, 210, 50);
    display.drawLine(x1, y1, x2, y2, RED);
  }
}

void drawGraphAxes() {
  display.drawFastVLine(10, 50, 160, WHITE);
  display.drawFastHLine(10, 210, 301, WHITE);
}

void drawVerticalBars() {
  int total_duration = preheat_duration + soak_duration + reflow_duration + peak_duration + cooling_duration;
  drawDottedLine(map(preheat_duration, 0, total_duration, 10, 310), 50, 210, GREY);
  drawDottedLine(map(preheat_duration + soak_duration, 0, total_duration, 10, 310), 50, 210, GREY);
  drawDottedLine(map(preheat_duration + soak_duration + reflow_duration, 0, total_duration, 10, 310), 50, 210, GREY);
  drawDottedLine(map(preheat_duration + soak_duration + reflow_duration + peak_duration, 0, total_duration, 10, 310), 50, 210, GREY);
}

void drawDottedLine(int x, int y_start, int y_end, uint16_t color) {
  for (int y = y_start; y < y_end; y += 4) display.drawPixel(x, y, color);
}

void drawLegend() {
  display.setTextColor(RED);   display.setTextSize(1); display.setCursor(15, 45); display.print("Target");
  display.setTextColor(GREEN); display.setCursor(60, 45); display.print("Actual");
}

void clearOldPlot() { display.fillRect(0, 0, 320, 213, BLACK); }