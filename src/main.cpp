//AIOGauge >>> All-In-One-Oil-Gauge

/*
PCB fix:
- rotate screen connector 180°
- add resistor to temp (4.7k-5V)
- louder buzzer/tranzistor
- ESP32S3 without module + antenna (https://files.waveshare.com/wiki/ESP32-S3-Zero/ESP32-S3-Zero-Sch.pdf)
- relocate gps to back?
Write down sensor pinout to cable to PCB
*/

// ----- User Settings ----- (350, loads that if not saved)
int alarmTemp = 130; //above this temperature is alarm
float alarmPress = 1.0; //below this pressure is alarm
bool useImperial = false; //true for imperial, false for metric
bool usePSI = false; //true for PSI, false for bar
bool invertOutput = false; //true for pressureOutput = normally floating
bool buzzerEna = true; //enable or disable
bool dragEna = false; //enable or disable
bool tftEnabled = true; //physical TFT on/off (phone can be the screen)

//GPS Serial1 = ESP TX->17 (to GPS RXD), ESP RX<-18 (from GPS TXD)
//ADC I2C = SDA8, SCL9 via level shifter
//TFT SPI = MOSI11, CLK12, CS10, DC13, RST14
//BOSCH = TempA0+4.7k5v, PressA1
int pressureOutput = 15; //pressure alarm output pin, normally grounded
int buzzerOutput = 4;
int settA = 6, settB = 7;
int BL_PIN = 5;
int GPS_TX_PIN = 17; //ESP32 TX -> GPS RXD
int GPS_RX_PIN = 18; //ESP32 RX <- GPS TXD

//http://192.168.4.1/update for OTA, /log for logs
const char *otaSSID = "AIOGauge";
const char *otaPassword = "12345678";

// ----- Code -----
#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SparkFunLSM6DS3.h>
#include <TinyGPS++.h>
#include <Preferences.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ElegantOTA.h>
#include <LittleFS.h>

Preferences preferences;
TinyGPSPlus gps;
Adafruit_ADS1115 ads;
TFT_eSPI tft = TFT_eSPI();
LSM6DS3 myIMU(I2C_MODE, 0x6A);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

uint16_t colorTemp = 0x0000, colorPress = 0x0000, dragTime = 0, valupdate = 0, dragStart = 0, color;
bool gpsReady = false, lastOPval = false;
volatile bool tftPowerPending = false;
long tempVal = 140, speedVal = 0, DnowSpeed = 0, DlastSpeed = 0, lastTemp = 0, bright = 100,lastSpeed = -10;
float pressVal = 0, fG = -5.0, sG = -5.0, lastPress = 0;
int lastDot[2] = {175, 175};

// ----- Ride logging -----
#define RIDE_BUDGET_BYTES (400 * 1024)
#define RIDE_BUF_SAMPLES 64
#define LOG_SAMPLE_MS 500

struct __attribute__((packed)) LogSample {
  int16_t press_x100; // bar * 100
  int16_t temp_c;     // °C
  int16_t speed_kmh;  // km/h; -1 if GPS invalid
};

LogSample rideBuf[RIDE_BUF_SAMPLES];
size_t rideBufCount = 0;
fs::File rideFile;
uint16_t currentRideId = 0;
unsigned long logupdate = 0;
bool rideLogging = false;

// ----- Drag result overlay -----
unsigned long dragShowUntil = 0;
unsigned long dragShowCurrent = 0;
unsigned long dragShowBest = 0;
bool dragShowNewBest = false;

void drawCenterText(int centerX, int centerY, int textSize, uint16_t fColor, uint16_t bColor, String text);
void drawUI();
void TFT_SET_BL(uint8_t Value);
void applyTftPower();
void showDragOverlay();
String liveToJson();
String formatDragMs(unsigned long ms);

// ----- Temperature Sensor Parameters -----
const float pullupResistor = 4600.0; //Pull-up resistor (ohms)
const float supplyVoltage = 5.0; //Supply voltage for the sensor circuit

// Temperature/Resistance Lookup table
const int tableSize = 16;
float temperatureTable[tableSize] = { -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110 };
float resistanceTable[tableSize]  = { 44864, 25524, 15067, 9195, 5784, 3740, 2480, 1683, 1167, 824, 594, 434.9, 323.4, 244, 186.6, 144.5 };

// ----- Pressure Sensor Parameters -----
// Pressure sensor outputs 0.5V at 0 Bar and 4.5V at 10 Bar.
// Datasheet formula: Vout = (0.0008 * (pressure in kPa) + 0.1) * 5
// Solve for pressure (kPa): pressure_kPa = (Vout - 0.5) / 0.004
// Optionally, pressure in bar = pressure_kPa / 100

// Function to linearly interpolate the temperature based on sensor resistance
float interpolateTemperature(float resistance) {
  if (resistance>resistanceTable[0]) { //Check for out-of-range values
    return temperatureTable[0];
  }
  if (resistance<resistanceTable[tableSize-1]) {
    return temperatureTable[tableSize-1];
  }
  for (int i = 0; i < tableSize - 1; i++) { //Find the segment in the table
    if (resistance <= resistanceTable[i] && resistance >= resistanceTable[i + 1]) {
      float t1 = temperatureTable[i];
      float t2 = temperatureTable[i + 1];
      float r1 = resistanceTable[i];
      float r2 = resistanceTable[i + 1];
      return t1 + (t2 - t1) * ((resistance - r1) / (r2 - r1)); //Linear interpolation formula
    }
  }
  return NAN; //Return NaN if interpolation fails
}

// ----- Misc funcs -----
//realtime logger
#define LOG_BUFFER_SIZE (50 * 1024)  // 50 KB
char *logBuffer = nullptr;
size_t logIndex = 0;
void logPrint(const char *str) {
  Serial.println(str);
  if (!logBuffer) return;
  size_t len = strlen(str);
  size_t needed = len + 1; //newline
  if (logIndex + needed >= LOG_BUFFER_SIZE) {
    logIndex = 0;
    logBuffer[0] = '\0';
  }
  memcpy(logBuffer + logIndex, str, len);
  logIndex += len;
  logBuffer[logIndex++] = '\n';
  logBuffer[logIndex] = '\0';
}
inline void logPrint(const __FlashStringHelper *str) {
  logPrint((const char*)str);
}
void logPrint(const String &str) {
    logPrint(str.c_str());
}

//Map function for float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Screen dimming (0 = off, 1–100 = brightness %)
void TFT_SET_BL(uint8_t Value) {
  if (Value > 100) {
    printf("TFT_SET_BL Error \r\n");
  } else {
    ledcWrite(BL_PIN, (uint32_t)(Value * 2.55f));
  }
}

void invalidateGaugeCaches() {
  lastTemp = -2;
  lastPress = -2;
  lastSpeed = -2;
  colorTemp = TFT_RED;
  colorPress = TFT_RED;
  lastDot[0] = 175;
  lastDot[1] = 175;
}

void applyTftPower() {
  if (tftEnabled) {
    TFT_SET_BL((uint8_t)bright);
    if (dragShowUntil != 0) {
      showDragOverlay();
    } else {
      tft.fillScreen(TFT_BLACK);
      drawUI();
      invalidateGaugeCaches();
    }
  } else {
    TFT_SET_BL(0);
  }
}

// ----- GPS auto-baud detect & configure -----
//Waits up to timeoutMs on the CURRENT Serial1 baud rate for a checksum-validNMEA sentence.
bool gpsBaudIsValid(uint32_t timeoutMs) {
  TinyGPSPlus probe;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial1.available() > 0) {
      if (probe.encode(Serial1.read())) {
        return true; //got a full, checksum-valid sentence at this baud
      }
    }
  }
  return false;
}

//Sends the CASIC config strings to reconfigure the ATGM336H module.
void sendGPSConfig() {
  Serial1.println("$PCAS04,1*18"); //GPS only (no BDS/GLONASS)
  delay(100);
  Serial1.println("$PCAS11,3*1E"); //automotive mode
  delay(100);
  Serial1.println("$PCAS03,1,0,0,0,0,1,0,,,,,,,*32"); //only GGA+VTG output
  delay(100);
  Serial1.println("$PCAS02,100*1E"); //10Hz update rate
  delay(100);
  Serial1.println("$PCAS01,5*19"); //baud -> 115200 (module switches now)
  delay(200);
}

//Detects current module state and gets it (and our UART) onto 115200/10Hz/GPS-only.
void setupGPS() {
  //Assume normal operating state first: 115200
  Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
  Serial1.flush();
  while (Serial1.available()) { Serial1.read(); } //clear any boot garbage
  if (gpsBaudIsValid(1500)) {
    logPrint("GPS: already running at 115200, skipping reconfigure");
    return;
  }

  //Fell back to factory default 9600 (e.g. battery-backed RAM lost after low VBAT)
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
  Serial1.flush();
  while (Serial1.available()) { Serial1.read(); }
  if (gpsBaudIsValid(1500)) {
    logPrint("GPS: found at 9600 (default), sending reconfigure...");
    sendGPSConfig();
    Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(100);
    logPrint("GPS: reconfigured and switched to 115200");
    return;
  }

  //Neither responded (module still booting / not connected) - default to 115200
  logPrint("GPS: no response at 115200 or 9600, defaulting to 115200");
  Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

//RGB stuff
uint16_t hsvToRgb(uint8_t hue) {
    float r, g, b;
    float h = hue / 255.0 * 360.0;
    int i = h / 60;
    float f = (h / 60) - i;
    float q = 1 - f, t = f;
    switch (i % 6) {
      case 0: r = 1; g = t; b = 0; break;
      case 1: r = q; g = 1; b = 0; break;
      case 2: r = 0; g = 1; b = t; break;
      case 3: r = 0; g = q; b = 1; break;
      case 4: r = t; g = 0; b = 1; break;
      default: r = 1; g = 0; b = q; break;
    }
    return tft.color565(r * 255, g * 255, b * 255);
}
uint8_t hue = 0;

String formatDragMs(unsigned long ms) {
  if (ms == 0) {
    return "None";
  }
  unsigned long seconds = ms / 1000;
  int decimal = (ms % 1000) / 10;
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu.%02ds", seconds, decimal);
  return String(buf);
}

String getStringTime() {
  return formatDragMs(dragTime);
}

// ----- Ride logger -----
String ridePath(uint16_t id) {
  char buf[20];
  snprintf(buf, sizeof(buf), "/r/%04u.bin", id);
  return String(buf);
}

size_t totalRideBytes() {
  size_t total = 0;
  fs::File root = LittleFS.open("/r");
  if (!root || !root.isDirectory()) {
    return 0;
  }
  fs::File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      total += f.size();
    }
    f = root.openNextFile();
  }
  return total;
}

bool deleteOldestRide() {
  int oldest = -1;
  fs::File root = LittleFS.open("/r");
  if (!root || !root.isDirectory()) {
    return false;
  }
  fs::File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = f.name();
      int slash = name.lastIndexOf('/');
      if (slash >= 0) {
        name = name.substring(slash + 1);
      }
      int id = name.toInt();
      if (id > 0 && id != (int)currentRideId && (oldest < 0 || id < oldest)) {
        oldest = id;
      }
    }
    f = root.openNextFile();
  }
  if (oldest < 0) {
    return false;
  }
  String path = ridePath((uint16_t)oldest);
  if (LittleFS.remove(path)) {
    logPrint("Ride logger: deleted oldest " + path);
    return true;
  }
  return false;
}

void enforceRideBudget(size_t extraBytes) {
  while (totalRideBytes() + extraBytes > RIDE_BUDGET_BYTES) {
    if (!deleteOldestRide()) {
      break;
    }
  }
}

void flushLog() {
  if (!rideLogging || rideBufCount == 0 || !rideFile) {
    return;
  }
  size_t bytes = rideBufCount * sizeof(LogSample);
  enforceRideBudget(bytes);
  if (totalRideBytes() + bytes > RIDE_BUDGET_BYTES) {
    logPrint("Ride logger: budget full, dropping samples");
    rideBufCount = 0;
    return;
  }
  size_t written = rideFile.write((uint8_t *)rideBuf, bytes);
  rideFile.flush();
  if (written != bytes) {
    logPrint("Ride logger: short write");
  }
  rideBufCount = 0;
}

void logSample() {
  if (!rideLogging) {
    return;
  }
  LogSample s;
  float pressBar = usePSI ? (pressVal / 14.5038f) : pressVal;
  long tempC = useImperial ? (long)round((tempVal - 32) * 5.0 / 9.0) : tempVal;
  long speedKmh = (speedVal < 0) ? -1 : (useImperial ? (long)round(speedVal * 1.60934) : speedVal);
  s.press_x100 = (int16_t)round(pressBar * 100.0f);
  s.temp_c = (int16_t)tempC;
  s.speed_kmh = (int16_t)speedKmh;
  rideBuf[rideBufCount++] = s;
  if (rideBufCount >= RIDE_BUF_SAMPLES) {
    flushLog();
  }
}

void setupLogger() {
  if (!LittleFS.begin(true)) {
    logPrint("LittleFS mount failed");
    rideLogging = false;
    return;
  }
  if (!LittleFS.exists("/r")) {
    LittleFS.mkdir("/r");
  }
  if (!LittleFS.exists("/index.html")) {
    logPrint("WARNING: /index.html missing — run: pio run -t uploadfs");
  }
  if (!LittleFS.exists("/live.html")) {
    logPrint("WARNING: /live.html missing — run: pio run -t uploadfs");
  }

  preferences.begin("settings", false);
  currentRideId = preferences.getUShort("rideSeq", 0) + 1;
  if (currentRideId == 0) {
    currentRideId = 1;
  }
  preferences.putUShort("rideSeq", currentRideId);
  preferences.end();

  String path = ridePath(currentRideId);
  enforceRideBudget(0);
  rideFile = LittleFS.open(path, "a");
  if (!rideFile) {
    logPrint("Ride logger: failed to open " + path);
    rideLogging = false;
    return;
  }
  rideBufCount = 0;
  rideLogging = true;
  logPrint("Ride logger: started " + path);
}

void showDragOverlay() {
  if (!tftEnabled) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.setTextPadding(0);
  drawCenterText(120, 40, 1, TFT_WHITE, TFT_BLACK, useImperial ? "0-60 mph" : "0-100 km/h");
  if (dragShowNewBest) {
    drawCenterText(120, 70, 1, TFT_GREEN, TFT_BLACK, "NEW BEST!");
  }
  drawCenterText(120, 110, 1, TFT_LIGHTGREY, TFT_BLACK, "This run");
  tft.setFreeFont(FSSB24);
  drawCenterText(120, 145, 1, TFT_WHITE, TFT_BLACK, formatDragMs(dragShowCurrent));
  tft.setFreeFont(FSS9);
  drawCenterText(120, 185, 1, TFT_LIGHTGREY, TFT_BLACK, "Best: " + formatDragMs(dragShowBest));
}

void clearDragOverlayAndResume() {
  dragShowUntil = 0;
  if (!tftEnabled) {
    invalidateGaugeCaches();
    return;
  }
  tft.fillScreen(TFT_BLACK);
  drawUI();
  invalidateGaugeCaches();
}

// ----- Display draw stuff -----
//Draw test with center alignment
void drawCenterText(int centerX, int centerY, int textSize, uint16_t fColor, uint16_t bColor, String text) {
  tft.setTextSize(textSize);
  tft.setTextColor(fColor, bColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, centerX, centerY);
}

//Draw test with right alignment
void drawRFixText(int rightX, int centerY, int textSize, uint16_t fColor, uint16_t bColor, String text) {
  tft.setTextSize(textSize);
  tft.setTextColor(fColor, bColor);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(text, rightX, centerY);
}

void setOutputPress(bool val) { //true to ground with invert off
  if (lastOPval != val) {
    if(val) {
      if(invertOutput) {
        digitalWrite(pressureOutput, LOW);
      } else {
        digitalWrite(pressureOutput, HIGH);
      }
    } else {
      if(invertOutput) {
        digitalWrite(pressureOutput, HIGH);
      } else {
        digitalWrite(pressureOutput, LOW);
      }
    }
    lastOPval = val;
  }
}

// Alarm IO when TFT draws are skipped (display off / drag overlay)
void updateAlarmOutputs() {
  bool tempAlarm = tempVal > alarmTemp;
  bool pressAlarm = pressVal < alarmPress;
  setOutputPress(pressAlarm);
  static unsigned long lastBuzzSec = 0;
  unsigned long sec = millis() / 1000;
  if ((tempAlarm || pressAlarm) && buzzerEna && millis() > 10000 && (sec % 2) == 0 && lastBuzzSec != sec) {
    lastBuzzSec = sec;
    tone(buzzerOutput, 4000, 1000);
  }
}

//Draw speed value to display
void drawSpeed(long val) {
  if(lastSpeed != val) {
    tft.setFreeFont(FSSB24);
    tft.setTextPadding(80);
    if(val>=0) {
      drawRFixText(100, 170, 1, TFT_WHITE, TFT_BLACK, String(val));
    } else {
      drawRFixText(100, 170, 1, TFT_WHITE, TFT_BLACK, "###");
    }
  }
  lastSpeed = val;
}

//Draw temperature value to display
void drawTemp(long val, bool imper) {
  if(val > alarmTemp) {
    if((long)(millis()/1000)%2 == 0) { //Blink every other second red
      if(colorTemp != TFT_RED) {
        tft.fillRect(0, 16, 120, 104, TFT_RED);
        tft.setFreeFont(FSS9);
        tft.setTextPadding(0);
        if(imper) {
          drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_RED, "*F");
        } else {
          drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_RED, "*C");
        }
        drawCenterText(90, 22, 1, TFT_WHITE, TFT_RED, "Temp");
        colorTemp = TFT_RED;
        if(buzzerEna && millis()>10000) {
          tone(buzzerOutput,4000,1000);
        }
        lastTemp = -1;
      }
      if(lastTemp != val) {
        tft.setFreeFont(FSSB24);
        tft.setTextPadding(80);
        drawRFixText(100, 70, 1, TFT_WHITE, TFT_RED, String(val));
      }
    } else {
      if(colorTemp != TFT_WHITE) {
        tft.fillRect(0, 16, 120, 104, TFT_BLACK);
        tft.setFreeFont(FSS9);
        tft.setTextPadding(0);
        if(imper) {
          drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "*F");
        } else {
          drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "*C");
        }
        drawCenterText(90, 22, 1, TFT_WHITE, TFT_BLACK, "Temp");
        colorTemp = TFT_WHITE;
        lastTemp = -1;
      }
      if(lastTemp != val) {
        tft.setFreeFont(FSSB24);
        tft.setTextPadding(80);
        drawRFixText(100, 70, 1, TFT_WHITE, TFT_BLACK, String(val));
      }
    }
  } else {
    if(colorTemp != TFT_WHITE) {
      tft.fillRect(0, 16, 120, 104, TFT_BLACK);
      tft.setFreeFont(FSS9);
      tft.setTextPadding(0);
      if(imper) {
        drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "*F");
      } else {
        drawCenterText(90, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "*C");
      }
      drawCenterText(90, 22, 1, TFT_WHITE, TFT_BLACK, "Temp");
      colorTemp = TFT_WHITE;
    }
    if(lastTemp != val) {
      tft.setFreeFont(FSSB24);
      tft.setTextPadding(80);
      drawRFixText(100, 70, 1, TFT_WHITE, TFT_BLACK, String(val));
    }
  }
  lastTemp = val;
}

//Draw pressure value to display
void drawPress(float val, bool PSI) {
  if(val < alarmPress) {
    if((long)(millis()/1000)%2 == 0) { //Blink every other second red
      if(colorPress != TFT_RED) {
        tft.fillRect(121, 16, 119, 104, TFT_RED);
        tft.setFreeFont(FSS9);
        tft.setTextPadding(0);
        if(PSI) {
          drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_RED, "PSI");
        } else {
          drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_RED, "bar");
        }
        drawCenterText(149, 23, 1, TFT_WHITE, TFT_RED, "Press");
        colorPress = TFT_RED;
        if(buzzerEna && millis()>10000) {
          tone(buzzerOutput,4000,1000);
        }
        lastPress = -1;
      }
      if(lastPress != val) {
        tft.setFreeFont(FSSB24);
        tft.setTextPadding(80);
        drawCenterText(167, 70, 1, TFT_WHITE, TFT_RED, ((PSI)?String(val,0):String(val,1)));
        setOutputPress(true);
      }
    } else {
      if(colorPress != TFT_WHITE) {
        tft.fillRect(121, 16, 119, 104, TFT_BLACK);
        tft.setFreeFont(FSS9);
        tft.setTextPadding(0);
        if(PSI) {
          drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "PSI");
        } else {
          drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "bar");
        }
        drawCenterText(149, 23, 1, TFT_WHITE, TFT_BLACK, "Press");
        colorPress = TFT_WHITE;
        lastPress = -1;
      }
      if(lastPress != val) {
        tft.setFreeFont(FSSB24);
        tft.setTextPadding(80);
        drawCenterText(167, 70, 1, TFT_WHITE, TFT_BLACK, ((PSI)?String(val,0):String(val,1)));
        setOutputPress(true);
      }
    }
  } else {
    if(colorPress != TFT_WHITE) {
      tft.fillRect(121, 16, 119, 104, TFT_BLACK);
      tft.setFreeFont(FSS9);
      tft.setTextPadding(0);
      if(PSI) {
        drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "PSI");
      } else {
        drawCenterText(147, 108, 1, TFT_LIGHTGREY, TFT_BLACK, "bar");
      }
      drawCenterText(149, 23, 1, TFT_WHITE, TFT_BLACK, "Press");
      colorPress = TFT_WHITE;
    }
    if(lastPress != val) {
      tft.setFreeFont(FSSB24);
      tft.setTextPadding(80);
      drawCenterText(167, 70, 1, TFT_WHITE, TFT_BLACK, ((PSI)?String(val,0):String(val,1)));
      setOutputPress(false);
    }
  }
  lastPress = val;
}

//Draw G-force GUI to display
void drawG(float fG, float sG) {
  tft.fillCircle(lastDot[0],lastDot[1], 5, TFT_BLACK);
  tft.drawCircle(175, 175, 30, TFT_WHITE);
  tft.drawCircle(175, 175, 16, TFT_WHITE);
  lastDot[0] = 175+mapfloat(sG, -1.0, 1.0, -30, 30);
  lastDot[1] = 175+mapfloat(fG, -1.0, 1.0, -30, 30);
  tft.fillCircle(lastDot[0], lastDot[1], 5, TFT_RED);
}

// ----- Combined sensors funcs -----
// ----- Temperature Sensor on ADS1115 Channel 0 -----
long getTemp(bool imper) {
  float sensorResistance = pullupResistor / (supplyVoltage / ads.computeVolts(ads.readADC_SingleEnded(0)) - 1);
  //Interpolate & return temperature from the resistance value
  float sensorTemperature = interpolateTemperature(sensorResistance);
  if(imper) {
    return (long)((sensorTemperature * 9.0 / 5.0) + 32);
  } else {
    return (long)sensorTemperature;
  }
}

// ----- Pressure Sensor on ADS1115 Channel 1 -----
float getPress(bool PSI) {
  // pressure_kPa = (Vout - 0.5) / 0.004
  float tst = ads.readADC_SingleEnded(1);
  float pressure_kPa = (ads.computeVolts(tst) - 0.5) / 0.004;
  if(PSI) {
    return (round(pressure_kPa*1.45038)/10.0); //in PSI;
  } else {
    return (pressure_kPa / 100.0); //In bar
  }
}

// ----- GPS Speed -----
long getSpeed(bool imper) {
  if(gpsReady) {
    if(imper) {
      return (long)gps.speed.mph();
    } else {
      return (long)gps.speed.kmph();
    }
  } else {
    return -1;
  }
}

void saveSettings() {
  preferences.begin("settings", false);
  preferences.putInt("aTemp",alarmTemp);
  preferences.putFloat("aPress",alarmPress);
  preferences.putBool("uImp",useImperial);
  preferences.putBool("uPSI",usePSI);
  preferences.putBool("invrt",invertOutput);
  preferences.putBool("eBuzz",buzzerEna);
  preferences.putBool("eDrag",dragEna);
  preferences.putBool("tftOn",tftEnabled);
  preferences.putInt("bright",bright);
  preferences.putULong("dragTime",dragTime);
  preferences.end();
}

String settingsToJson() {
  String j = "{";
  j += "\"aTemp\":" + String(alarmTemp) + ",";
  j += "\"aPress\":" + String(alarmPress, 1) + ",";
  j += "\"uImp\":" + String(useImperial ? "true" : "false") + ",";
  j += "\"uPSI\":" + String(usePSI ? "true" : "false") + ",";
  j += "\"invrt\":" + String(invertOutput ? "true" : "false") + ",";
  j += "\"eBuzz\":" + String(buzzerEna ? "true" : "false") + ",";
  j += "\"eDrag\":" + String(dragEna ? "true" : "false") + ",";
  j += "\"tftOn\":" + String(tftEnabled ? "true" : "false") + ",";
  j += "\"bright\":" + String(bright) + ",";
  j += "\"dragTime\":" + String(dragTime);
  j += "}";
  return j;
}

String liveToJson() {
  bool tempAlarm = tempVal > alarmTemp;
  bool pressAlarm = pressVal < alarmPress;
  bool dragShow = dragShowUntil != 0;
  String j = "{";
  j += "\"temp\":" + String(tempVal) + ",";
  if (usePSI) {
    j += "\"press\":" + String(pressVal, 0) + ",";
  } else {
    j += "\"press\":" + String(pressVal, 1) + ",";
  }
  j += "\"speed\":" + String(speedVal) + ",";
  j += "\"gpsOk\":" + String(gpsReady ? "true" : "false") + ",";
  j += "\"fG\":" + String(fG, 3) + ",";
  j += "\"sG\":" + String(sG, 3) + ",";
  j += "\"aTemp\":" + String(alarmTemp) + ",";
  j += "\"aPress\":" + String(alarmPress, 1) + ",";
  j += "\"uImp\":" + String(useImperial ? "true" : "false") + ",";
  j += "\"uPSI\":" + String(usePSI ? "true" : "false") + ",";
  j += "\"tempAlarm\":" + String(tempAlarm ? "true" : "false") + ",";
  j += "\"pressAlarm\":" + String(pressAlarm ? "true" : "false") + ",";
  j += "\"drag\":{";
  j += "\"show\":" + String(dragShow ? "true" : "false") + ",";
  j += "\"title\":\"" + String(useImperial ? "0-60 mph" : "0-100 km/h") + "\",";
  j += "\"currentMs\":" + String(dragShowCurrent) + ",";
  j += "\"bestMs\":" + String(dragShowBest) + ",";
  j += "\"newBest\":" + String(dragShowNewBest ? "true" : "false");
  j += "}";
  j += "}";
  return j;
}

String ridesToJson() {
  String j = "[";
  bool first = true;
  fs::File root = LittleFS.open("/r");
  if (root && root.isDirectory()) {
    fs::File f = root.openNextFile();
    while (f) {
      if (!f.isDirectory()) {
        String name = f.name();
        int slash = name.lastIndexOf('/');
        if (slash >= 0) {
          name = name.substring(slash + 1);
        }
        int id = name.toInt();
        size_t bytes = f.size();
        size_t samples = bytes / sizeof(LogSample);
        float durationSec = samples * (LOG_SAMPLE_MS / 1000.0f);
        if (!first) {
          j += ",";
        }
        first = false;
        j += "{\"id\":" + String(id);
        j += ",\"samples\":" + String(samples);
        j += ",\"durationSec\":" + String(durationSec, 1);
        j += ",\"bytes\":" + String(bytes);
        j += ",\"current\":" + String(id == (int)currentRideId ? "true" : "false");
        j += "}";
      }
      f = root.openNextFile();
    }
  }
  j += "]";
  return j;
}

void setupWebServer() {
  logBuffer = (char*)malloc(LOG_BUFFER_SIZE + 1);
  if (logBuffer) {
    logBuffer[0] = '\0';
    logIndex = 0;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(otaSSID, otaPassword);
  logPrint("WiFi AP started: " + String(otaSSID) + " @ " + WiFi.softAPIP().toString());

  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      client->text(liveToJson());
    } else if (type == WS_EVT_DISCONNECT) { }
  });
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/live", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/live.html", "text/html");
  });

  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", logBuffer ? logBuffer : "(log buffer not allocated)");
  });

  server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", settingsToJson());
  });

  server.on("/api/live", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", liveToJson());
  });

  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("aTemp", true))
      alarmTemp = request->getParam("aTemp", true)->value().toInt();
    if (request->hasParam("aPress", true))
      alarmPress = request->getParam("aPress", true)->value().toFloat();
    if (request->hasParam("uImp", true))
      useImperial = request->getParam("uImp", true)->value() == "1" || request->getParam("uImp", true)->value() == "true";
    if (request->hasParam("uPSI", true))
      usePSI = request->getParam("uPSI", true)->value() == "1" || request->getParam("uPSI", true)->value() == "true";
    if (request->hasParam("invrt", true))
      invertOutput = request->getParam("invrt", true)->value() == "1" || request->getParam("invrt", true)->value() == "true";
    if (request->hasParam("eBuzz", true))
      buzzerEna = request->getParam("eBuzz", true)->value() == "1" || request->getParam("eBuzz", true)->value() == "true";
    if (request->hasParam("eDrag", true))
      dragEna = request->getParam("eDrag", true)->value() == "1" || request->getParam("eDrag", true)->value() == "true";
    if (request->hasParam("tftOn", true)) {
      bool next = request->getParam("tftOn", true)->value() == "1" || request->getParam("tftOn", true)->value() == "true";
      if (next != tftEnabled) {
        tftEnabled = next;
        tftPowerPending = true;
      }
    }
    if (request->hasParam("bright", true)) {
      bright = constrain(request->getParam("bright", true)->value().toInt(), 2, 100);
      if (tftEnabled) {
        tftPowerPending = true;
      }
    }
    saveSettings();
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/rides", HTTP_GET, [](AsyncWebServerRequest *request) {
    flushLog();
    request->send(200, "application/json", ridesToJson());
  });

  server.on("/api/ride", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("id")) {
      request->send(400, "text/plain", "missing id");
      return;
    }
    uint16_t id = (uint16_t)request->getParam("id")->value().toInt();
    bool reopen = false;
    if (id == currentRideId) {
      flushLog();
      if (rideFile) {
        rideFile.close();
        reopen = true;
      }
    }
    String path = ridePath(id);
    if (!LittleFS.exists(path)) {
      if (reopen) {
        rideFile = LittleFS.open(ridePath(currentRideId), "a");
      }
      request->send(404, "text/plain", "ride not found");
      return;
    }
    request->send(LittleFS, path, "application/octet-stream");
    if (reopen) {
      rideFile = LittleFS.open(ridePath(currentRideId), "a");
      if (!rideFile) {
        rideLogging = false;
        logPrint("Ride logger: failed to reopen after download");
      }
    }
  });

  ElegantOTA.begin(&server); //adds the /update page
  server.begin();
  logPrint("Web server + OTA ready");
}

void drawUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(FSS9);
  tft.setTextPadding(0);
  drawCenterText(118, 7, 1, TFT_WHITE, TFT_BLACK, "OIL");
  if(useImperial) {
    drawCenterText(90, 130, 1, TFT_LIGHTGREY, TFT_BLACK, "mph");
  } else {
    drawCenterText(90, 130, 1, TFT_LIGHTGREY, TFT_BLACK, "km/h");
  }
  drawCenterText(145, 130, 1, TFT_LIGHTGREY, TFT_BLACK, "G");
  drawCenterText(96, 224, 1, TFT_WHITE, TFT_BLACK, "GPS");
  drawCenterText(144, 224, 1, TFT_WHITE, TFT_BLACK, "Gyro");
}

// ----- Setup&Loop -----
void setup() {
  //Init own IO
  pinMode(pressureOutput, OUTPUT);
  pinMode(buzzerOutput, OUTPUT);
  pinMode(settA, INPUT_PULLUP);
  pinMode(settB, INPUT_PULLUP);
  pinMode(BL_PIN, OUTPUT);
  //Init debug serial
  Serial.begin(115200);
  //Init WiFi AP + OTA update page + realtime log page (needs Serial for logPrint's debug echo)
  setupWebServer();
  //Load settings
  preferences.begin("settings", true); //preferences.putUInt("counter", counter);
  alarmTemp = preferences.getInt("aTemp",130);
  alarmPress = preferences.getFloat("aPress",1.0);
  useImperial = preferences.getBool("uImp",false);
  usePSI = preferences.getBool("uPSI",false);
  invertOutput = preferences.getBool("invrt",false);
  buzzerEna = preferences.getBool("eBuzz",false);
  dragEna = preferences.getBool("eDrag",false);
  bright = preferences.getInt("bright",100);
  dragTime = preferences.getULong("dragTime",0);
  tftEnabled = preferences.getBool("tftOn", true);
  preferences.end();
  //Mount LittleFS + open this boot's ride log
  setupLogger();
  //Brightness pin
  ledcAttach(BL_PIN, 25000, 8);
  //Init/configure gps
  setupGPS();
  //Init TFT
  tft.begin();
  tft.setRotation(2); //Adjust rotation if needed
  applyTftPower();
  //Init ADC
  if (!ads.begin()) {
    logPrint("Failed to initialize ADS1115!");
    if (tftEnabled) {
      drawCenterText(120, 120, 1, TFT_WHITE, TFT_BLACK, "ADC ERROR");
    }
    while (1); //Halt if initialization fails
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1115_860SPS);
    Wire.setClock(800000);
  }
  if(myIMU.begin()) {
    logPrint("[E] An Error has occurred while connecting to LSM!");
  }
  if (tftEnabled) {
    drawUI();
  }
  logupdate = millis();
  valupdate = millis();
}

void loop(void) {
  ElegantOTA.loop(); //services pending OTA update/reboot requests

  if (tftPowerPending) {
    tftPowerPending = false;
    applyTftPower();
  }

  //Feed gps
  while(Serial1.available() > 0) {
    if(gps.encode(Serial1.read())) {
      if(!gps.location.isValid() || gps.location.age() > 2000 || gps.satellites.value() < 2) {
        gpsReady = false;
      } else {
        gpsReady = true;
      }
    }
  }

  // Clear drag overlay after 5s and resume gauges
  if (dragShowUntil != 0 && (long)(millis() - dragShowUntil) >= 0) {
    clearDragOverlayAndResume();
  }

  if(gpsReady && dragEna && dragShowUntil == 0) {
    DnowSpeed = gps.speed.kmph();
    if(DlastSpeed < 2 && DnowSpeed >= 2 && dragStart == 0) {
      dragStart = millis();
    }
    if(dragStart != 0 && ((useImperial)?(DnowSpeed>=60):(DnowSpeed>=100))) {
      unsigned long currentDrag = millis()-dragStart;
      bool isNewBest = (dragTime == 0 || currentDrag < dragTime);
      if(isNewBest) {
        dragTime = (uint16_t)currentDrag;
        preferences.begin("settings", false);
        preferences.putULong("dragTime",dragTime);
        preferences.end();
      }
      dragShowCurrent = currentDrag;
      dragShowBest = dragTime;
      dragShowNewBest = isNewBest;
      dragShowUntil = millis() + 5000;
      showDragOverlay();
      dragStart = 0;
    }
    if (dragStart != 0 && (millis()-dragStart) > 30000) {
      dragStart = 0;
    }
    DlastSpeed = DnowSpeed;
  }

  //Update values every 100ms (skip gauge draws while drag result is on screen or TFT off)
  if((millis()-valupdate)>100) {
    tempVal = getTemp(useImperial);
    pressVal = round(getPress(usePSI) * 10) / 10.0;
    speedVal = getSpeed(useImperial);
    fG = myIMU.readFloatAccelZ();
    sG = myIMU.readFloatAccelX();
    if (tftEnabled && dragShowUntil == 0) {
      drawG(fG,sG);
      drawTemp(tempVal,useImperial);
      drawPress(pressVal,usePSI);
      drawSpeed(speedVal);
      hue = (hue + 1) % 256;
      color = hsvToRgb(hue);
      tft.drawFastHLine(0, 120, 240, color);
      tft.drawFastVLine(120, 18, 222, color);
    } else if (!tftEnabled) {
      updateAlarmOutputs();
    }
    if (ws.count() > 0) {
      ws.textAll(liveToJson());
    }
    ws.cleanupClients();
    valupdate = millis();
  }

  //Log SI samples every 0.5s; flush buffer about every 8s
  if ((millis() - logupdate) >= LOG_SAMPLE_MS) {
    logSample();
    static uint8_t flushTicks = 0;
    if (++flushTicks >= 16) { // 16 * 0.5s = 8s
      flushLog();
      flushTicks = 0;
    }
    logupdate = millis();
  }

  //Settings menu
  if(digitalRead(settA) == LOW) { 
    bool loopVar = true, drawRun = true;
    unsigned long debounce = 0;
    int currentMenu = 0, textSide = 75, valueSide = 185, paddValue = 50, paddText = 142, downAmnt = 65;
    if (!tftEnabled) {
      TFT_SET_BL((uint8_t)bright); //temporarily light so menu is visible
    }
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFreeFont(FSS9);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Settings",120,20);
    if(useImperial) {
      tft.drawString("0-60mph: "+getStringTime(),120,40);
    } else {
      tft.drawString("0-100kph: "+getStringTime(),120,40);
    }
    while(digitalRead(settA) == LOW){delay(10);};
    while(loopVar) {
      if(currentMenu == 0) { //Temp alarm trigger point
        if(digitalRead(settB) == LOW) {
          debounce = millis();
          while(digitalRead(settB) == LOW) {
            if(millis()>debounce+1000) {
              alarmTemp++;
              if(useImperial) {
                if(alarmTemp > 300) {
                  alarmTemp = 80;
                }
              } else {
                if(alarmTemp > 150) {
                  alarmTemp = 80;
                }
              }
              tft.drawNumber(alarmTemp, valueSide, downAmnt+(17*currentMenu));
              delay(20);
            }
          }
          alarmTemp++;
          if(useImperial) {
            if(alarmTemp > 300) {
              alarmTemp = 80;
            }
          } else {
            if(alarmTemp > 150) {
              alarmTemp = 80;
            }
          }
          tft.drawNumber(alarmTemp, valueSide, downAmnt+(17*currentMenu));
          delay(10);
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Temp Alarm:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Press Alarm: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawFloat(alarmPress, 1, valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 1) { //Press alarm trigger point
        if(digitalRead(settB) == LOW) {
          debounce = millis();
          while(digitalRead(settB) == LOW) {
            if(millis()>debounce+1000) {
              if(usePSI) {
                alarmPress+=1;
                if(alarmPress > 30) {
                  alarmPress = 0;
                }
              } else {
                alarmPress+=0.1;
                if(alarmPress > 3.0) {
                  alarmPress = 0.0;
                }
              }
              tft.drawFloat(alarmPress, 1, valueSide, downAmnt+(17*currentMenu));
              delay(20);
            }
          }
          if(usePSI) {
            alarmPress+=10;
            if(alarmPress > 300) {
              alarmPress = 0;
            }
          } else {
            alarmPress+=0.1;
            if(alarmPress > 3.0) {
              alarmPress = 0.0;
            }
          }
          tft.drawFloat(alarmPress, 1, valueSide, downAmnt+(17*currentMenu));
          delay(10);
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Press Alarm:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Use PSI: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawString(((usePSI)?"True":"False"), valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 2) { //bar/PSI
        if(digitalRead(settB) == LOW) {
          usePSI = !usePSI;
          tft.drawString(((usePSI)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settB) == LOW){delay(10);};
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Use PSI:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Use imperial: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawString(((useImperial)?"True":"False"), valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 3) { //imperial/metric
        if(digitalRead(settB) == LOW) {
          useImperial = !useImperial;
          tft.drawString(((useImperial)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settB) == LOW){delay(10);};
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Use imperial:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Invert output: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawString(((invertOutput)?"True":"False"), valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 4) { //invert output
        if(digitalRead(settB) == LOW) {
          invertOutput = !invertOutput;
          tft.drawString(((invertOutput)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settB) == LOW){delay(10);};
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Invert output:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Alarm buzzer: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawString(((buzzerEna)?"True":"False"), valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 5) { //buzzer
        if(digitalRead(settB) == LOW) {
          buzzerEna = !buzzerEna;
          tft.drawString(((buzzerEna)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settB) == LOW){delay(10);};
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Alarm buzzer:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Drag mode: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawString(((dragEna)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 6) { //drag
        if(digitalRead(settB) == LOW) {
          debounce = millis();
          while(digitalRead(settB) == LOW) {
            if(millis()>debounce+1000) {
              preferences.begin("settings", false);
              preferences.putULong("dragTime",0);
              preferences.end();
              tft.drawString("RESET", valueSide, downAmnt+(17*currentMenu));
              if(useImperial) {
                tft.drawString("0-60mph: "+getStringTime(),120,40);
              } else {
                tft.drawString("0-100kph: "+getStringTime(),120,40);
              }
              delay(200);
            }
          }
          dragEna = !dragEna;
          tft.drawString(((dragEna)?"True":"False"), valueSide, downAmnt+(17*currentMenu));
          delay(10);
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Drag mode:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Brightness: <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          tft.drawNumber(bright, valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 7) { //Screen brightness
        if(digitalRead(settB) == LOW) {
          debounce = millis();
          while(digitalRead(settB) == LOW) {
            if(millis()>debounce+1000) {
              bright+=2;
              if(bright > 100) {
                bright = 2;
              }
              tft.drawNumber(bright, valueSide, downAmnt+(17*currentMenu));
              delay(20);
            }
          }
          bright+=2;
          if(bright > 100) {
            bright = 2;
          }
          tft.drawNumber(bright, valueSide, downAmnt+(17*currentMenu));
          delay(10);
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Brightness:", textSide, downAmnt+(17*(currentMenu)));
          currentMenu++;
          tft.drawString("> Save <", textSide, downAmnt+(17*(currentMenu)));
          tft.setTextPadding(paddValue);
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 8) { //save
        if(digitalRead(settB) == LOW) {
          tft.setTextPadding(paddValue);
          tft.drawString("......", valueSide, downAmnt+(17*currentMenu));
          saveSettings();
          delay(200);
          tft.drawString("Saved", valueSide, downAmnt+(17*currentMenu));
          while(digitalRead(settB) == LOW){delay(10);};
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Save", textSide, downAmnt+(17*currentMenu));
          currentMenu++;
          tft.drawString("> Exit <", 120, downAmnt+(17*currentMenu));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
        }
      }
      if(currentMenu == 9) { //exit
        if(digitalRead(settB) == LOW) {
          loopVar = false;
        }
        if(digitalRead(settA) == LOW || drawRun) {
          tft.setTextPadding(paddText);
          tft.drawString("Exit", 120, downAmnt+(17*currentMenu));
          currentMenu = 0;
          tft.drawString("> Temp Alarm: <", textSide, downAmnt+(17*currentMenu));
          tft.setTextPadding(paddValue);
          tft.drawNumber(alarmTemp, valueSide, downAmnt+(17*(currentMenu)));
          while(digitalRead(settA) == LOW && !drawRun){delay(10);};
          drawRun = false;
        }
      }
    }
    invalidateGaugeCaches();
    applyTftPower();
  }
}
