//AIOilGauge >>> All-In-One-Oil-Gauge

//test all (buzzer, output, drag, sensor input, settings)
//make pcb, there are 2 versions of the sensor?
//0-100 option?, display main, display settings padding

// ----- User Settings ----- (350, loads that if not saved)
int alarmTemp = 130; //above this temperature is alarm
float alarmPress = 1.0; //below this pressure is alarm
bool useImperial = false; //true for imperial, false for metric
bool usePSI = false; //true for PSI, false for bar
bool invertOutput = false; //true for pressureOutput = normally floating
bool buzzerEna = true; //enable or disable
bool dragEna = false; //enable or disable

//GPS Serial1 = TX16, RX15
//ADC I2C = SDA8, SCL9 via level shifter
//TFT SPI = MOSI11, CLK12, CS10, DC13, RST14
//BOSCH = TempA0+4.7k5v, PressA1
int pressureOutput = 15; //pressure alarm output pin, normally grounded
int buzzerOutput = 4;
int settA = 6, settB = 7;
int BL_PIN = 5;

// ----- Code -----
#include <TFT_eSPI.h>
#include <SPI.h>
#include "Free_Fonts.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SparkFunLSM6DS3.h>
#include <TinyGPS++.h>
#include <Preferences.h>

Preferences preferences;
TinyGPSPlus gps;
Adafruit_ADS1115 ads;
TFT_eSPI tft = TFT_eSPI();
LSM6DS3 myIMU;

uint16_t colorTemp = 0x0000, colorPress = 0x0000, dragTime = 0, valupdate = 0, dragStart = 0, color;
bool gpsReady = false, lastOPval = false;
long tempVal = 140, speedVal = 0, DnowSpeed = 0, DlastSpeed = 0, lastTemp = 0, bright = 100,lastSpeed = -10;
float pressVal = 0, fG = -5.0, sG = -5.0, lastPress = 0;

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
//Map function for float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Screen dimming
void TFT_SET_BL(uint8_t Value) {
  if (Value < 0 || Value > 100) {
    printf("TFT_SET_BL Error \r\n");
  } else {
    ledcWrite(BL_PIN, Value * 2.55);
  }
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

String getStringTime() {
  if(dragTime == 0) {
    return " None ";
  }
  unsigned long seconds = dragTime / 1000;
  int decimal = (dragTime % 1000) / 10; // Extract 2 decimal places
  return String(seconds) + "." + (decimal < 10 ? "0s" : "s") + String(decimal);
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
          tone(buzzerOutput,6262,1000);
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
          tone(buzzerOutput,6262,1000);
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
int lastDot[2] = {0,0};
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
  // V_out = V_supply * (R_sensor / (R_sensor + R_pullup))
  float sensorResistance = pullupResistor * (supplyVoltage / ads.computeVolts(ads.readADC_SingleEnded(0)) - 1);
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
  preferences.putInt("bright",bright);
  preferences.putULong("dragTime",dragTime);
  preferences.end();
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
  //Load settings
  preferences.begin("settings", true); //preferences.putUInt("counter", counter);
  alarmTemp = preferences.getInt("aTemp",130);
  alarmPress = preferences.getFloat("aPress",1.0);
  useImperial = preferences.getBool("uImp",false);
  usePSI = preferences.getBool("uPSI",false);
  invertOutput = preferences.getBool("invrt",false);
  buzzerEna = preferences.getBool("eBuzz",false);
  bright = preferences.getInt("bright",100);
  dragTime = preferences.getULong("dragTime",0);
  preferences.end();
  //Brightness pin
  ledcAttach(BL_PIN, 25000, 8);
  ledcWrite(BL_PIN, bright);
  //Init gps
  Serial1.begin(115200);
  //Init TFT
  TFT_SET_BL(bright);
  tft.begin();
  tft.setRotation(3); //Adjust rotation if needed
  //Init ADC
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115!");
    drawCenterText(120, 120, 1, TFT_WHITE, TFT_BLACK, "ADC ERROR");
    while (1); //Halt if initialization fails
  } else {
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(RATE_ADS1115_860SPS);
    Wire.setClock(800000);
  }
  if(myIMU.begin()) {
    Serial.println("[E] An Error has occurred while connecting to LSM!");
  }
  drawUI();
}

void loop(void) {
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

  if(gpsReady && dragEna) {
    DnowSpeed = gps.speed.kmph();
    if(DlastSpeed < 2 && DnowSpeed >= 2 && dragStart == 0) {
      dragStart = millis();
    }
    if(dragStart != 0 && ((useImperial)?(DnowSpeed>=60):(DnowSpeed>=100))) {
      unsigned long currentDrag = millis()-dragStart;
      if(currentDrag < dragTime) {
        dragTime = currentDrag;
        preferences.begin("settings", false);
        preferences.putULong("dragTime",dragTime);
        preferences.end();
      }
      //display currentDrag & best //new best! for x amount before clearing and redrawing
      dragStart = 0;
    }
    if (dragStart != 0 && (millis()-dragStart) > 30000) {
      dragStart = 0;
    }
    DlastSpeed = DnowSpeed;
  }

  //Update values every 100ms
  if((millis()-valupdate)>100) {
    //unsigned long drawTime = millis();
    tempVal = getTemp(useImperial);
    pressVal = round(getPress(usePSI) * 10) / 10.0;
    speedVal = getSpeed(useImperial);
    fG = myIMU.readFloatAccelX();
    sG = myIMU.readFloatAccelY();
    drawG(fG,sG);
    drawTemp(tempVal,useImperial);
    drawPress(pressVal,usePSI);
    drawSpeed(speedVal);
    /*Serial.println("Temp: "+String(tempVal));
    Serial.println("Press: "+String(pressVal,2));
    Serial.println("Speed: "+String(speedVal));
    Serial.println("Sats.: "+String(gps.satellites.value()));
    Serial.println("------------");*/
    hue = (hue + 1) % 256;
    color = hsvToRgb(hue);
    tft.drawFastHLine(0, 120, 240, color);
    tft.drawFastVLine(120, 18, 222, color);
    //Serial.println(">T:"+String(millis()-drawTime));
    valupdate = millis();
  }

  //Settings menu
  if(digitalRead(settA) == LOW) { 
    bool loopVar = true, drawRun = true;
    unsigned long debounce = 0;
    int currentMenu = 0, textSide = 75, valueSide = 185, paddValue = 50, paddText = 142, downAmnt = 65;
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
              preferences.putULong("dragTime",dragTime);
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
    tft.fillScreen(TFT_BLACK); //Force functions to draw again
    drawUI();
    lastTemp = -2;
    lastPress = -2;
    lastSpeed = -2;
    colorTemp = TFT_RED;
    colorPress = TFT_RED;
    TFT_SET_BL(bright);
  }
}
