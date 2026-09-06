#include <Arduino.h>
#include <Wire.h>
#include <Tasker.h>
#include "Configuration.h"
#include "temp_sensor.h"
#include "dist_sensor.h"
#include "display.h"
#include "ble.h"
#include "algo.h"
#if (BOARD == BOARD_ESP32_FEATHER)
#include "Adafruit_MAX1704X.h"
#elif (BOARD == BOARD_M5STICKS3)
#include <M5Unified.h>
#include <Preferences.h>
M5Canvas lcdCanvas(&M5.Display);
M5Canvas settingsCanvas(&M5.Display);
Preferences m5Preferences;
#endif

TempSensor tempSensor;
DistSensor distSensor;
uint8_t mirrorTire = 0;
char wheelPos[] = "  ";  // Wheel position for Tire A
char deviceNameSuffix[] = "  ";

#if (BOARD == BOARD_ESP32_FEATHER)
Adafruit_MAX17048 maxlipo;
#endif

#if (BOARD == BOARD_ESP32_LOLIND32)
  #if (FIS_SENSOR2_PRESENT == 1)
    TempSensor tempSensor2;
    uint8_t mirrorTire2 = 0;
    char wheelPos2[] = "  ";  // Wheel position for Tire B
  #endif
  #if (DIST_SENSOR2 != DIST_NONE)
    DistSensor distSensor2;
  #endif
#endif

BLDevice bleDevice;
Display display;
Tasker tasker;

int vBattery = 0;          // Current battery voltage in mV
int lipoPercentage = 0;    // Current battery percentage
float updateRate = 0.0;    // Reflects the actual update rate
int measurementCycles = 0; // Counts how many measurement cycles were completed. printStatus() uses it to roughly calculate refresh rate.

// Function declarations
void updateWheelPos(void);
void printStatus(void);
void blinkOnTempChange(int16_t);
void blinkOnDistChange(uint16_t);
int getVbat(void);
void updateBattery(void);
void updateRefreshRate(void);
#if (BOARD == BOARD_M5STICKS3)
void updateM5StickDisplay(void);
uint16_t thermalColor(int16_t temperature, int16_t minimum, int16_t maximum);
void updateM5DisplayRotation(void);
#endif

#if (BOARD == BOARD_M5STICKS3)
const unsigned long LCD_STARTUP_DURATION = 5000;
const unsigned long LCD_SINGLE_CLICK_DURATION = 30000;
const unsigned long LCD_ACTIVE_CLICK_INCREMENT = 100000;
const unsigned long LCD_REFRESH_INTERVAL = 200;
unsigned long lcdOffAt = 0;
unsigned long lastLcdRefresh = 0;
uint8_t lcdSelection = 0;
char wheelPositionBeforeSettings[3] = "  ";
#endif

#ifdef DUMMYDATA
  #include "dummydata.h"
#endif


// ----------------------------------------

void setup(){
#if (BOARD == BOARD_M5STICKS3)
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  m5Preferences.begin("rubbertrack", false);
  lcdOffAt = millis() + LCD_STARTUP_DURATION;
  M5.Display.setBrightness(128);
#else
  Serial.begin(115200);
  delay(1000);
  if (Serial){
    delay(3000); // Wait for Serial
  }
#endif
  Serial.printf("\nBegin startup. Arduino version: %d\n", ARDUINO);

#if (BOARD == BOARD_ESP32_FEATHER) || (BOARD == BOARD_ESP32_LOLIND32) || (BOARD == BOARD_M5STICKS3)
  Serial.printf("ESP32 IDF version: %s\n", esp_get_idf_version());
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
#endif

#ifdef DUMMYDATA
  debug("=======  DUMMYDATA  ========\n");
#endif

  if (GPIOLEDDIST >= 0) pinMode(GPIOLEDDIST, OUTPUT);
  if (GPIOLEDTEMP >= 0) pinMode(GPIOLEDTEMP, OUTPUT);
  if (GPIODISTSENSORXSHUT >= 0) pinMode(GPIODISTSENSORXSHUT, OUTPUT);
  if (GPIOLEFT >= 0) pinMode(GPIOLEFT, INPUT_PULLUP);
  if (GPIOFRONT >= 0) pinMode(GPIOFRONT, INPUT_PULLUP);
  if (GPIOCAR >= 0) pinMode(GPIOCAR, INPUT_PULLUP);
  if (GPIOMIRR >= 0) pinMode(GPIOMIRR, INPUT_PULLUP);
#if (BOARD == BOARD_ESP32_LOLIND32)
  if (GPIOMIRR2 >= 0) pinMode(GPIOMIRR2, INPUT_PULLUP);
  if (GPIOUNUSEDA2 >= 0) pinMode(GPIOUNUSEDA2, INPUT);
  if (GPIOUNUSEDB1 >= 0) pinMode(GPIOUNUSEDB1, INPUT);
#endif

  updateWheelPos();
  String savedWheelPos = m5Preferences.getString("wheel", "");
  if (savedWheelPos == "FL" || savedWheelPos == "FR" || savedWheelPos == "RL" || savedWheelPos == "RR") {
    strcpy(wheelPos, savedWheelPos.c_str());
    strcpy(deviceNameSuffix, wheelPos);
  }
  mirrorTire = m5Preferences.getUChar("mirror", mirrorTire);
  updateM5DisplayRotation();
  char bleName[32] = "RejsaRubber";
  sprintf(bleName, "%s%s\0",bleName, deviceNameSuffix); // Extend bleName[] with the suffix


// TIRE 1 MIRRORED?
  if (MIRRORTIRE == 1 || (GPIOMIRR >= 0 && digitalRead(GPIOMIRR) == 0)) {
    mirrorTire = 1;
    debug("Temperature sensor orientation for %s is mirrored.\n", wheelPos);
  }

// I2C channel 1
  #if (BOARD == BOARD_ESP32_FEATHER) || (BOARD == BOARD_ESP32_LOLIND32) || (BOARD == BOARD_M5STICKS3)
    Wire.begin(GPIOSDA,GPIOSCL); // initialize I2C w/ I2C pins from config
  #else
    Wire.begin();
  #endif

  #if (DIST_SENSOR != DIST_NONE)
    debug("Starting distance sensor for %s...\n", wheelPos);
    if (distSensor.initialise(&Wire, wheelPos)) {
      debug("Distance sensor for %s present.\n", wheelPos);
    }
    else {
      debug("ERROR: Distance sensor for %s not present.\n", wheelPos);
    }
  #endif

  debug("Starting temperature sensor for %s...\n", wheelPos);
  if (!tempSensor.initialise(FIS_REFRESHRATE, &Wire)) {
    // perform automatic system reboot to retry temp sensor initialization
    #if (BOARD == BOARD_ESP32_FEATHER) || (BOARD == BOARD_ESP32_LOLIND32) || (BOARD == BOARD_M5STICKS3)
      debug("Rebooting the MCU now...\n");
      ESP.restart();
    #elif (BOARD == BOARD_NRF52_FEATHER)
      debug("Rebooting the MCU now...\n");
      NVIC_SystemReset();
    #endif
  }

#if (BOARD == BOARD_ESP32_LOLIND32)
  // I2C channel 2
  #if (FIS_SENSOR2_PRESENT == 1)

    // TIRE 2 MIRRORED?
    if ((MIRRORTIRE2 == 1 || digitalRead(GPIOMIRR2) == 0)) {
      mirrorTire2 = 1;
      debug("Temperature sensor 2 orientation for %s is mirrored.\n", wheelPos2);
    }

    Wire1.begin(GPIOSDA2,GPIOSCL2); // initialize I2C w/ I2C pins from config

    #if (DIST_SENSOR2 != DIST_NONE)
      debug("Starting distance sensor 2 for %s...\n", wheelPos2);
      if (distSensor2.initialise(&Wire1, wheelPos2)) {
        debug("Distance sensor 2 for %s present.\n", wheelPos2);
      }
      else {
        debug("ERROR: Distance sensor 2 for %s not present.\n", wheelPos2);
      }
    #endif

    debug("Starting temperature sensor 2 for %s...\n", wheelPos2);
    if (!tempSensor2.initialise(FIS_REFRESHRATE, &Wire1)) {
      // perform automatic system reboot to retry temp sensor initialization
      #if (BOARD == BOARD_ESP32_FEATHER) || (BOARD == BOARD_ESP32_LOLIND32)
        debug("Rebooting the MCU now...\n");
        ESP.restart();
      #endif
    }
  #else
    // set unused I2C pins to input mode (just to be sure)
    pinMode(GPIOSDA2, INPUT);
    pinMode(GPIOSCL2, INPUT);
  #endif
#endif

#if (BOARD == BOARD_ESP32_FEATHER)
  debug(F("\nStarting battery monitor:MAX17048"));

  while (!maxlipo.begin()) {
    debug(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!\n"));
    delay(2000);
  }
  debug("Found MAX17048 with Chip ID: 0x%x.\n",maxlipo.getChipID());
  delay(2000);
#endif
updateBattery();

// display
#if (DISP_DEVICE != DISP_NONE)
  display.setup();
  tasker.setInterval(updateDisplay,200);
#endif

// Status LED
#if (STATUS_LED != STATUS_LED_SENSOR)
  tasker.setInterval(updateLED,100);
#endif

// BLE
  debug("Starting BLE device: %s\n", bleName);
  bleDevice.setupDevice(bleName);

// Set up periodic functions
#ifdef _DEBUG
  tasker.setInterval(printStatus, SERIAL_UPDATERATE*1000); // Print status every second
#endif
  tasker.setInterval(updateBattery, BATTERY_UPDATERATE*1000); // Update battery status every minute
  tasker.setInterval(updateRefreshRate, 2000);

  debug("Running!\n");

#ifdef DUMMYDATA
  // 2do: make DUMMYDATA compatible with 2x I2C
  dummyloop();
#endif
}

void loop() {
#if (BOARD == BOARD_M5STICKS3)
  M5.update();
  if (M5.BtnB.wasPressed()) {
    uint8_t previousSelection = lcdSelection;
    lcdSelection = (lcdSelection + 1) % 3;
    if (previousSelection == 0 && lcdSelection == 1) {
      strcpy(wheelPositionBeforeSettings, wheelPos);
    }
    if (previousSelection != 0 && lcdSelection == 0 &&
        strcmp(wheelPositionBeforeSettings, wheelPos) != 0) {
      m5Preferences.putString("wheel", wheelPos);
      debug("Tire position changed to %s. Restarting to refresh BLE and sensor setup.\n", wheelPos);
      delay(100);
      ESP.restart();
    }
    lcdOffAt = millis() + 60000;
    M5.Display.setBrightness(128);
  }
  if (M5.BtnA.wasPressed()) {
    if (lcdSelection == 1) {
      if (strcmp(wheelPos, "FL") == 0) {
        strcpy(wheelPos, "FR");
      } else if (strcmp(wheelPos, "FR") == 0) {
        strcpy(wheelPos, "RL");
      } else if (strcmp(wheelPos, "RL") == 0) {
        strcpy(wheelPos, "RR");
      } else {
        strcpy(wheelPos, "FL");
      }
      strcpy(deviceNameSuffix, wheelPos);
      m5Preferences.putString("wheel", wheelPos);
      updateM5DisplayRotation();
    } else if (lcdSelection == 2) {
      mirrorTire = !mirrorTire;
      m5Preferences.putUChar("mirror", mirrorTire);
      updateM5DisplayRotation();
    } else {
      if (lcdOffAt == 0) {
        lcdOffAt = millis() + LCD_SINGLE_CLICK_DURATION;
      } else {
        lcdOffAt += LCD_ACTIVE_CLICK_INCREMENT;
      }
      M5.Display.setBrightness(128);
    }
    if (lcdSelection != 0) {
      lcdOffAt = millis() + 60000;
      M5.Display.setBrightness(128);
    }
  }
  if (lcdOffAt != 0) {
    M5.Display.setBrightness(128);
  }
  if (lcdOffAt != 0 && (long)(millis() - lcdOffAt) >= 0) {
    lcdOffAt = 0;
    M5.Display.setBrightness(0);
  }
#endif

// I2C channel 1
  #if (DIST_SENSOR != DIST_NONE)
    distSensor.measure();
  #endif
  tempSensor.measure();

// I2C channel 2
#if (FIS_SENSOR2_PRESENT == 1)
  #if (DIST_SENSOR2 != DIST_NONE)
    distSensor2.measure();
  #endif
  tempSensor2.measure();
#endif

  if (bleDevice.isConnected()) {
    bleDevice.transmit(tempSensor.measurement_16, mirrorTire, distSensor.distance, vBattery, lipoPercentage);
  }

#if (BOARD == BOARD_M5STICKS3)
  updateM5StickDisplay();
#endif

  #if (DISP_DEVICE == DISP_NONE) // Only use the LEDs w/o display
    blinkOnTempChange(tempSensor.measurement_16[8]/20);    // Use one single temp in the middle of the array
    blinkOnDistChange(distSensor.distance/20);    // value/nn -> Ignore smaller changes to prevent noise triggering blinks
  #endif

// 2do: integrate tempSensor2 & distSensor2 into BLE transmission
// 2do: integrate Wheelpost into BLE transmission

  measurementCycles++;

  tasker.loop();
}

#if (BOARD == BOARD_M5STICKS3)
void updateM5StickDisplay(void) {
  if (lcdOffAt == 0 || millis() - lastLcdRefresh < LCD_REFRESH_INTERVAL) return;

  lastLcdRefresh = millis();
  int16_t minimum = tempSensor.image[0];
  int16_t maximum = tempSensor.image[0];
  for (uint16_t index=1; index<FIS_X * FIS_Y; index++) {
    if (tempSensor.image[index] < minimum) minimum = tempSensor.image[index];
    if (tempSensor.image[index] > maximum) maximum = tempSensor.image[index];
  }
  if (maximum == minimum) maximum++;

  int displayWidth = M5.Display.width();
  int displayHeight = M5.Display.height();
  int settingsWidth = 80;
  int imageAreaWidth = displayWidth - settingsWidth;
  int cellSize = min(imageAreaWidth / FIS_X, displayHeight / FIS_Y);
  int cellWidth = cellSize;
  int cellHeight = cellSize;
  int imageWidth = cellWidth * FIS_X;
  int imageHeight = cellHeight * FIS_Y;
  int imageX = (imageAreaWidth - imageWidth) / 2;
  int imageY = (displayHeight - imageHeight) / 2;
  bool reverseLayout = (strcmp(wheelPos, "FR") == 0 || strcmp(wheelPos, "RR") == 0) != (mirrorTire != 0);

  lcdCanvas.fillScreen(BLACK);
  for (uint8_t y=0; y<FIS_Y; y++) {
    for (uint8_t x=0; x<FIS_X; x++) {
      uint8_t sourceX = reverseLayout ? FIS_X - 1 - x : x;
      uint8_t sourceY = reverseLayout ? FIS_Y - 1 - y : y;
      lcdCanvas.fillRect(imageX + x * cellWidth, imageY + y * cellHeight, cellWidth, cellHeight,
                         thermalColor(tempSensor.image[sourceY * FIS_X + sourceX], minimum, maximum));
    }
  }
  int targetY = imageY + IGNORE_TOP_ROWS * cellHeight;
  int targetHeight = EFFECTIVE_ROWS * cellHeight;
  lcdCanvas.drawRect(imageX, targetY, imageWidth, targetHeight, RED);
  lcdCanvas.drawRect(imageX + 1, targetY + 1, imageWidth - 2, targetHeight - 2, RED);
  lcdCanvas.drawRect(imageX + 2, targetY + 2, imageWidth - 4, targetHeight - 4, RED);

  int settingsX = imageAreaWidth;
  bool reverseText = strcmp(wheelPos, "FR") == 0 || strcmp(wheelPos, "RR") == 0;
  settingsCanvas.setRotation(reverseText ? 2 : 0);
  settingsCanvas.fillScreen(BLACK);
  settingsCanvas.setTextColor(WHITE, BLACK);
  settingsCanvas.setTextSize(2);
  settingsCanvas.setCursor(4, 4);
  settingsCanvas.printf("%d%%", lipoPercentage);
  settingsCanvas.setCursor(4, 76);
  if (lcdSelection == 2) settingsCanvas.fillRect(4, 74, 76, 20, WHITE);
  settingsCanvas.setTextColor(lcdSelection == 2 ? RED : WHITE, lcdSelection == 2 ? WHITE : BLACK);
  settingsCanvas.print("INSIDE");
  settingsCanvas.setTextColor(WHITE, BLACK);
  const char* tirePositions[] = {"FL", "FR", "RL", "RR"};
  const int tireX[] = {4, 44, 4, 44};
  const int tireY[] = {20, 20, 48, 48};
  for (uint8_t tireIndex=0; tireIndex<4; tireIndex++) {
    bool isCurrent = strcmp(wheelPos, tirePositions[tireIndex]) == 0;
    bool isSelected = lcdSelection == 1 && isCurrent;
    if (isSelected) {
      settingsCanvas.fillRect(tireX[tireIndex], tireY[tireIndex], 32, 20, WHITE);
    }
    settingsCanvas.setTextColor(isCurrent ? RED : WHITE, isSelected ? WHITE : BLACK);
    settingsCanvas.setTextSize(2);
    settingsCanvas.setCursor(tireX[tireIndex], tireY[tireIndex]);
    settingsCanvas.print(tirePositions[tireIndex]);
  }

  unsigned long remainingSeconds = (lcdOffAt - millis() + 999) / 1000;
  lcdCanvas.pushSprite(0, 0);
  settingsCanvas.setTextColor(WHITE, BLACK);
  settingsCanvas.setCursor(4, displayHeight - 24);
  settingsCanvas.printf("%lus", remainingSeconds);
  settingsCanvas.pushSprite(settingsX, 0, BLACK);
}

void updateM5DisplayRotation(void) {
  bool reverseWheel = strcmp(wheelPos, "FR") == 0 || strcmp(wheelPos, "RR") == 0;
  uint8_t rotation = 3;
  if (reverseWheel) rotation = (rotation + 2) % 4;
  if (mirrorTire) rotation = (rotation + 2) % 4;
  M5.Display.setRotation(rotation);
  lcdCanvas.deleteSprite();
  lcdCanvas.createSprite(M5.Display.width(), M5.Display.height());
  settingsCanvas.deleteSprite();
  settingsCanvas.createSprite(80, M5.Display.height());
}

uint16_t thermalColor(int16_t temperature, int16_t minimum, int16_t maximum) {
  uint16_t level = (uint32_t)(temperature - minimum) * 255 / (maximum - minimum);
  uint8_t red;
  uint8_t green;
  uint8_t blue;

  if (level < 85) {
    red = 0;
    green = level * 3;
    blue = 255 - level * 2;
  } else if (level < 170) {
    red = (level - 85) * 3;
    green = 255;
    blue = 85 - (level - 85);
  } else {
    red = 255;
    green = 255 - (level - 170) * 3;
    blue = 0;
  }
  return M5.Display.color565(red, green, blue);
}
#endif

void updateDisplay(void) {
  display.refreshDisplay(tempSensor.measurement, tempSensor.outerTireEdgePositionSmoothed, tempSensor.innerTireEdgePositionSmoothed, tempSensor.validAutozoomFrame, updateRate, distSensor.distance, lipoPercentage, bleDevice.isConnected());

// 2do: integrate tempSensor2 & distSensor2 for display
}

void updateLED(void) {
  static uint8_t sd_cnt = 0;

  sd_cnt++;
  if( sd_cnt % 30 == 0)
  {
    if (GPIOLEDDIST >= 0) digitalWrite(GPIOLEDDIST, HIGH);
#if (BOARD == BOARD_ESP32_FEATHER)
    neopixelWrite(RGB_BUILTIN,255,255,255);
#else
    if (GPIOLEDTEMP >= 0) digitalWrite(GPIOLEDTEMP, HIGH);
#endif
    sd_cnt = 0;
  }
  else {
    if (GPIOLEDDIST >= 0) digitalWrite(GPIOLEDDIST, LOW);
    if (GPIOLEDTEMP >= 0) digitalWrite(GPIOLEDTEMP, LOW);
  }
}

// Figure out wheel position coding
void updateWheelPos(void) {
#if (FIS_SENSOR2_PRESENT == 1)
  uint8_t leftPinVal = (GPIOLEFT >= 0) ? digitalRead(GPIOLEFT) : 1;
  uint8_t frontPinVal = (GPIOFRONT >= 0) ? digitalRead(GPIOFRONT) : 1;
  uint8_t carPinVal = (GPIOCAR >= 0) ? digitalRead(GPIOCAR) : 1;

  if (leftPinVal) {
    // GPIOLEFT  = 0 => axis: both sensors are mounted on the front or rear axle
    wheelPos[1]  = 'L';
    wheelPos2[1] = 'R';

    if (frontPinVal) {
      // GPIOFRONT = 0 => axis 1 (front)
      wheelPos[0]  = 'F';
      wheelPos2[0] = 'F';
      deviceNameSuffix[0] = 'F';
    } else {
      // GPIOFRONT = 1 => axis 2 (rear)
      wheelPos[0]  = 'R';
      wheelPos2[0] = 'R';
      deviceNameSuffix[0] = 'R';
    }
  }
  else {
    // GPIOLEFT  = 1 => axis: axis: both sensors are mounted on the left or right vehicle side
    wheelPos[0]  = 'F';
    wheelPos2[0] = 'R';

    if (frontPinVal) {
      // GPIOFRONT = 0 => axis 1 (left)
      wheelPos[1]  = 'L';
      wheelPos2[1] = 'L';
      deviceNameSuffix[0] = 'L';
    } else {
      // GPIOFRONT = 1 => axis 2 (right)
      wheelPos[1]  = 'R';
      wheelPos2[1] = 'R';
      deviceNameSuffix[0] = 'R';
    }
  }

  // overwrite left/right if we happen to be a motorcycle
  if (!carPinVal)  wheelPos[1]  = ' ';
  if (!carPinVal)  wheelPos2[1] = ' ';

#else
  uint8_t leftVal = (GPIOLEFT >= 0) ? digitalRead(GPIOLEFT) : 1;
  uint8_t frontVal = (GPIOFRONT >= 0) ? digitalRead(GPIOFRONT) : 1;
  uint8_t carVal = (GPIOCAR >= 0) ? digitalRead(GPIOCAR) : 1;
  uint8_t wheelPosCode = leftVal + (frontVal << 1) + (carVal << 2);
  if (wheelPosCode >= 7) wheelPosCode = DEVICENAMECODE; // set from configuration

  switch (wheelPosCode) {
    case 0: sprintf(wheelPos, "FL"); break;
    case 1: sprintf(wheelPos, "FR"); break;
    case 2: sprintf(wheelPos, "RL"); break;
    case 3: sprintf(wheelPos, "RR"); break;
    case 4: sprintf(wheelPos, "F "); break;
    case 5: sprintf(wheelPos, "F "); break;
    case 6: sprintf(wheelPos, "R "); break;
    case 7: sprintf(wheelPos, "  "); break;
    default: sprintf(wheelPos, "??"); break;
  }
  strncpy(deviceNameSuffix, wheelPos, 3);
#endif
}

void printStatus(void) {
#if (DIST_SENSOR != DIST_NONE)
  char distSensor_str[6];
  sprintf(distSensor_str, "%imm", distSensor.distance);
#else
  const char* distSensor_str = "N/A  ";
#endif
#if (DIST_SENSOR2 != DIST_NONE)
  char distSensor2_str[6];
  sprintf(distSensor2_str, "%imm", distSensor2.distance);
#else
  const char* distSensor2_str = "N/A  ";
#endif


  debug("Rate: %.1fHz\tV: %dmV (%d%%) \tWheel: %s\tD: %s\t", (float)updateRate, vBattery, lipoPercentage, wheelPos, distSensor_str);
  debug("Zoomrate: %.2f%% \tOutliers: %.2f%%\tMaxRowDelta: %.1f\tAvgTemp: %.1f\tAvgStdDev: %.1f\t", tempSensor.runningAvgZoomedFramesRate*100, tempSensor.runningAvgOutlierRate*100, tempSensor.maxRowDeltaTmp/10, tempSensor.movingAvgFrameTmp/10, tempSensor.movingAvgStdDevFrameTmp/10);
  for (uint8_t i=0; i<FIS_X; i++) {
    debug("T: %.1f\t",(float)tempSensor.measurement[i]/10);
  }
  debug("\n");
#if (FIS_SENSOR2_PRESENT == 1)
  debug("\t\t\t\t\tWheel: %s\tD: %s\t", wheelPos2, distSensor2_str);
  debug("Zoomrate: %.2f%% \tOutliers: %.2f%%\tMaxRowDelta: %.1f\tAvgTemp: %.1f\tAvgStdDev: %.1f\t", tempSensor2.runningAvgZoomedFramesRate*100, tempSensor2.runningAvgOutlierRate*100, tempSensor2.maxRowDeltaTmp/10, tempSensor2.movingAvgFrameTmp/10, tempSensor2.movingAvgStdDevFrameTmp/10);
  for (uint8_t i=0; i<FIS_X; i++) {
    debug("T: %.1f\t",(float)tempSensor2.measurement[i]/10);
  }
  debug("\n");
#endif
}

void updateRefreshRate(void) {
  static long lastUpdate = 0;
  updateRate = (float)measurementCycles / (millis()-lastUpdate) * 1000;
  lastUpdate = millis();
  measurementCycles = 0;
}

#if (DISP_DEVICE == DISP_NONE)
void blinkOnDistChange(uint16_t distnew) {
#if (STATUS_LED == STATUS_LED_SENSOR)
  if (GPIOLEDDIST >= 0) {
    static uint16_t distold = 0;

/* DEPRECATED?
    if (!Bluefruit.connected()) {
      digitalWrite(ledDist, HIGH);
      return;
    }
*/

    if (distold != distnew) {
      digitalWrite(GPIOLEDDIST, HIGH);
      delay(3);
      digitalWrite(GPIOLEDDIST, LOW);
    }
    distold = distnew;
  }
#endif
}

void blinkOnTempChange(int16_t tempnew) {
#if (STATUS_LED == STATUS_LED_SENSOR)
  if (GPIOLEDTEMP >= 0) {
    static int16_t tempold = 0;
    if (tempold != tempnew) {
#if (BOARD == BOARD_ESP32_FEATHER)
      neopixelWrite(RGB_BUILTIN,0,0,255);
#else
      digitalWrite(GPIOLEDTEMP, HIGH);
#endif
      delay(3);
      digitalWrite(GPIOLEDTEMP, LOW);
    }
    tempold = tempnew;
  }
#endif
}
#endif

int getVbat(void) {
  double adcRead=0;
#if (BOARD == BOARD_ESP32_LOLIND32) // Compensation for ESP32's crappy ADC -> https://bitbucket.org/Blackneron/esp32_adc/src/master/
  const double f1 = 1.7111361460487501e+001;
  const double f2 = 4.2319467860421662e+000;
  const double f3 = -1.9077375643188468e-002;
  const double f4 = 5.4338055402459246e-005;
  const double f5 = -8.7712931081088873e-008;
  const double f6 = 8.7526709101221588e-011;
  const double f7 = -5.6536248553232152e-014;
  const double f8 = 2.4073049082147032e-017;
  const double f9 = -6.7106284580950781e-021;
  const double f10 = 1.1781963823253708e-024;
  const double f11 = -1.1818752813719799e-028;
  const double f12 = 5.1642864552256602e-033;

  const int loops = 5;
  const int loopDelay = 1;

  int counter = 1;
  int inputValue = 0;
  double totalInputValue = 0;
  double averageInputValue = 0;
  for (counter = 1; counter <= loops; counter++) {
    inputValue = analogRead(VBAT_PIN);
    totalInputValue += inputValue;
    delay(loopDelay);
  }
  averageInputValue = totalInputValue / loops;
  adcRead = f1 + f2 * pow(averageInputValue, 1) + f3 * pow(averageInputValue, 2) + f4 * pow(averageInputValue, 3) + f5 * pow(averageInputValue, 4) + f6 * pow(averageInputValue, 5) + f7 * pow(averageInputValue, 6) + f8 * pow(averageInputValue, 7) + f9 * pow(averageInputValue, 8) + f10 * pow(averageInputValue, 9) + f11 * pow(averageInputValue, 10) + f12 * pow(averageInputValue, 11);
#elif (BOARD == BOARD_NRF52_FEATHER)
  adcRead = analogRead(VBAT_PIN);
#endif
  return adcRead * MILLIVOLTFULLSCALE * BATRESISTORCOMP / STEPSFULLSCALE;
}

void updateBattery(void) {
#if (BOARD == BOARD_ESP32_LOLIND32) || (BOARD == BOARD_NRF52_FEATHER)
  vBattery = getVbat();
  lipoPercentage = lipoPercent(vBattery);

#elif (BOARD == BOARD_ESP32_FEATHER)
  float f_mVolt;
  f_mVolt = maxlipo.cellVoltage() * 1000.0; // V to mV
  if (isnan(f_mVolt)) {
    return;
  }

  vBattery = (int)f_mVolt;
  lipoPercentage = maxlipo.cellPercent();
  if(lipoPercentage < 0)
  {
    lipoPercentage = 0;
  }
  else if( 100.0 < lipoPercentage)
  {
    lipoPercentage = 100.0;
  }
#elif (BOARD == BOARD_M5STICKS3)
  M5.update();
  vBattery = M5.Power.getBatteryVoltage();
  lipoPercentage = M5.Power.getBatteryLevel();
  if (lipoPercentage < 0) {
    lipoPercentage = 0;
  } else if (lipoPercentage > 100) {
    lipoPercentage = 100;
  }
#endif
}
