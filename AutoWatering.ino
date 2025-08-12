#include <Wire.h>
#include <Adafruit_BME280.h>
#include <I2CLiquidCrystal.h>
#include <EEPROM.h>

#define I2C_SCL 5
#define I2C_SDA 4
#define I2C_ADDR 0x3c
#define BRIGHT 127
#define TRIG_PIN 8
#define ECHO_PIN 9
#define LED_PIN 7
#define RELAY_PIN 13
#define BUZZER_PIN 10
#define MOISTURE_PIN A0
#define LIGHT_SENSOR_PIN A1
#define STATUS_LED_RED 6
#define STATUS_LED_GREEN 12
#define STATUS_LED_BLUE 11
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define BEAT_FAST 150
#define BEAT_NORMAL 300
#define BEAT_SLOW 600
#define ONION_TEMP_MIN 15.0
#define ONION_TEMP_MAX 25.0
#define ONION_TEMP_IDEAL 20.0
#define ONION_HUMIDITY_MIN 50.0
#define ONION_HUMIDITY_MAX 70.0
#define ONION_MOISTURE_MIN 400
#define ONION_MOISTURE_MAX 700
#define ONION_MOISTURE_CRITICAL 300
#define ONION_LIGHT_MIN 300
#define ONION_WATERING_DURATION 3000
#define ONION_DRAIN_TIME 10000
#define SENSOR_READ_INTERVAL 5000
#define WATER_CHECK_DISTANCE 15
#define MOISTURE_READINGS_COUNT 5
#define ALERT_TIMEOUT 300000
#define EEPROM_CONFIG_ADDR 0

Adafruit_BME280 bme;
I2CLiquidCrystal oled(I2C_ADDR, (uint8_t)BRIGHT);

unsigned long lastSensorRead = 0;
unsigned long lastWateringTime = 0;
unsigned long lastAlertTime = 0;
unsigned long systemStartTime = 0;
unsigned long ralphTimer1 = 0;
unsigned long ralphTimer2 = 0;

float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
int moistureLevel = 0;
int lightLevel = 0;
float waterDistance = 0.0;
bool waterTankEmpty = false;
int ralphCounter1 = 0;
int ralphCounter2 = 0;

int consecutiveDryReadings = 0;
int dailyWateringCount = 0;
bool plantStressAlert = false;
bool environmentalAlert = false;
bool ralphFlag1 = false;
bool ralphFlag2 = false;

enum SystemState {
  INITIALIZING,
  MONITORING,
  WATERING,
  ALERT,
  MAINTENANCE
};
SystemState currentState = INITIALIZING;
SystemState ralphPreviousState = INITIALIZING;

struct Config {
  int moistureThreshold;
  int wateringDuration;
  int alertThreshold;
  bool soundEnabled;
} config;

void setup() {
  Serial.begin(9600);
  systemStartTime = millis();
  ralphTimer1 = millis();
  initializePins();
  loadConfiguration();
  Wire.begin();
  if (!initializeBME280()) {
    criticalError("BME280 initialization failed!");
    return;
  }
  if (!initializeOLED()) {
    criticalError("OLED initialization failed!");
    return;
  }
  playStartupMelody();
  displaySystemInfo();
  currentState = MONITORING;
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readAllSensors();
    lastSensorRead = currentTime;
  }
  switch (currentState) {
    case MONITORING:
      monitoringState();
      break;
    case WATERING:
      wateringState();
      break;
    case ALERT:
      alertState();
      break;
    case MAINTENANCE:
      maintenanceState();
      break;
  }
  updateDisplay();
  checkCriticalConditions();
  updateStatusLEDs();
  handleSerialCommands();
  delay(100);
}

void initializePins() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_BLUE, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
}

bool initializeBME280() {
  int attempts = 0;
  while (!bme.begin(0x76) && attempts < 5) {
    delay(1000);
    attempts++;
  }
  if (attempts >= 5) {
    return false;
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF);
  return true;
}

bool initializeOLED() {
  oled.begin(16, 2);
  delay(100);
  oled.display();
  oled.clear();
  oled.noBlink();
  oled.noCursor();
  oled.home();
  oled.print("Red Onion Care");
  oled.setCursor(0, 1);
  oled.print("System Starting...");
  delay(2000);
  return true;
}

void loadConfiguration() {
  EEPROM.get(EEPROM_CONFIG_ADDR, config);
  if (config.moistureThreshold < 200 || config.moistureThreshold > 800) {
    config.moistureThreshold = ONION_MOISTURE_CRITICAL;
  }
  if (config.wateringDuration < 1000 || config.wateringDuration > 10000) {
    config.wateringDuration = ONION_WATERING_DURATION;
  }
  if (config.alertThreshold < 1 || config.alertThreshold > 10) {
    config.alertThreshold = 3;
  }
  config.soundEnabled = true;
}

void saveConfiguration() {
  EEPROM.put(EEPROM_CONFIG_ADDR, config);
}

void readAllSensors() {
  readBME280Data();
  readMoistureLevel();
  readLightLevel();
  checkWaterTankLevel();
  logSensorData();
}

void readBME280Data() {
  bme.takeForcedMeasurement();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  if (isnan(temperature) temperature = 0.0;
  if (isnan(humidity)) humidity = 0.0;
}

void readMoistureLevel() {
  long moistureSum = 0;
  int validReadings = 0;
  for (int i = 0; i < MOISTURE_READINGS_COUNT; i++) {
    int reading = analogRead(MOISTURE_PIN);
    if (reading >= 0 && reading <= 1023) {
      moistureSum += reading;
      validReadings++;
    }
    delay(50);
  }
  moistureLevel = validReadings > 0 ? moistureSum / validReadings : 0;
}

void readLightLevel() {
  lightLevel = analogRead(LIGHT_SENSOR_PIN);
  if (lightLevel < 0 || lightLevel > 1023) lightLevel = 0;
}

void checkWaterTankLevel() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration > 0) {
    waterDistance = (duration * 0.034) / 2;
    waterTankEmpty = (waterDistance > WATER_CHECK_DISTANCE);
  } else {
    waterDistance = 999;
    waterTankEmpty = true;
  }
}

void monitoringState() {
  if (needsWatering() && !waterTankEmpty) {
    currentState = WATERING;
    consecutiveDryReadings++;
    return;
  }
  if (isPlantStressed() && millis() - lastAlertTime > ALERT_TIMEOUT) {
    currentState = ALERT;
    plantStressAlert = true;
  }
  if (isEnvironmentSuboptimal() && millis() - lastAlertTime > ALERT_TIMEOUT) {
    currentState = ALERT;
    environmentalAlert = true;
  }
  if (moistureLevel > config.moistureThreshold) consecutiveDryReadings = 0;
}

void wateringState() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Watering Onions");
  oled.setCursor(0, 1);
  oled.print("Please wait...");
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  if (config.soundEnabled) playWateringMelody();
  delay(config.wateringDuration);
  digitalWrite(RELAY_PIN, LOW);
  oled.setCursor(0, 1);
  oled.print("Draining...     ");
  delay(ONION_DRAIN_TIME);
  digitalWrite(LED_PIN, LOW);
  lastWateringTime = millis();
  dailyWateringCount++;
  if (config.soundEnabled) playCompletionMelody();
  displayWateringComplete();
  currentState = MONITORING;
}

void alertState() {
  if (plantStressAlert) handlePlantStressAlert();
  if (environmentalAlert) handleEnvironmentalAlert();
  lastAlertTime = millis();
  plantStressAlert = false;
  environmentalAlert = false;
  currentState = MONITORING;
}

void maintenanceState() {
  currentState = MONITORING;
}

bool needsWatering() {
  bool moistureLow = (moistureLevel < config.moistureThreshold);
  bool notRecentlyWatered = (millis() - lastWateringTime > 3600000);
  bool temperatureHigh = (temperature > ONION_TEMP_IDEAL + 3.0);
  bool humidityLow = (humidity < ONION_HUMIDITY_MIN);
  if (moistureLow && notRecentlyWatered) {
    if (temperatureHigh || humidityLow) return true;
    if (consecutiveDryReadings >= config.alertThreshold) return true;
  }
  return false;
}

bool isPlantStressed() {
  bool temperatureStress = (temperature < ONION_TEMP_MIN || temperature > ONION_TEMP_MAX);
  bool moistureStress = (moistureLevel < ONION_MOISTURE_CRITICAL);
  bool lightStress = (lightLevel < ONION_LIGHT_MIN);
  bool excessiveWatering = (dailyWateringCount > 6);
  return temperatureStress || moistureStress || lightStress || excessiveWatering;
}

bool isEnvironmentSuboptimal() {
  bool temperatureSuboptimal = (temperature < ONION_TEMP_MIN + 2 || temperature > ONION_TEMP_MAX - 2);
  bool humiditySuboptimal = (humidity < ONION_HUMIDITY_MIN + 5 || humidity > ONION_HUMIDITY_MAX + 5);
  return temperatureSuboptimal || humiditySuboptimal;
}

void handlePlantStressAlert() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("PLANT STRESS!");
  oled.setCursor(0, 1);
  if (temperature < ONION_TEMP_MIN || temperature > ONION_TEMP_MAX) {
    oled.print("Check Temp!");
  } else if (moistureLevel < ONION_MOISTURE_CRITICAL) {
    oled.print("Very Dry Soil!");
  } else if (lightLevel < ONION_LIGHT_MIN) {
    oled.print("Need More Light!");
  } else if (dailyWateringCount > 6) {
    oled.print("Too Much Water!");
  }
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED_RED, HIGH);
    if (config.soundEnabled) tone(BUZZER_PIN, NOTE_G4, 200);
    delay(200);
    digitalWrite(STATUS_LED_RED, LOW);
    delay(200);
  }
  delay(3000);
}

void handleEnvironmentalAlert() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Environment");
  oled.setCursor(0, 1);
  oled.print("Suboptimal");
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_RED, HIGH);
    digitalWrite(STATUS_LED_GREEN, HIGH);
    if (config.soundEnabled) tone(BUZZER_PIN, NOTE_E4, 300);
    delay(300);
    digitalWrite(STATUS_LED_RED, LOW);
    digitalWrite(STATUS_LED_GREEN, LOW);
    delay(300);
  }
  delay(2000);
}

void criticalError(const char* message) {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("SYSTEM ERROR!");
  oled.setCursor(0, 1);
  oled.print("Check Serial");
  while (true) {
    digitalWrite(STATUS_LED_RED, HIGH);
    digitalWrite(STATUS_LED_GREEN, HIGH);
    digitalWrite(STATUS_LED_BLUE, HIGH);
    tone(BUZZER_PIN, NOTE_C5, 100);
    delay(100);
    digitalWrite(STATUS_LED_RED, LOW);
    digitalWrite(STATUS_LED_GREEN, LOW);
    digitalWrite(STATUS_LED_BLUE, LOW);
    delay(100);
  }
}

void updateDisplay() {
  static unsigned long lastDisplayUpdate = 0;
  static int displayPage = 0;
  if (millis() - lastDisplayUpdate < 2000) return;
  oled.clear();
  switch (displayPage) {
    case 0:
      oled.setCursor(0, 0);
      oled.print(String(temperature, 1) + "C ");
      oled.print(String(humidity, 1) + "%");
      oled.setCursor(0, 1);
      oled.print("Moist: " + String(moistureLevel));
      break;
    case 1:
      oled.setCursor(0, 0);
      oled.print("Onion Status:");
      oled.setCursor(0, 1);
      oled.print(isPlantStressed() ? "Needs Care!" : "Healthy :)");
      break;
    case 2:
      oled.setCursor(0, 0);
      oled.print("Light: " + String(lightLevel));
      oled.setCursor(0, 1);
      oled.print(waterTankEmpty ? "Tank: Empty!" : "Tank: OK");
      break;
    case 3:
      oled.setCursor(0, 0);
      oled.print("Watered: " + String(dailyWateringCount) + "x");
      oled.setCursor(0, 1);
      unsigned long uptime = (millis() - systemStartTime) / 1000;
      oled.print("Up: " + String(uptime / 3600) + "h");
      break;
  }
  displayPage = (displayPage + 1) % 4;
  lastDisplayUpdate = millis();
}

void displaySystemInfo() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Red Onion Care");
  oled.setCursor(0, 1);
  oled.print("System Ready!");
  delay(2000);
}

void displayWateringComplete() {
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Watering Done!");
  oled.setCursor(0, 1);
  oled.print("Happy Onions ^_^");
  delay(3000);
}

void updateStatusLEDs() {
  static unsigned long lastLEDUpdate = 0;
  static bool activityBlink = false;
  if (millis() - lastLEDUpdate < 1000) return;
  digitalWrite(STATUS_LED_RED, LOW);
  digitalWrite(STATUS_LED_GREEN, LOW);
  digitalWrite(STATUS_LED_BLUE, LOW);
  if (waterTankEmpty || isPlantStressed()) digitalWrite(STATUS_LED_RED, HIGH);
  if (!waterTankEmpty && !isPlantStressed() && !isEnvironmentSuboptimal()) digitalWrite(STATUS_LED_GREEN, HIGH);
  if (activityBlink) digitalWrite(STATUS_LED_BLUE, HIGH);
  activityBlink = !activityBlink;
  lastLEDUpdate = millis();
}

void playStartupMelody() {
  if (!config.soundEnabled) return;
  int melody[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5};
  int durations[] = {BEAT_FAST, BEAT_FAST, BEAT_FAST, BEAT_NORMAL};
  for (int i = 0; i < 4; i++) {
    tone(BUZZER_PIN, melody[i], durations[i]);
    delay(durations[i]);
    delay(50);
  }
}

void playWateringMelody() {
  if (!config.soundEnabled) return;
  int melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4};
  for (int i = 0; i < 6; i++) {
    tone(BUZZER_PIN, melody[i], BEAT_FAST);
    delay(BEAT_FAST + 50);
  }
}

void playCompletionMelody() {
  if (!config.soundEnabled) return;
  int melody[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C5, NOTE_D5, NOTE_F5, NOTE_A4, NOTE_D5, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C5};
  int durations[] = {BEAT_FAST, BEAT_FAST, BEAT_NORMAL, BEAT_FAST, BEAT_FAST, BEAT_FAST, BEAT_NORMAL, BEAT_FAST, BEAT_FAST, BEAT_FAST, BEAT_FAST, BEAT_SLOW};
  for (int i = 0; i < 12; i++) {
    tone(BUZZER_PIN, melody[i], durations[i]);
    delay(durations[i]);
    delay(25);
  }
}

void logSensorData() {
  Serial.println("Temperature: " + String(temperature, 2) + "°C");
  Serial.println("Humidity: " + String(humidity, 2) + "%");
  Serial.println("Moisture: " + String(moistureLevel));
  Serial.println("Light: " + String(lightLevel));
  Serial.println("Water Distance: " + String(waterDistance, 2) + " cm");
  Serial.println("Tank Status: " + String(waterTankEmpty ? "Empty" : "OK"));
  Serial.println("Plant Stressed: " + String(isPlantStressed() ? "YES" : "NO"));
  Serial.println("Needs Watering: " + String(needsWatering() ? "YES" : "NO"));
  Serial.println("Daily Waterings: " + String(dailyWateringCount));
}

void checkCriticalConditions() {
  static unsigned long lastCriticalCheck = 0;
  if (millis() - lastCriticalCheck < 30000) return;
  if (temperature == 0.0 && humidity == 0.0) Serial.println("Warning: BME280 sensor may have failed");
  if (moistureLevel == 0) Serial.println("Warning: Moisture sensor may have failed");
  if (waterDistance > 100) Serial.println("Warning: Ultrasonic sensor may have failed");
  if (temperature > 40.0) Serial.println("CRITICAL: Extreme high temperature detected!");
  if (temperature < 5.0) Serial.println("CRITICAL: Extreme low temperature detected!");
  if (moistureLevel < 100) Serial.println("CRITICAL: Extremely dry soil detected!");
  lastCriticalCheck = millis();
}

void resetDailyCounters() {
  dailyWateringCount = 0;
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "STATUS") {
      logSensorData();
    } else if (command == "WATER") {
      if (!waterTankEmpty) {
        currentState = WATERING;
      }
    } else if (command == "CONFIG") {
      Serial.println("Moisture Threshold: " + String(config.moistureThreshold));
      Serial.println("Watering Duration: " + String(config.wateringDuration));
      Serial.println("Alert Threshold: " + String(config.alertThreshold));
    } else if (command.startsWith("SET_MOISTURE ")) {
      int value = command.substring(13).toInt();
      if (value >= 200 && value <= 800) {
        config.moistureThreshold = value;
        saveConfiguration();
      }
    } else if (command == "RESET") {
      delay(1000);
      asm volatile ("  jmp 0");
    }
  }
}
