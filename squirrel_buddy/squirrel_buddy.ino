/*
 * Squirrel Buddy — Environmental Monitor for TRMNL
 * ESP32 (ESP-WROOM-32, 30-pin, CP2102 USB-C)
 *
 * Reads BME280, BH1750, and INMP441 sensors every 5 minutes,
 * computes qualitative labels, and POSTs data to a TRMNL
 * private plugin webhook. Uses deep sleep between readings.
 *
 * ---- LIBRARY INSTALL (Arduino IDE) ----
 * Sketch > Include Library > Manage Libraries…
 *   1. "Adafruit BME280"       — by Adafruit (also installs Adafruit Unified Sensor)
 *   2. "BH1750"                — by Christopher Laws
 *   3. "ArduinoJson"           — by Benoit Blanchon
 *   4. HTTPClient              — built-in with ESP32 board package (no install needed)
 *
 * ---- BOARD SETUP ----
 * Tools > Board > ESP32 Arduino > "ESP32 Dev Module"
 * Tools > Upload Speed > 921600
 * Tools > Port > (pick your CP2102 port)
 * Serial Monitor baud: 115200
 */

#include "labels.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// =============================================================
// CONFIG — Edit these three lines with your actual credentials
// =============================================================
const char* ssid       = "Balay";
const char* password   = "Colurado0330";
const char* webhookUrl = "https://trmnl.com/api/custom_plugins/c0748537-e287-476e-82d8-017e8bb39b08";

// =============================================================
// NTP — for the updated_at timestamp
// =============================================================
const char* ntpServer     = "pool.ntp.org";
const long  utcOffsetSec  = -5 * 3600;  // Central Time: -5 (CDT) or -6 (CST)
const int   daylightSec   = 0;          // Set to 3600 if you want auto-DST

// =============================================================
// Deep sleep interval
// =============================================================
#define SLEEP_MINUTES  5
#define uS_TO_S_FACTOR 1000000ULL

// =============================================================
// Pin Definitions
// =============================================================

// I2C (shared by BME280 and BH1750)
#define I2C_SDA 21
#define I2C_SCL 22

// I2S (INMP441 microphone)
#define I2S_SCK  32   // Serial Clock (BCLK)
#define I2S_WS   25   // Word Select (LRCLK)
#define I2S_SD   33   // Serial Data (DOUT from mic)

// =============================================================
// I2S config
// =============================================================
#define I2S_PORT       I2S_NUM_0
#define SAMPLE_RATE    16000
#define SAMPLE_BITS    32           // INMP441 sends 24-bit in 32-bit frames
#define BLOCK_SIZE     512
#define DISCARD_MS     300          // ms of startup noise to throw away
#define NUM_READS      10           // how many blocks to average for sound_level

// =============================================================
// Sensor objects
// =============================================================
Adafruit_BME280 bme;
BH1750 lightMeter;

// =============================================================
// WiFi — connect with retries
// =============================================================
bool connectWiFi() {
  Serial.print("WiFi: connecting to ");
  Serial.print(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(" connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println(" FAILED after 15 seconds.");
    return false;
  }
}

// =============================================================
// NTP — sync time and format as "h:mmam/pm"
// =============================================================
String getTimeString() {
  configTime(utcOffsetSec, daylightSec, ntpServer);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5000)) {
    Serial.println("NTP: failed to get time");
    return "??:??";
  }

  int hour12 = timeinfo.tm_hour % 12;
  if (hour12 == 0) hour12 = 12;
  const char* ampm = (timeinfo.tm_hour < 12) ? "am" : "pm";

  char buf[16];
  snprintf(buf, sizeof(buf), "%d:%02d%s", hour12, timeinfo.tm_min, ampm);
  return String(buf);
}

// =============================================================
// I2S — start driver, discard warm-up noise
// =============================================================
bool setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,   // L/R → GND = left channel
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("I2S: driver install failed");
    return false;
  }
  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("I2S: set pin failed");
    return false;
  }

  // Discard the first 300ms of noisy samples
  static int32_t discard[BLOCK_SIZE];
  size_t bytesRead;
  unsigned long start = millis();
  while (millis() - start < DISCARD_MS) {
    i2s_read(I2S_PORT, discard, sizeof(discard), &bytesRead, portMAX_DELAY);
  }

  Serial.println("I2S: OK (warm-up noise discarded)");
  return true;
}

// =============================================================
// Read microphone — compute sound_level
//
// sound_level: average absolute amplitude, scaled to ~0-100
// =============================================================
void readMicrophone(int &soundLevel) {
  static int32_t raw[BLOCK_SIZE];     // static to avoid stack overflow
  size_t bytesRead;

  // Accumulate amplitude over several blocks for a stable reading
  int64_t totalAmplitude = 0;
  int totalSamples = 0;

  for (int r = 0; r < NUM_READS; r++) {
    esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK || bytesRead == 0) continue;

    int count = bytesRead / sizeof(int32_t);
    for (int i = 0; i < count; i++) {
      int32_t sample = raw[i] >> 8;  // 24-bit signed value
      totalAmplitude += abs(sample);
    }
    totalSamples += count;
  }

  // --- sound_level: scale raw amplitude to a 0-100 range ---
  // These divisors are tuned for INMP441 @ 24-bit. Adjust during calibration.
  int32_t avgAmplitude = (totalSamples > 0) ? (int32_t)(totalAmplitude / totalSamples) : 0;
  soundLevel = avgAmplitude / 500;      // rough scaling — tune this!
  if (soundLevel > 100) soundLevel = 100;

  Serial.print("  Mic raw avg amplitude: ");
  Serial.print(avgAmplitude);
  Serial.print(" → sound_level: ");
  Serial.println(soundLevel);
}

// =============================================================
// Label helpers — threshold-based qualitative labels
// =============================================================

Label getTempLabel(float tempF) {
  if (tempF < 40)       return {"Cold",  "Too cold for most activity"};
  else if (tempF < 60)  return {"Cool",  "Cool but manageable"};
  else if (tempF <= 85) return {"Warm",  "Warm enough for foraging"};
  else                   return {"Hot",   "Heat may limit activity"};
}

Label getLightLabel(float lux) {
  if (lux < 10)          return {"Dark",        "Too dark to forage safely"};
  else if (lux < 100)    return {"Dim",         "Low light, limited visibility"};
  else if (lux <= 10000) return {"Bright",      "Good visibility, bright"};
  else                    return {"Very Bright", "Full sun, high exposure"};
}

Label getSoundLabel(int level) {
  if (level < 20)       return {"Quiet",    "Calm, no disturbances nearby"};
  else if (level <= 60) return {"Moderate", "Some noise, may stay alert"};
  else                   return {"Loud",     "Noisy, likely to avoid area"};
}

Label getPressureLabel(float hpa) {
  if (hpa < 1000)        return {"Low",    "Dropping pressure, storm may be approaching"};
  else if (hpa <= 1020)  return {"Stable", "Stable pressure, no storm approaching"};
  else                    return {"High",   "High pressure, clear conditions"};
}

// =============================================================
// Compute overall condition from labels
// =============================================================
String getCondition(Label temp, Label light, Label sound) {
  // Stressed triggers → "sheltering"
  if (sound.label == "Loud" || temp.label == "Cold" || temp.label == "Hot" ||
      light.label == "Dark") {
    return "sheltering";
  }
  // Borderline triggers → "cautious"
  if (sound.label == "Moderate" || temp.label == "Cool" ||
      light.label == "Dim") {
    return "cautious";
  }
  // Everything good → "active"
  return "active";
}

// Build a one-line summary of why
String getConditionDesc(String condition, Label temp, Label light, Label sound) {
  if (condition == "active") {
    return temp.label + ", calm and quiet, good light for foraging";
  } else if (condition == "cautious") {
    // Mention the borderline factors
    String reasons = "";
    if (temp.label == "Cool") reasons += "cool temps";
    if (sound.label == "Moderate") {
      if (reasons.length() > 0) reasons += ", ";
      reasons += "some noise";
    }
    if (light.label == "Dim") {
      if (reasons.length() > 0) reasons += ", ";
      reasons += "low light";
    }
    return "Staying alert — " + reasons;
  } else {
    // sheltering — mention the worst factors
    String reasons = "";
    if (temp.label == "Cold") reasons += "too cold";
    if (temp.label == "Hot") reasons += "too hot";
    if (sound.label == "Loud") {
      if (reasons.length() > 0) reasons += ", ";
      reasons += "loud noise";
    }
    if (light.label == "Dark") {
      if (reasons.length() > 0) reasons += ", ";
      reasons += "too dark";
    }
    return "Sheltering — " + reasons;
  }
}

// =============================================================
// POST data to TRMNL webhook
// =============================================================
void postToTRMNL(JsonDocument &doc) {
  HTTPClient http;
  http.begin(webhookUrl);
  http.addHeader("Content-Type", "application/json");

  String jsonStr;
  serializeJson(doc, jsonStr);

  Serial.println();
  Serial.println("---- POST to TRMNL ----");
  Serial.println(jsonStr);

  int httpCode = http.POST(jsonStr);
  Serial.print("HTTP response: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println(http.getString());
  } else {
    Serial.print("POST failed: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

// =============================================================
// setup() — runs once each wake cycle
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Squirrel Buddy — Sensor Monitor");
  Serial.println("========================================");
  Serial.println();

  // --- Connect WiFi ---
  if (!connectWiFi()) {
    Serial.println("No WiFi — going back to sleep.");
    esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

  // --- Sync time via NTP ---
  String updatedAt = getTimeString();
  Serial.print("Time: ");
  Serial.println(updatedAt);
  Serial.println();

  // --- Init I2C sensors ---
  Wire.begin(I2C_SDA, I2C_SCL);

  bool bmeOk = bme.begin(0x76, &Wire);
  Serial.print("BME280: ");
  Serial.println(bmeOk ? "OK" : "NOT FOUND");

  bool bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.print("BH1750: ");
  Serial.println(bh1750Ok ? "OK" : "NOT FOUND");

  // --- Init I2S microphone ---
  bool i2sOk = setupI2S();

  Serial.println();
  Serial.println("---- Sensor Readings ----");

  // --- Read BME280 ---
  float tempC     = bmeOk ? bme.readTemperature()            : 0;
  float tempF     = bmeOk ? (tempC * 9.0 / 5.0 + 32.0)      : 0;
  float humidity   = bmeOk ? bme.readHumidity()               : 0;
  float pressure   = bmeOk ? bme.readPressure() / 100.0F      : 0;  // Pa → hPa

  Serial.print("  Temp:     ");  Serial.print(tempF, 1);    Serial.println(" °F");
  Serial.print("  Humidity: ");  Serial.print(humidity, 1);  Serial.println(" %");
  Serial.print("  Pressure: "); Serial.print(pressure, 1);  Serial.println(" hPa");

  // --- Read BH1750 ---
  float lux = bh1750Ok ? lightMeter.readLightLevel() : 0;
  Serial.print("  Light:    "); Serial.print(lux, 1);       Serial.println(" lux");

  // --- Read INMP441 ---
  int soundLevel = 0;
  if (i2sOk) {
    readMicrophone(soundLevel);
  } else {
    Serial.println("  Mic: skipped (I2S failed)");
  }

  // Clean up I2S before WiFi POST (frees DMA memory)
  if (i2sOk) {
    i2s_driver_uninstall(I2S_PORT);
  }

  // --- Compute labels ---
  Label temp_l     = getTempLabel(tempF);
  Label light_l    = getLightLabel(lux);
  Label sound_l    = getSoundLabel(soundLevel);
  Label pressure_l = getPressureLabel(pressure);

  String condition     = getCondition(temp_l, light_l, sound_l);
  String conditionDesc = getConditionDesc(condition, temp_l, light_l, sound_l);

  Serial.println();
  Serial.println("---- Labels ----");
  Serial.print("  Temp:      "); Serial.print(temp_l.label);     Serial.print(" — "); Serial.println(temp_l.desc);
  Serial.print("  Light:     "); Serial.print(light_l.label);    Serial.print(" — "); Serial.println(light_l.desc);
  Serial.print("  Sound:     "); Serial.print(sound_l.label);    Serial.print(" — "); Serial.println(sound_l.desc);
  Serial.print("  Pressure:  "); Serial.print(pressure_l.label); Serial.print(" — "); Serial.println(pressure_l.desc);
  Serial.print("  Condition: "); Serial.print(condition);        Serial.print(" — "); Serial.println(conditionDesc);

  // --- Build JSON payload ---
  JsonDocument doc;
  JsonObject mv = doc["merge_variables"].to<JsonObject>();

  mv["temperature_f"]   = (int)round(tempF);
  mv["humidity_pct"]    = (int)round(humidity);
  mv["pressure_hpa"]    = (int)round(pressure);
  mv["light_lux"]       = (int)round(lux);
  mv["sound_level"]     = soundLevel;
  mv["condition"]       = condition;
  mv["temp_label"]      = temp_l.label;
  mv["light_label"]     = light_l.label;
  mv["sound_label"]     = sound_l.label;
  mv["pressure_label"]  = pressure_l.label;
  mv["temp_desc"]       = temp_l.desc;
  mv["light_desc"]      = light_l.desc;
  mv["sound_desc"]      = sound_l.desc;
  mv["pressure_desc"]   = pressure_l.desc;
  mv["condition_desc"]  = conditionDesc;
  mv["updated_at"]      = updatedAt;

  // --- POST to TRMNL ---
  postToTRMNL(doc);

  // --- Deep sleep ---
  Serial.println();
  Serial.print("Sleeping for ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" minutes...");
  Serial.println();

  WiFi.disconnect(true);
  esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

// =============================================================
// loop() — never reached (deep sleep resets to setup)
// =============================================================
void loop() {
  // Deep sleep restarts from setup(), so this never runs.
}
