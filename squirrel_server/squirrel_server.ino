/*
 * Squirrel Buddy — Sensor Server
 * ESP32 (ESP-WROOM-32, 30-pin, CP2102 USB-C)
 *
 * Reads BME280, BH1750, and INMP441 sensors every 10 seconds.
 * Serves live data as JSON at /api/data and an HTML dashboard at /.
 *
 * ---- LIBRARY INSTALL (Arduino IDE) ----
 * Sketch > Include Library > Manage Libraries…
 *   1. "Adafruit BME280"       → install (pulls in Adafruit Unified Sensor)
 *   2. "BH1750"                → by Christopher Laws
 *
 * No extra networking libraries needed — uses the built-in WebServer.
 *
 * ---- BOARD SETUP ----
 * Tools > Board > ESP32 Arduino > "ESP32 Dev Module"
 * Tools > Upload Speed > 921600
 * Serial Monitor baud: 115200
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <driver/i2s.h>
#include <WebServer.h>

// ============================================================
// WiFi credentials — REPLACE THESE with your network details
// ============================================================
const char* ssid     = "Balay";
const char* password = "Colurado0330";

// ---- Pin Definitions ----
#define I2C_SDA  21
#define I2C_SCL  22
#define I2S_SCK  32
#define I2S_WS   25
#define I2S_SD   33

// ---- I2S / Audio Config ----
#define I2S_PORT        I2S_NUM_0
#define SAMPLE_RATE     16000
#define BLOCK_SIZE      1024
#define DISCARD_MS      300

// ---- Sensor Objects ----
Adafruit_BME280 bme;
BH1750 lightMeter;
WebServer server(80);

// ---- Cached Sensor Readings ----
// These get refreshed every 10 seconds; web requests read them instantly.
struct SensorData {
  float temperature_c  = 0;
  float humidity_pct   = 0;
  float pressure_hpa   = 0;
  float light_lux      = 0;
  int32_t sound_level  = 0;
  unsigned long timestamp_ms = 0;
} latest;

bool bmeOk     = false;
bool bh1750Ok  = false;
bool i2sOk     = false;

unsigned long lastReadMs = 0;
const unsigned long READ_INTERVAL_MS = 10000;  // 10 seconds

// ============================================================
// WiFi connect — blocks until connected, prints IP
// ============================================================
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

// ============================================================
// I2C Bus Scan — prints detected addresses for quick debug
// ============================================================
void scanI2C() {
  Serial.println("------ I2C Bus Scan ------");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  No I2C devices found!");
  }
  Serial.println("--------------------------\n");
}

// ============================================================
// Sensor Init
// ============================================================
void setupBME280() {
  Serial.print("BME280: ");
  if (bme.begin(0x76, &Wire)) {
    Serial.println("OK at 0x76");
    bmeOk = true;
  } else {
    Serial.println("NOT FOUND — check wiring");
  }
}

void setupBH1750() {
  Serial.print("BH1750: ");
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    Serial.println("OK at 0x23");
    bh1750Ok = true;
  } else {
    Serial.println("NOT FOUND — check wiring");
  }
}

void setupI2S() {
  Serial.print("INMP441: ");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
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
    Serial.println("driver install FAILED");
    return;
  }
  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("pin config FAILED");
    return;
  }

  // Discard startup noise — INMP441 needs ~150-300ms to stabilize
  int32_t trash[BLOCK_SIZE];
  size_t bytesRead;
  unsigned long t0 = millis();
  while (millis() - t0 < DISCARD_MS) {
    i2s_read(I2S_PORT, trash, sizeof(trash), &bytesRead, portMAX_DELAY);
  }

  Serial.println("OK");
  i2sOk = true;
}

// ============================================================
// Read INMP441 — computes overall amplitude
// ============================================================
void readMic(int32_t &amplitude) {
  int32_t raw[BLOCK_SIZE];
  size_t bytesRead = 0;

  esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead,
                           1000 / portTICK_PERIOD_MS);
  if (err != ESP_OK || bytesRead == 0) {
    amplitude = -1;
    return;
  }

  int n = bytesRead / sizeof(int32_t);

  // --- Overall amplitude (mean absolute value of 24-bit samples) ---
  int64_t sum = 0;
  for (int i = 0; i < n; i++) {
    int32_t sample = raw[i] >> 8;   // 24-bit signed
    sum += abs(sample);
  }
  amplitude = (int32_t)(sum / n);
}

// ============================================================
// Read all sensors and update the cached `latest` struct
// ============================================================
void refreshSensors() {
  if (bmeOk) {
    latest.temperature_c = bme.readTemperature();
    latest.humidity_pct  = bme.readHumidity();
    latest.pressure_hpa  = bme.readPressure() / 100.0f;
  }

  if (bh1750Ok) {
    latest.light_lux = lightMeter.readLightLevel();
  }

  if (i2sOk) {
    readMic(latest.sound_level);
  }

  latest.timestamp_ms = millis();
}

// ============================================================
// Print readings to Serial Monitor
// ============================================================
void printReadings() {
  Serial.println("---- Sensor Readings ----");
  Serial.printf("  Temp:       %.1f °C\n",  latest.temperature_c);
  Serial.printf("  Humidity:   %.1f %%\n",   latest.humidity_pct);
  Serial.printf("  Pressure:   %.1f hPa\n", latest.pressure_hpa);
  Serial.printf("  Light:      %.1f lux\n", latest.light_lux);
  Serial.printf("  Sound:      %d\n",       latest.sound_level);
  Serial.printf("  Uptime:     %lu ms\n",   latest.timestamp_ms);
  Serial.println("-------------------------\n");
}

// ============================================================
// Build JSON response string
// ============================================================
String buildJSON() {
  // Using manual string building to avoid pulling in ArduinoJson
  String json = "{";
  json += "\"temperature_c\":"  + String(latest.temperature_c, 1);
  json += ",\"humidity_pct\":"  + String(latest.humidity_pct, 1);
  json += ",\"pressure_hpa\":" + String(latest.pressure_hpa, 1);
  json += ",\"light_lux\":"     + String(latest.light_lux, 1);
  json += ",\"sound_level\":"   + String(latest.sound_level);
  json += ",\"timestamp_ms\":"  + String(latest.timestamp_ms);
  json += "}";
  return json;
}

// ============================================================
// Build HTML status page
// ============================================================
String buildHTML() {
  // Static HTML page — JavaScript fetches /api/data every 2 seconds
  // and updates the values in place (no page reload flicker).
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Squirrel Buddy</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: 'IBM Plex Mono', 'Courier New', monospace;
    background: #111; color: #e8e8e8;
    display: flex; justify-content: center;
    padding: 2rem 1rem;
  }
  .card {
    background: #1a1a1a; border: 1px solid #333;
    border-radius: 8px; max-width: 420px; width: 100%;
    padding: 1.5rem;
  }
  h1 { font-size: 1.1rem; margin-bottom: 0.3rem; }
  .sub { color: #888; font-size: 0.75rem; margin-bottom: 1.2rem; }
  .row {
    display: flex; justify-content: space-between;
    padding: 0.5rem 0; border-bottom: 1px solid #222;
  }
  .label { color: #999; }
  .val { font-weight: bold; transition: opacity 0.3s; }
  .val.updating { opacity: 0.5; }
  .status {
    text-align: center; margin-top: 1rem;
    font-size: 0.7rem; color: #555;
  }
  .status.ok { color: #4a4; }
  .status.err { color: #a44; }
  .json-link {
    display: block; text-align: center; margin-top: 0.8rem;
    color: #6cf; font-size: 0.8rem;
  }
</style>
</head>
<body>
<div class="card">
  <h1>&#127330; Squirrel Buddy</h1>
  <div class="sub">Live Oak / East Balcony &mdash; Pip's environment</div>

  <div class="row"><span class="label">Temperature</span><span class="val" id="temp">--</span></div>
  <div class="row"><span class="label">Humidity</span><span class="val" id="hum">--</span></div>
  <div class="row"><span class="label">Pressure</span><span class="val" id="pres">--</span></div>
  <div class="row"><span class="label">Light</span><span class="val" id="lux">--</span></div>
  <div class="row"><span class="label">Sound level</span><span class="val" id="snd">--</span></div>
  <div class="row"><span class="label">Uptime</span><span class="val" id="up">--</span></div>

  <div class="status" id="status">connecting...</div>
  <a class="json-link" href="/api/data">View raw JSON &rarr;</a>
</div>

<script>
// Poll /api/data every 2 seconds and update the page
async function refresh() {
  try {
    const r = await fetch('/api/data');
    const d = await r.json();
    document.getElementById('temp').textContent = d.temperature_c.toFixed(1) + ' \u00B0C';
    document.getElementById('hum').textContent  = d.humidity_pct.toFixed(1) + ' %';
    document.getElementById('pres').textContent = d.pressure_hpa.toFixed(1) + ' hPa';
    document.getElementById('lux').textContent  = d.light_lux.toFixed(1) + ' lux';
    document.getElementById('snd').textContent  = d.sound_level;
    document.getElementById('up').textContent   = Math.floor(d.timestamp_ms / 1000) + ' s';
    const s = document.getElementById('status');
    s.textContent = 'live \u2014 updated ' + new Date().toLocaleTimeString();
    s.className = 'status ok';
  } catch (e) {
    const s = document.getElementById('status');
    s.textContent = 'connection lost \u2014 retrying...';
    s.className = 'status err';
  }
}
refresh();                       // first fetch immediately
setInterval(refresh, 2000);      // then every 2 seconds
</script>
</body>
</html>
)rawhtml";
  return html;
}

// ============================================================
// Web server route setup
// ============================================================
void setupServer() {
  // JSON API endpoint
  server.on("/api/data", []() {
    server.send(200, "application/json", buildJSON());
  });

  // HTML dashboard
  server.on("/", []() {
    server.send(200, "text/html", buildHTML());
  });

  server.begin();
  Serial.println("Web server started.");
  Serial.print("  Dashboard:  http://");
  Serial.println(WiFi.localIP());
  Serial.print("  JSON API:   http://");
  Serial.print(WiFi.localIP());
  Serial.println("/api/data");
  Serial.println();
}

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Squirrel Buddy — Sensor Server");
  Serial.println("========================================\n");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2C();

  // Sensors
  setupBME280();
  setupBH1750();
  setupI2S();
  Serial.println();

  // WiFi
  connectWiFi();

  // Take first reading immediately
  refreshSensors();
  printReadings();

  // Start web server
  setupServer();
}

// ============================================================
// loop() — refresh sensors every 10 seconds
// ============================================================
void loop() {
  // Handle incoming HTTP requests (must be called frequently)
  server.handleClient();

  unsigned long now = millis();
  if (now - lastReadMs >= READ_INTERVAL_MS) {
    lastReadMs = now;
    refreshSensors();
    printReadings();
  }
}
