/*
 * Squirrel Buddy — Sensor Wiring Test
 * ESP32 (ESP-WROOM-32, 30-pin, CP2102 USB-C)
 *
 * Tests BME280, BH1750, and INMP441 and prints readings to Serial Monitor.
 *
 * ---- LIBRARY INSTALL (Arduino IDE) ----
 * Sketch > Include Library > Manage Libraries…
 *   1. Search "Adafruit BME280"    → install (also installs Adafruit Unified Sensor)
 *   2. Search "BH1750"             → install the one by Christopher Laws
 *   3. No extra library needed for INMP441 — uses the built-in ESP32 I2S driver.
 *
 * ---- BOARD SETUP ----
 * Tools > Board > ESP32 Arduino > "ESP32 Dev Module"
 * Tools > Upload Speed > 921600
 * Tools > Port > (pick your CP2102 port)
 * Serial Monitor baud: 115200
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <driver/i2s.h>

// ---- Pin Definitions ----

// I2C (shared by BME280 and BH1750)
#define I2C_SDA 21
#define I2C_SCL 22

// I2S (INMP441 microphone)
#define I2S_SCK  32  // Serial Clock (BCLK)
#define I2S_WS   25  // Word Select (LRCLK)
#define I2S_SD   33  // Serial Data (DOUT from mic)

// ---- Sensor Objects ----
Adafruit_BME280 bme;
BH1750 lightMeter;

// ---- I2S Config ----
#define I2S_PORT       I2S_NUM_0
#define SAMPLE_RATE    16000
#define SAMPLE_BITS    32        // INMP441 sends 24-bit data in 32-bit frames
#define BLOCK_SIZE     512       // samples per read
#define DISCARD_MS     300       // ms of startup noise to throw away

bool bmeOk = false;
bool bh1750Ok = false;
bool i2sOk = false;

// =====================================================
// I2C Bus Scanner — lists every device on the bus
// =====================================================
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
    Serial.println("  No I2C devices found! Check SDA/SCL wiring and pull-ups.");
  } else {
    Serial.print("  Total devices: ");
    Serial.println(found);
  }
  Serial.println("--------------------------");
  Serial.println();
}

// =====================================================
// BME280 Setup
// =====================================================
void setupBME280() {
  Serial.print("BME280: ");
  // Try address 0x76 first (SDO tied to GND)
  if (bme.begin(0x76, &Wire)) {
    Serial.println("OK at 0x76");
    bmeOk = true;
  } else if (bme.begin(0x77, &Wire)) {
    Serial.println("OK at 0x77 (SDO is HIGH — expected 0x76 with SDO→GND)");
    bmeOk = true;
  } else {
    Serial.println("NOT FOUND!");
    Serial.println("  -> Check wiring: SDA→21, SCL→22, VCC→3.3V, GND→GND");
    Serial.println("  -> Check SDO→GND (for 0x76) or SDO→3.3V (for 0x77)");
    Serial.println("  -> Check CSB→3.3V (enables I2C mode)");
  }
}

// =====================================================
// BH1750 Setup
// =====================================================
void setupBH1750() {
  Serial.print("BH1750: ");
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    Serial.println("OK at 0x23");
    bh1750Ok = true;
  } else {
    Serial.println("NOT FOUND!");
    Serial.println("  -> Check wiring: SDA→21, SCL→22, VCC→3.3V, GND→GND");
    Serial.println("  -> ADDR pin should be GND for address 0x23");
  }
}

// =====================================================
// INMP441 (I2S) Setup
// =====================================================
void setupI2S() {
  Serial.print("INMP441: ");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // L/R pin → GND = left
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,  // not transmitting
    .data_in_num = I2S_SD
  };

  esp_err_t err;
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("driver install FAILED (err ");
    Serial.print(err);
    Serial.println(")");
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.print("set pin FAILED (err ");
    Serial.print(err);
    Serial.println(")");
    return;
  }

  // Discard initial noisy samples (INMP441 needs ~150-300ms to stabilize)
  int32_t discard[BLOCK_SIZE];
  size_t bytesRead;
  unsigned long start = millis();
  while (millis() - start < DISCARD_MS) {
    i2s_read(I2S_PORT, discard, sizeof(discard), &bytesRead, portMAX_DELAY);
  }

  Serial.println("OK (I2S driver started, startup noise discarded)");
  i2sOk = true;
}

// =====================================================
// Read INMP441 and return average absolute amplitude
// =====================================================
int32_t readMicAmplitude() {
  int32_t samples[BLOCK_SIZE];
  size_t bytesRead = 0;

  esp_err_t err = i2s_read(I2S_PORT, samples, sizeof(samples), &bytesRead, 1000 / portTICK_PERIOD_MS);
  if (err != ESP_OK || bytesRead == 0) {
    return -1;  // read failed
  }

  int numSamples = bytesRead / sizeof(int32_t);

  // INMP441 puts 24-bit data in the upper bits of a 32-bit word.
  // Shift right by 8 to get a usable signed 24-bit value.
  int64_t sum = 0;
  for (int i = 0; i < numSamples; i++) {
    int32_t val = samples[i] >> 8;  // 24-bit signed value
    sum += abs(val);
  }

  return (int32_t)(sum / numSamples);
}

// =====================================================
// setup() — runs once at boot
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);  // give serial monitor time to connect

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Squirrel Buddy — Sensor Wiring Test");
  Serial.println("========================================");
  Serial.println();

  // Start I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);

  // Step 1: Scan I2C bus
  scanI2C();

  // Step 2: Init each sensor
  setupBME280();
  setupBH1750();
  setupI2S();

  Serial.println();
  Serial.println("Setup complete. Reading sensors every 2 seconds...");
  Serial.println();
}

// =====================================================
// loop() — runs every 2 seconds
// =====================================================
void loop() {
  Serial.println("---- Sensor Readings ----");

  // --- BME280 ---
  if (bmeOk) {
    float temp     = bme.readTemperature();
    float humidity  = bme.readHumidity();
    float pressure  = bme.readPressure() / 100.0F;  // Pa → hPa

    Serial.print("  WARMTH  (BME280):  ");
    Serial.print(temp, 1);
    Serial.print(" °C  |  Humidity: ");
    Serial.print(humidity, 1);
    Serial.print(" %  |  Pressure: ");
    Serial.print(pressure, 1);
    Serial.println(" hPa");
  } else {
    Serial.println("  WARMTH  (BME280):  [ERROR — sensor not found]");
  }

  // --- BH1750 ---
  if (bh1750Ok) {
    float lux = lightMeter.readLightLevel();
    Serial.print("  COVER   (BH1750):  ");
    Serial.print(lux, 1);
    Serial.println(" lux");
  } else {
    Serial.println("  COVER   (BH1750):  [ERROR — sensor not found]");
  }

  // --- INMP441 ---
  if (i2sOk) {
    int32_t amplitude = readMicAmplitude();
    Serial.print("  NOISE  (INMP441):  ");
    if (amplitude < 0) {
      Serial.println("[ERROR — I2S read failed]");
    } else {
      Serial.print("amplitude = ");
      Serial.print(amplitude);
      if (amplitude < 500) {
        Serial.println("  (very quiet / possibly dead — try tapping near mic)");
      } else if (amplitude < 5000) {
        Serial.println("  (quiet — background noise)");
      } else if (amplitude < 50000) {
        Serial.println("  (moderate sound)");
      } else {
        Serial.println("  (loud!)");
      }
    }
  } else {
    Serial.println("  NOISE  (INMP441):  [ERROR — I2S driver not started]");
  }

  Serial.println("-------------------------");
  Serial.println();

  delay(2000);
}
