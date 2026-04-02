/**
 * ============================================================
 *  Weather Station — Sensor & Data Collection
 *  Hardware: LilyGo T-SIM7080G (ESP32-S3 based)
 *
 *  Sensors:
 *    - AS5048A   : Wind direction     (SPI)
 *    - BMP390    : Barometric pressure (SPI)
 *    - SHT31     : Temp & Humidity    (I2C, with bus-reset watchdog)
 *    - Hall switch: Wind speed        (Hardware interrupt, always on)
 *
 *  Output format (METAR-style):
 *    CYUL 1256Z 14021G26KT 18/17 A2970 RMK
 *
 *  Libraries required (install via Arduino Library Manager):
 *    - Adafruit BMP3XX Library   (BMP390)
 *    - Adafruit SHT31 Library    (SHT31)
 *    - Adafruit Unified Sensor   (dependency)
 * ============================================================
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>

// ============================================================
//  USER CONFIGURATION — Edit these to match your installation
// ============================================================

#define STATION_ID          "CYUL"   // ICAO or custom 4-letter station identifier

// Station altitude above mean sea level in metres.
// Used to reduce station pressure to sea-level altimeter setting (QNH).
#define STATION_ALTITUDE_M  36.0f

// Magnetic declination correction in degrees.
// Positive = clockwise offset (e.g. +15 if station faces 15° east of mag. north).
// Find your local declination at: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define MAG_NORTH_OFFSET    0


// ============================================================
//  PIN DEFINITIONS — Adjust to match your wiring !! VERY IMPORTSNT CHANGE HERE AND CONFIGURE !!
// ============================================================

// --- Shared SPI Bus ---
#define SPI_SCK     18
#define SPI_MISO    16
#define SPI_MOSI    17

// --- SPI Chip Select (each sensor has its own CS line) ---
#define CS_AS5048A  10    // Wind direction sensor
#define CS_BMP390    12    // Barometric pressure sensor

// --- I2C Bus ---
#define I2C_SDA     13 //maybe change to 3 and I2C_SCL to 2
#define I2C_SCL     14

// --- Wind Speed Hall Effect Sensor ---
// Must be an interrupt-capable GPIO. On ESP32-S3 this is nearly any GPIO.
// Wired as active-LOW (pull-up enabled): each magnet pass = one FALLING edge.
#define WIND_SPEED_PIN  34


// ============================================================
//  CONSTANTS — Do not change unless you know what you're doing
// ============================================================

// Wind cup geometry
#define WIND_CUP_RADIUS_M   0.12f                            // metres
#define WIND_CUP_CIRC_M     (2.0f * M_PI * WIND_CUP_RADIUS_M)  // ~0.7540 m / rotation

// Unit conversion
#define MS_TO_KNOTS         1.94384f

// Data collection
#define DATA_INTERVAL_MS    60000UL   // Collect and report every 60 seconds

// Wind history
#define WIND_HISTORY_SIZE   10        // Keep 10 one-minute buckets (= 10 min of history)
#define WIND_AVG_MINUTES     2        // Average the last N minutes for reported wind speed

// I2C reliability settings
#define I2C_SPEED_HZ        100000    // 100 kHz — more reliable than 400 kHz for long wires
#define I2C_TIMEOUT_MS         500    // Max time to wait for a SHT31 read
#define SHT31_RETRY_COUNT        3    // Attempts before giving up and resetting the bus

// SHT31 I2C address (ADDR pin LOW = 0x44, HIGH = 0x45)
#define SHT31_ADDR          0x44


// ============================================================
//  WIND SPEED — ISR state (must be volatile + mutex protected)
// ============================================================

volatile uint32_t  windPulseCount = 0;
portMUX_TYPE       windMux = portMUX_INITIALIZER_UNLOCKED;


// ============================================================
//  WIND HISTORY — circular buffer, one entry per minute
// ============================================================

float   windSpeedHistory[WIND_HISTORY_SIZE];  // Wind speed per minute in knots
uint8_t windHistoryIndex = 0;                 // Next write position
uint8_t windHistoryCount = 0;                 // How many valid entries exist so far


// ============================================================
//  SENSOR OBJECTS
// ============================================================

Adafruit_SHT31   sht31;
Adafruit_BMP3XX  bmp;


// ============================================================
//  SOFTWARE CLOCK
//  Tracks UTC time between GPS syncs. Updated every loop tick.
// ============================================================

struct SoftClock {
  uint16_t year    = 2024;
  uint8_t  month   = 1;
  uint8_t  day     = 1;
  uint8_t  hour    = 0;
  uint8_t  minute  = 0;
  uint8_t  second  = 0;
  uint32_t lastMs  = 0;   // millis() value at the last clock update
} clk;


// ============================================================
//  WEATHER DATA — latest confirmed readings
// ============================================================

struct WeatherData {
  float tempC      = 0.0f;   // °C
  float humidity   = 0.0f;   // %RH
  float dewPointC  = 0.0f;   // °C  (derived from temp + humidity)
  float pressHPa   = 0.0f;   // hPa (station pressure, raw from BMP390)
  float altimInHg  = 0.0f;   // inHg (QNH altimeter setting, sea-level reduced)
  float windDirDeg = 0.0f;   // degrees (0–359, magnetic, corrected for offset)
  float windSpdKt  = 0.0f;   // knots (2-minute average)
  float windGstKt  = 0.0f;   // knots (10-minute peak gust)
} wx;


// ============================================================
//  FORWARD DECLARATIONS
// ============================================================

// AS5048A
uint16_t as5048aReadRaw();
float    as5048aGetDegrees();

// BMP390
bool     bmp390Init();
float    bmp390GetPressureHPa();
float    pressureToAltimeterInHg(float pressHPa, float altM, float tempC);

// SHT31 / I2C
bool     sht31Init();
bool     sht31Read(float &tempC, float &humidity);
void     i2cBusReset();
float    calcDewPoint(float tempC, float humidity);

// Wind speed
void IRAM_ATTR windPulseISR();
float    getWindSpeedKnots(uint32_t pulses, uint32_t intervalMs);
float    getWindAvgKnots();
float    getWindGustKnots();

// Clock
void     updateSoftClock();
String   getTimeZuluString();
void     syncGPSTime();

// METAR
String   buildMETAR();


// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n========================================"));
  Serial.println(F("  Weather Station — Booting"));
  Serial.println(F("========================================\n"));

  // ---- SPI Bus ----------------------------------------
  // Deassert both CS lines before starting SPI
  pinMode(CS_AS5048A, OUTPUT);  digitalWrite(CS_AS5048A, HIGH);
  pinMode(CS_BMP390,  OUTPUT);  digitalWrite(CS_BMP390,  HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println(F("[SPI]   Bus started"));

  // ---- I2C Bus ----------------------------------------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_SPEED_HZ);
  Serial.printf("[I2C]   Bus started at %d Hz\n", I2C_SPEED_HZ);

  // ---- BMP390 (SPI) -----------------------------------
  if (!bmp390Init()) {
    Serial.println(F("[BMP390] WARNING: Init failed — pressure unavailable"));
  }

  // ---- SHT31 (I2C) ------------------------------------
  if (!sht31Init()) {
    Serial.println(F("[SHT31] WARNING: Init failed — temp/humidity unavailable"));
  }

  // ---- Wind Speed Hall Effect -------------------------
  pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windPulseISR, FALLING);
  Serial.println(F("[WIND]  Hall effect interrupt attached (FALLING edge)"));

  // ---- Wind history buffer ----------------------------
  memset(windSpeedHistory, 0, sizeof(windSpeedHistory));

  // ---- Soft Clock -------------------------------------
  clk.lastMs = millis();
  // TODO: Call syncGPSTime() here once SIM7080G modem is initialised
  Serial.println(F("[CLOCK] Soft clock started (GPS sync pending)"));

  Serial.println(F("\n[BOOT]  Initialisation complete — waiting for first data interval\n"));
}


// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
  static uint32_t lastDataMs    = 0;
  static uint32_t lastGPSSyncMs = 0;

  uint32_t now = millis();

  // Keep the software clock ticking every loop iteration
  updateSoftClock();

  // ----------------------------------------------------------
  //  Every DATA_INTERVAL_MS (1 minute): collect & report
  // ----------------------------------------------------------
  if (now - lastDataMs >= DATA_INTERVAL_MS) {
    lastDataMs = now;

    Serial.println(F("\n--- Collecting Weather Data ---"));

    // == 1. Wind Speed =====================================
    // Atomically snapshot and reset the pulse counter
    uint32_t pulses;
    portENTER_CRITICAL(&windMux);
    pulses = windPulseCount;
    windPulseCount = 0;
    portEXIT_CRITICAL(&windMux);

    float minuteWindKt = getWindSpeedKnots(pulses, DATA_INTERVAL_MS);

    // Store this minute's speed in the circular history buffer
    windSpeedHistory[windHistoryIndex] = minuteWindKt;
    windHistoryIndex = (windHistoryIndex + 1) % WIND_HISTORY_SIZE;
    if (windHistoryCount < WIND_HISTORY_SIZE) windHistoryCount++;

    wx.windSpdKt = getWindAvgKnots();   // 2-minute rolling average
    wx.windGstKt = getWindGustKnots();  // 10-minute peak

    Serial.printf("[WIND]  Pulses=%lu | This min=%.1f kt | 2min avg=%.1f kt | 10min gust=%.1f kt\n",
                  pulses, minuteWindKt, wx.windSpdKt, wx.windGstKt);

    // == 2. Wind Direction (AS5048A) =======================
    wx.windDirDeg = as5048aGetDegrees();
    // Apply magnetic north correction offset
    wx.windDirDeg = fmod(wx.windDirDeg + MAG_NORTH_OFFSET + 360.0f, 360.0f);
    Serial.printf("[DIR]   Wind direction = %.1f° (magnetic, corrected)\n", wx.windDirDeg);

    // == 3. Temperature & Humidity (SHT31) =================
    float rawTemp = 0, rawHum = 0;
    if (sht31Read(rawTemp, rawHum)) {
      wx.tempC     = rawTemp;
      wx.humidity  = rawHum;
      wx.dewPointC = calcDewPoint(wx.tempC, wx.humidity);
      Serial.printf("[SHT31] Temp=%.1f°C | RH=%.1f%% | Dew=%.1f°C\n",
                    wx.tempC, wx.humidity, wx.dewPointC);
    } else {
      Serial.println(F("[SHT31] Read failed — retaining last known values"));
    }

    // == 4. Pressure (BMP390) ==============================
    float rawPress = bmp390GetPressureHPa();
    if (rawPress > 0.0f) {
      wx.pressHPa  = rawPress;
      wx.altimInHg = pressureToAltimeterInHg(wx.pressHPa, STATION_ALTITUDE_M, wx.tempC);
      Serial.printf("[BMP390] Pressure=%.2f hPa | Altimeter=%.2f inHg\n",
                    wx.pressHPa, wx.altimInHg);
    } else {
      Serial.println(F("[BMP390] Read failed — retaining last known values"));
    }

    // == 5. Build & output METAR ===========================
    String metar = buildMETAR();
    Serial.println(F("\n=== METAR OUTPUT ==="));
    Serial.println(metar);
    Serial.println(F("====================\n"));

    // TODO: Pass `metar` string to cellular_connectivity module for SMS dispatch
  }

  // ----------------------------------------------------------
  //  Once every 24 hours: re-sync the clock from GPS
  // ----------------------------------------------------------
  if (now - lastGPSSyncMs >= 86400000UL) {
    lastGPSSyncMs = now;
    syncGPSTime();
  }
}


// ============================================================
//  AS5048A — WIND DIRECTION (SPI)
// ============================================================

/**
 * Perform a single raw angle read from the AS5048A.
 *
 * The AS5048A uses a pipelined SPI protocol:
 *   Frame 1 — send the READ_ANGLE command (0xFFFF); response is undefined.
 *   Frame 2 — send a NOP; response is the actual 14-bit angle.
 *
 * The upper two bits of the response are the error flag and parity bit;
 * we discard them with a 0x3FFF mask.
 *
 * SPI mode: Mode 1 (CPOL=0, CPHA=1), MSB first, up to 10 MHz.
 */
uint16_t as5048aReadRaw() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // Frame 1: issue the READ ANGLE command
  digitalWrite(CS_AS5048A, LOW);
  SPI.transfer16(0xFFFF);           // 0x3FFF command + parity bit set
  digitalWrite(CS_AS5048A, HIGH);
  delayMicroseconds(2);             // t_CSn: min 350 ns between frames

  // Frame 2: clock out the result while sending a NOP
  digitalWrite(CS_AS5048A, LOW);
  uint16_t result = SPI.transfer16(0xC000);   // NOP with R/W bit set
  digitalWrite(CS_AS5048A, HIGH);

  SPI.endTransaction();
  return result & 0x3FFF;           // Discard error + parity bits → 14-bit angle
}

/**
 * Return wind direction in degrees (0.0–359.9°).
 *
 * Takes N samples and averages them using circular (vector) averaging
 * to correctly handle the 0°/360° wraparound boundary.
 * Without this, averaging 355° and 5° would yield 180° instead of 0°.
 */
float as5048aGetDegrees() {
  const int samples = 5;
  float sinSum = 0.0f, cosSum = 0.0f;

  for (int i = 0; i < samples; i++) {
    uint16_t raw = as5048aReadRaw();
    float    deg = (raw / 16384.0f) * 360.0f;
    float    rad = deg * DEG_TO_RAD;
    sinSum += sinf(rad);
    cosSum += cosf(rad);
    delay(5);
  }

  float avgRad = atan2f(sinSum, cosSum);   // Returns -π to +π
  float avgDeg = avgRad * RAD_TO_DEG;
  if (avgDeg < 0.0f) avgDeg += 360.0f;
  return avgDeg;
}


// ============================================================
//  BMP390 — BAROMETRIC PRESSURE (SPI)
// ============================================================

/**
 * Initialise the BMP390 over SPI using the Adafruit BMP3XX library.
 * Settings are tuned for weather-station accuracy:
 *   - 16× pressure oversampling → noise down to ~0.016 hPa RMS
 *   - 2× temperature oversampling
 *   - IIR filter coefficient 3 → smooths out short pressure spikes
 */
bool bmp390Init() {
  if (!bmp.begin_SPI(CS_BMP390, SPI_SCK, SPI_MISO, SPI_MOSI)) {
    Serial.println(F("[BMP390] begin_SPI() failed — check wiring and CS pin"));
    return false;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  Serial.println(F("[BMP390] Initialised OK"));
  return true;
}

/**
 * Read current station pressure in hPa.
 * Returns 0.0 on failure.
 */
float bmp390GetPressureHPa() {
  if (!bmp.performReading()) {
    Serial.println(F("[BMP390] performReading() failed"));
    return 0.0f;
  }
  return bmp.pressure / 100.0f;   // BMP3XX returns Pa; convert to hPa
}

/**
 * Reduce station pressure to sea-level altimeter setting (QNH) in inches of mercury.
 *
 * Aviation altimeter setting answers the question: "If this pressure were measured
 * at sea level, what would it read?" This lets all pilots reference a common baseline.
 *
 * Formula (standard atmosphere hypsometric equation):
 *
 *   QNH_hPa = P_station × ( (T_K + 0.0065 × h) / T_K )^5.2561
 *
 * where:
 *   P_station = raw station pressure in hPa
 *   T_K       = station temperature in Kelvin
 *   h         = station altitude above MSL in metres
 *   5.2561    = standard atmosphere exponent (g / (R × L))
 *
 * Then: QNH_inHg = QNH_hPa × 0.02953
 *
 * @param pressHPa    Station pressure in hPa
 * @param altM        Station altitude above MSL in metres
 * @param tempC       Current temperature in °C
 * @return            Altimeter setting in inHg
 */
float pressureToAltimeterInHg(float pressHPa, float altM, float tempC) {
  float T_K    = tempC + 273.15f;
  float ratio  = (T_K + 0.0065f * altM) / T_K;
  float qnh_hPa = pressHPa * powf(ratio, 5.2561f);
  return qnh_hPa * 0.02953f;
}


// ============================================================
//  SHT31 — TEMPERATURE & HUMIDITY (I2C)
// ============================================================

/**
 * Initialise the SHT31.
 * We disable the internal heater — it is only useful to prevent
 * condensation during testing; leaving it on will bias temperature readings.
 */
bool sht31Init() {
  if (!sht31.begin(SHT31_ADDR)) {
    Serial.printf("[SHT31] Not found at I2C address 0x%02X\n", SHT31_ADDR);
    return false;
  }
  sht31.heater(false);
  Serial.println(F("[SHT31] Initialised OK"));
  return true;
}

/**
 * Read temperature and humidity from the SHT31 with robust error handling.
 *
 * I2C is susceptible to bus lockups: if a transaction is interrupted
 * mid-byte (power glitch, firmware crash), the sensor can hold SDA LOW
 * indefinitely, blocking all devices. We handle this with:
 *
 *   1. Sanity-check the values (NaN, out-of-physical-range).
 *   2. On failure, perform a hardware I2C bus reset (i2cBusReset()).
 *   3. Retry up to SHT31_RETRY_COUNT times.
 *
 * @param tempC    Output: temperature in °C
 * @param humidity Output: relative humidity in %
 * @return true on success
 */
bool sht31Read(float &tempC, float &humidity) {
  for (uint8_t attempt = 0; attempt < SHT31_RETRY_COUNT; attempt++) {
    uint32_t t0 = millis();
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    uint32_t elapsed = millis() - t0;

    // Check for valid, physically plausible values
    bool valid = !isnan(t) && !isnan(h)
              && (t > -40.0f) && (t < 125.0f)
              && (h >= 0.0f)  && (h <= 100.0f)
              && (elapsed < I2C_TIMEOUT_MS);

    if (valid) {
      // Warn on unusual values that might indicate sensor degradation or glitch
      if (t > 70.0f || t < -35.0f) {
        Serial.printf("[SHT31] Warning: temperature %.1f°C is outside expected range\n", t);
      }
      if (h < 1.0f || h > 99.9f) {
        Serial.printf("[SHT31] Warning: humidity %.1f%% is at sensor limits\n", h);
      }
      tempC    = t;
      humidity = h;
      return true;
    }

    // Read failed — log details
    Serial.printf("[SHT31] Attempt %d failed (elapsed=%lu ms, t=%.2f, h=%.2f)\n",
                  attempt + 1, elapsed, t, h);

    // On all but the last attempt, reset the I2C bus and re-init the sensor
    if (attempt < SHT31_RETRY_COUNT - 1) {
      Serial.println(F("[I2C]   Triggering bus reset..."));
      i2cBusReset();
      sht31.begin(SHT31_ADDR);   // Re-discover the sensor after bus reset
      delay(50);
    }
  }

  Serial.println(F("[SHT31] All retry attempts exhausted"));
  return false;
}

/**
 * Hardware I2C bus reset — the gold-standard recovery procedure.
 *
 * If a sensor is mid-transaction and holding SDA LOW, the only way to
 * recover is to manually toggle SCL until the device releases SDA,
 * then send a STOP condition. This matches the NXP I2C specification
 * for bus recovery (UM10204, §3.1.16).
 *
 * Procedure:
 *   1. Release I2C peripheral and take manual control of pins.
 *   2. Drive SCL HIGH+LOW 9 times (worst case: 8 data bits + ACK).
 *   3. Issue a proper STOP (SDA LOW→HIGH while SCL is HIGH).
 *   4. Release pins and restart Wire.
 */
void i2cBusReset() {
  Wire.end();
  delay(10);

  // Take manual GPIO control
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);

  digitalWrite(I2C_SDA, HIGH);
  delayMicroseconds(5);

  // Toggle SCL 9 times to clock out any stuck byte
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }

  // Generate STOP condition: SDA LOW → HIGH while SCL is HIGH
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH);
  delayMicroseconds(5);

  // Release bus pins and reinitialise Wire
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(20);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_SPEED_HZ);
  Serial.println(F("[I2C]   Bus reset complete"));
}

/**
 * Calculate dew point temperature using the Magnus formula.
 *
 * Accurate to within ±0.35°C over the range −40°C to +60°C.
 *
 * γ(T, RH) = (a × T) / (b + T) + ln(RH / 100)
 * Td = (b × γ) / (a − γ)
 *
 * where a = 17.271, b = 237.3°C
 *
 * @param tempC    Temperature in °C
 * @param humidity Relative humidity in % (0–100)
 * @return Dew point in °C
 */
float calcDewPoint(float tempC, float humidity) {
  const float a = 17.271f;
  const float b = 237.3f;
  float gamma = (a * tempC) / (b + tempC) + logf(humidity / 100.0f);
  return (b * gamma) / (a - gamma);
}


// ============================================================
//  WIND SPEED — HALL EFFECT SENSOR (Interrupt)
// ============================================================

/**
 * ISR: fired on every FALLING edge of the hall effect sensor.
 *
 * Keep this function as short as possible — no Serial, no delay.
 * portMUX ensures atomic access to the shared counter if the main
 * core tries to read it at the same time.
 */
void IRAM_ATTR windPulseISR() {
  portENTER_CRITICAL_ISR(&windMux);
  windPulseCount++;
  portEXIT_CRITICAL_ISR(&windMux);
}

/**
 * Convert a pulse count measured over a known interval into knots.
 *
 * Each pulse = 1 full rotation of the wind cups.
 * Linear tip speed = rotations_per_second × circumference.
 *
 * @param pulses      Number of magnet passes in the measurement window
 * @param intervalMs  Length of the measurement window in milliseconds
 * @return Wind speed in knots
 */
float getWindSpeedKnots(uint32_t pulses, uint32_t intervalMs) {
  if (intervalMs == 0 || pulses == 0) return 0.0f;
  float rps = (float)pulses / (intervalMs / 1000.0f);  // rotations per second
  float mps = rps * WIND_CUP_CIRC_M;                   // metres per second
  return mps * MS_TO_KNOTS;
}

/**
 * Return the 2-minute average wind speed in knots.
 * Walks backwards through the circular history buffer.
 */
float getWindAvgKnots() {
  if (windHistoryCount == 0) return 0.0f;

  uint8_t count = min((uint8_t)WIND_AVG_MINUTES, windHistoryCount);
  float   sum   = 0.0f;

  for (int i = 0; i < count; i++) {
    int idx = ((int)windHistoryIndex - 1 - i + WIND_HISTORY_SIZE) % WIND_HISTORY_SIZE;
    sum += windSpeedHistory[idx];
  }
  return sum / (float)count;
}

/**
 * Return the peak wind gust over the last 10 minutes in knots.
 * Scans the entire history buffer for the maximum value.
 */
float getWindGustKnots() {
  if (windHistoryCount == 0) return 0.0f;

  float   maxVal = 0.0f;
  uint8_t count  = min(windHistoryCount, (uint8_t)WIND_HISTORY_SIZE);

  for (int i = 0; i < count; i++) {
    if (windSpeedHistory[i] > maxVal) maxVal = windSpeedHistory[i];
  }
  return maxVal;
}


// ============================================================
//  SOFTWARE CLOCK
// ============================================================

/**
 * Update the software clock from elapsed millis().
 * Called every loop iteration. Handles second/minute/hour rollover.
 * Day/month/year rollover is intentionally left simple — GPS sync
 * will correct any drift once per day.
 */
void updateSoftClock() {
  uint32_t now     = millis();
  uint32_t elapsed = now - clk.lastMs;
  if (elapsed < 1000) return;

  uint32_t secs    = elapsed / 1000;
  clk.lastMs      += secs * 1000;

  clk.second += secs;
  if (clk.second >= 60) { clk.minute += clk.second / 60; clk.second %= 60; }
  if (clk.minute >= 60) { clk.hour   += clk.minute / 60; clk.minute %= 60; }
  if (clk.hour   >= 24) { clk.day    += clk.hour   / 24; clk.hour   %= 24; }
  // Note: month/year rollover omitted — GPS sync corrects daily
}

/**
 * Return the current UTC time as a METAR time string: "HHMMz"
 * Example: "1256Z"
 */
String getTimeZuluString() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d%02dZ", clk.hour, clk.minute);
  return String(buf);
}

/**
 * GPS time synchronisation — STUB.
 *
 * To implement with the SIM7080G:
 *
 *   1. AT+CGNSPWR=1            — Power on the GNSS engine
 *   2. Wait a few seconds for fix
 *   3. AT+CGNSINF               — Query GNSS info
 *   4. Parse field 3 of the response: "YYYYMMDDHHmmss.0"
 *   5. Update clk.year, .month, .day, .hour, .minute, .second
 *   6. AT+CGNSPWR=0            — Power off GNSS to save battery
 *
 * The T-SIM7080G modem UART is typically Serial1 on the ESP32-S3.
 */
void syncGPSTime() {
  Serial.println(F("[GPS]   Daily time sync requested (stub — implement with modem AT commands)"));
  // TODO: Implement AT+CGNSINF parsing here
}


// ============================================================
//  METAR STRING BUILDER
// ============================================================

/**
 * Assemble a METAR-style observation string.
 *
 * Format:  CYUL 1256Z 14021G26KT 18/17 A2970 RMK
 *          ^^^^                                        Station ID
 *               ^^^^^                                  Observation time (UTC)
 *                     ^^^^^^^^^^                       Wind: DDD SS [Ggg] KT
 *                                 ^^^^^                Temp °C / Dew point °C
 *                                       ^^^^^          Altimeter (inHg × 100)
 *
 * Wind group rules:
 *   - Direction rounded to nearest 10°, 3 digits (e.g. 140)
 *   - Calm winds reported as 00000KT
 *   - Gust (Gxx) only appended when gust exceeds average by >= 5 kt
 *
 * Temperature/dew point rules:
 *   - Negative values prefixed with 'M' (e.g. M02 for −2°C)
 *   - Rounded to nearest integer
 *
 * Altimeter:
 *   - "A2970" means 29.70 inHg (multiply by 100, no decimal)
 */
String buildMETAR() {
  char buf[100];

  // --- Wind direction ---
  int windDir = 0;
  if (wx.windSpdKt >= 1.0f) {
    // Round to nearest 10 degrees
    windDir = (int)(wx.windDirDeg / 10.0f + 0.5f) * 10;
    if (windDir == 0)   windDir = 360;   // METAR uses 360, not 000, for northerly
    if (windDir == 370) windDir = 10;    // Wraparound safety
  }
  // Calm: windDir stays 0 → will print "000"

  // --- Wind speed & gust ---
  int windSpd = (int)(wx.windSpdKt + 0.5f);
  int windGst = (int)(wx.windGstKt + 0.5f);

  char gustStr[8] = "";
  if ((windGst - windSpd) >= 5) {
    snprintf(gustStr, sizeof(gustStr), "G%02d", windGst);
  }

  // --- Temperature / dew point (METAR negative value convention: M prefix) ---
  int tempInt  = (int)roundf(wx.tempC);
  int dewPtInt = (int)roundf(wx.dewPointC);

  char tempStr[8], dewStr[8];
  snprintf(tempStr, sizeof(tempStr), tempInt  < 0 ? "M%02d" : "%02d", abs(tempInt));
  snprintf(dewStr,  sizeof(dewStr),  dewPtInt < 0 ? "M%02d" : "%02d", abs(dewPtInt));

  // --- Altimeter setting: e.g. 29.70 inHg → "A2970" ---
  int altimInt = (int)(wx.altimInHg * 100.0f + 0.5f);

  snprintf(buf, sizeof(buf),
    "%s %s %03d%02d%sKT %s/%s A%04d RMK",
    STATION_ID,
    getTimeZuluString().c_str(),
    windDir,
    windSpd,
    gustStr,
    tempStr,
    dewStr,
    altimInt
  );

  return String(buf);
}
