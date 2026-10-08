/**
 * @file    T7080G_WX_Station_combined.ino
 * @brief   Weather station: AS5048A wind vane, BMP390 pressure/temp,
 *          SHT31 temp/humidity — SMS receive/reply with METAR-style output.
 *          Board: LilyGo T-SIM7080G (ESP32-S3 + AXP2101 PMU)
 *
 * =============================================================================
 * COMBINED v5 SMS-ECHO + WX-STATION  (merged)
 * =============================================================================
 *
 * KEY DESIGN DECISIONS (consolidated from both versions)
 * -------------------------------------------------------
 *
 * 1. PSM STRATEGY — CFUN=0/1 CYCLE EVERY 90 SECONDS  (from v5 SMS-only)
 *    AT+CPSMS=0 alone only sets a local modem flag. The PSM timers T3324/T3412
 *    were negotiated during the last Attach/TAU exchange and remain active on
 *    the network side until a new Attach is sent with PSM disabled.
 *
 *    CFUN=0 (minimum functionality / radio off) de-registers from the network.
 *    CFUN=1 (full functionality) re-registers, and the Attach message sent
 *    during this re-registration includes PSM=disabled, which Telus honours.
 *
 *    disablePSM() is called ONCE at startup and then EVERY 90 seconds in the
 *    watchdog to guard against network-forced PSM during a TAU exchange.
 *    configureSMS() MUST be called after every disablePSM() because CFUN=0
 *    resets +CNMI and +CMGF back to modem defaults.
 *
 * 2. CORRECTED PWRKEY POLARITY  (from v5)
 *    Per LilyGo official examples (Modem-Series/issues/265):
 *      LOW(100 ms) -> HIGH(1000 ms) -> LOW  =  correct power-on sequence.
 *    Older versions had HIGH->LOW which caused unreliable power-on.
 *    PWRKEY is only used at boot; recovery uses AT+CREBOOT.
 *
 * 3. HARDWARE WATCHDOG
 *    60-second WDT covers total modem silence (PSM fired and UART unresponsive).
 *    The ESP32 reboots and setup() runs fresh — cleaner than any recovery loop.
 *    The WDT timeout must be longer than the CFUN=0->1 re-registration (~25 s).
 *
 * 4. SMS CONFIG AFTER EVERY CFUN CYCLE
 *    configureSMS() is called after every disablePSM() call because CFUN=0
 *    resets +CNMI and +CMGF to modem defaults.
 *
 * 5. MISSED SMS RECOVERY
 *    checkForMissedSMS() runs after every configureSMS() call. Any +CMTI URC
 *    that arrived during the CFUN=0/1 blackout (~25 s of modem offline time)
 *    is silently lost. This function scans the SIM for unread messages and
 *    processes them so no text goes unanswered.
 *
 * =============================================================================
 * I2C BUS LAYOUT
 * =============================================================================
 *   Wire  (bus 0)  SDA=GPIO15, SCL=GPIO7  — AXP2101 PMU only (Qwiic connector)
 *   Wire1_SHT (bus 1)  SDA=GPIO13, SCL=GPIO14  — SHT31 sensor (GPIO header)
 *
 * The modem communicates over UART (GPIO4/5), no I2C involvement.
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_SHT31.h"
#include <esp_task_wdt.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
// I2C bus 0 — PMU only (internal, accessed via Qwiic connector)
#define I2C_SDA             15
#define I2C_SCL             7

// I2C bus 1 — SHT31 sensor (GPIO header, free for external use)
#define SHT31_SDA           13
#define SHT31_SCL           14

// SPI — AS5048A magnetic encoder + BMP390 barometer
#define SPI_SCK             18
#define SPI_MISO            16
#define SPI_MOSI            17
#define SPI_CS_AS5048       10
#define SPI_CS_BMP          12

// Modem UART (LilyGo T-SIM7080G)
#define BOARD_MODEM_RXD_PIN 4
#define BOARD_MODEM_TXD_PIN 5
#define BOARD_MODEM_PWR_PIN 41

// Wind speed hall effect sensor — active-LOW, one FALLING edge per cup rotation
#define WIND_SPEED_PIN      9

// Pressure reference for altitude calculation
#define SEALEVELPRESSURE_HPA  1013.25f

// Station altitude above MSL in metres — used for altimeter setting reduction.
// Example: CYUL Montreal = 36 m
#define STATION_ALTITUDE_M    36.0f

// ── Wind speed constants ──────────────────────────────────────────────────────
#define WIND_CUP_RADIUS_M   0.12f
#define WIND_CUP_CIRC_M     (2.0f * (float)M_PI * WIND_CUP_RADIUS_M)  // ~0.7540 m/rotation
#define MS_TO_KNOTS         1.94384f
#define WIND_HISTORY_SIZE   10   // One entry per minute; 10 = 10 min of history
#define WIND_AVG_MINUTES     2   // Minutes to average for reported wind speed

// ── AS5048A register ──────────────────────────────────────────────────────────
const uint16_t AS5_READ_ANGLE = 0x3FFF;   // 14-bit angle register

// ── Wind speed — ISR state (volatile + mutex protected) ──────────────────────
volatile uint32_t windPulseCount  = 0;
portMUX_TYPE      windMux         = portMUX_INITIALIZER_UNLOCKED;

// ── Wind speed — circular history buffer (one entry per minute) ──────────────
static float   windSpeedHistory[WIND_HISTORY_SIZE] = {};
static uint8_t windHistoryIndex = 0;
static uint8_t windHistoryCount = 0;

// ── TinyGSM ──────────────────────────────────────────────────────────────────
#define TINY_GSM_RX_BUFFER      1024
#define SerialAT                Serial1
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>

// Uncomment ONLY for AT-level debugging (adds per-byte serial overhead)
// #define DUMP_AT_COMMANDS
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// ── PMU ───────────────────────────────────────────────────────────────────────
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
XPowersPMU PMU;

// ── Sensors ───────────────────────────────────────────────────────────────────
Adafruit_BMP3XX  bmp;
TwoWire          Wire1_SHT = TwoWire(1);
Adafruit_SHT31   sht31(&Wire1_SHT);

// ── Hardware watchdog ─────────────────────────────────────────────────────────
#define WDT_TIMEOUT_SEC     60

// ── Timing ────────────────────────────────────────────────────────────────────
#define WATCHDOG_MS         90000UL
#define SENSOR_MS           60000UL

// ── Runtime state ─────────────────────────────────────────────────────────────
static String   rxLine        = "";
static uint32_t lastWatchdog  = 0;
static uint32_t lastSensor    = 0;
static bool     smsInFlight   = false;

struct WxSnapshot {
    float   windDeg    = 0.0f;
    float   windSpdKt  = 0.0f;
    float   windGstKt  = 0.0f;
    float   tempC      = NAN;
    float   pressHPa   = NAN;
    float   altimInHg  = NAN;
    float   shtTempC   = NAN;
    float   shtHumPct  = NAN;
    bool    valid      = false;
};
static WxSnapshot wx;

// ── Forward declarations ──────────────────────────────────────────────────────
void     initializePMU();
void     initializeSensors();
void     initializeModem();
void     connectToNetwork();
void     disablePSM();
void     configureSMS();
void     checkForMissedSMS();
void     readSensors();
String   buildMETAR();
void     handleIncomingSMS(int idx);
void     networkWatchdog();
String   readLineTimeout(uint32_t ms);
uint16_t readAS5048A(uint16_t addr);
void IRAM_ATTR windPulseISR();
float    getWindSpeedKnots(uint32_t pulses, uint32_t intervalMs);
float    getWindAvgKnots();
float    getWindGustKnots();

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=========================================");
    Serial.println(" T-SIM7080G Weather Station + SMS");
    Serial.println(" (combined v5-SMS + WX build)");
    Serial.println("=========================================\n");

    esp_task_wdt_config_t wdt = {
        .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = (1 << 0),
        .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&wdt);
    esp_task_wdt_add(NULL);

    // I2C buses — same order as working Doc 6
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire1_SHT.begin(SHT31_SDA, SHT31_SCL);

    initializePMU();
    esp_task_wdt_reset();

    initializeSensors();
    esp_task_wdt_reset();

    initializeModem();
    esp_task_wdt_reset();
    connectToNetwork();
    esp_task_wdt_reset();

    disablePSM();
    configureSMS();
    checkForMissedSMS();

    SerialAT.setTimeout(500);
    lastWatchdog = millis();
    lastSensor   = millis() - SENSOR_MS;    // Force immediate first sensor read

    Serial.println("\n[READY] PSM disabled. Waiting for SMS...\n");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    esp_task_wdt_reset();

    if (millis() - lastSensor >= SENSOR_MS) {
        readSensors();
        lastSensor = millis();
    }

    while (SerialAT.available()) {
        char c = (char)SerialAT.read();
        if (c == '\n') {
            rxLine.trim();
            if (rxLine.length() > 0) {
                if (!smsInFlight && rxLine.startsWith("+CMTI:")) {
                    int comma = rxLine.lastIndexOf(',');
                    if (comma != -1) {
                        handleIncomingSMS(rxLine.substring(comma + 1).toInt());
                    }
                }
                if (rxLine.indexOf("+CPSMSTATUS: ENTER PSM") >= 0) {
                    Serial.println("[WARN] PSM entered — watchdog will re-disable.");
                }
            }
            rxLine = "";
        } else if (c != '\r') {
            rxLine += c;
            if (rxLine.length() > 512) rxLine = "";
        }
    }

    while (Serial.available()) {
        SerialAT.write(Serial.read());
    }

    if (!smsInFlight && millis() - lastWatchdog >= WATCHDOG_MS) {
        networkWatchdog();
        lastWatchdog = millis();
    }
}

// =============================================================================
//  PMU
// =============================================================================
void initializePMU() {
    Serial.println("[PMU] Configuring AXP2101...");
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("[PMU] ERROR: AXP2101 not found! Halting.");
        while (1) { delay(1000); }
    }

    PMU.disableDC2();
    PMU.disableDC3();
    PMU.disableDC4();
    PMU.disableDC5();
    PMU.disableALDO1(); PMU.disableALDO2();
    PMU.disableALDO3(); PMU.disableALDO4();
    PMU.disableBLDO2();
    PMU.disableCPUSLDO();
    PMU.disableDLDO1(); PMU.disableDLDO2();

    PMU.setDC3Voltage(3000);
    PMU.enableDC3();

    PMU.setBLDO1Voltage(3300);
    PMU.enableBLDO1();

    PMU.setBLDO2Voltage(3300);
    PMU.enableBLDO2();

    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
    Serial.println("[PMU] Rails stable. Modem VCC=3V, BLDO1=3.3V, BLDO2=3.3V.");
}

// =============================================================================
//  SENSOR INIT  — exact init sequence from working Doc 6
// =============================================================================
void initializeSensors() {
    pinMode(SPI_CS_AS5048, OUTPUT);
    digitalWrite(SPI_CS_AS5048, HIGH);
    pinMode(SPI_CS_BMP, OUTPUT);
    digitalWrite(SPI_CS_BMP, HIGH);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("[SPI] Bus initialized.");

    // Single-argument begin_SPI — matches working Doc 6 exactly
    if (!bmp.begin_SPI(SPI_CS_BMP)) {
        Serial.println("[BMP390] WARNING: Sensor not found — pressure/temp unavailable.");
    } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        Serial.println("[BMP390] Initialized.");
    }

    if (!sht31.begin(0x44)) {
        Serial.println("[SHT31] WARNING: Sensor not found at 0x44 — humidity/temp unavailable.");
    } else {
        Serial.println("[SHT31] Initialized.");
        Serial.print("[SHT31] Heater: ");
        Serial.println(sht31.isHeaterEnabled() ? "ENABLED" : "DISABLED");
    }

    pinMode(WIND_SPEED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), windPulseISR, FALLING);
    memset(windSpeedHistory, 0, sizeof(windSpeedHistory));
    Serial.println("[WIND] Hall effect ISR attached on GPIO " + String(WIND_SPEED_PIN) + " (FALLING edge).");
}

// =============================================================================
//  SENSOR READ
// =============================================================================
uint16_t readAS5048A(uint16_t addr) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(SPI_CS_AS5048, LOW);
    uint16_t result = SPI.transfer16(addr);
    digitalWrite(SPI_CS_AS5048, HIGH);
    SPI.endTransaction();
    return result & 0x3FFF;
}

void readSensors() {
    // ── Wind speed ────────────────────────────────────────────────────────────
    uint32_t pulses;
    portENTER_CRITICAL(&windMux);
    pulses = windPulseCount;
    windPulseCount = 0;
    portEXIT_CRITICAL(&windMux);

    float minuteWindKt = getWindSpeedKnots(pulses, SENSOR_MS);
    windSpeedHistory[windHistoryIndex] = minuteWindKt;
    windHistoryIndex = (windHistoryIndex + 1) % WIND_HISTORY_SIZE;
    if (windHistoryCount < WIND_HISTORY_SIZE) windHistoryCount++;

    wx.windSpdKt = getWindAvgKnots();
    wx.windGstKt = getWindGustKnots();

    // ── Wind direction (AS5048A) ──────────────────────────────────────────────
    uint16_t rawAngle = readAS5048A(AS5_READ_ANGLE);
    wx.windDeg = ((rawAngle & 0x3FFF) * 360.0f) / 16384.0f;

    // ── Pressure + temperature (BMP390) ──────────────────────────────────────
    if (bmp.performReading()) {
        wx.pressHPa = bmp.pressure / 100.0f;
        wx.tempC    = bmp.temperature;

        float T_K     = (isnan(wx.shtTempC) ? wx.tempC : wx.shtTempC) + 273.15f;
        float ratio   = (T_K + 0.0065f * STATION_ALTITUDE_M) / T_K;
        float qnh_hPa = wx.pressHPa * powf(ratio, 5.2561f);
        wx.altimInHg  = qnh_hPa * 0.02953f;
    } else {
        Serial.println("[BMP390] Read failed.");
    }

    // ── Temperature + humidity (SHT31) ───────────────────────────────────────
    wx.shtTempC  = sht31.readTemperature();
    wx.shtHumPct = sht31.readHumidity();

    if (isnan(wx.shtTempC) || isnan(wx.shtHumPct)) {
        Serial.println("[SHT31] Read failed.");
    }

    wx.valid = true;

    Serial.println("---[ Sensor Update ]---");
    Serial.printf("  Wind Dir : %.1f deg\n",   wx.windDeg);
    Serial.printf("  Wind Spd : %.1f kt (2min avg) | Gust %.1f kt (10min peak)\n",
                  wx.windSpdKt, wx.windGstKt);
    Serial.printf("  Pressure : %.2f hPa | Altimeter %.2f inHg\n", wx.pressHPa, wx.altimInHg);
    Serial.printf("  Temp BMP : %.1f C\n",     wx.tempC);
    Serial.printf("  Temp SHT : %.1f C\n",     wx.shtTempC);
    Serial.printf("  Humidity : %.1f %%\n",    wx.shtHumPct);
    Serial.println("---[ METAR ]---");
    Serial.println(buildMETAR());
    Serial.println("---------------");
}

// =============================================================================
//  METAR BUILDER
// =============================================================================
String buildMETAR() {
    uint32_t uptimeSec = millis() / 1000;
    uint8_t  hh = (uptimeSec / 3600) % 24;
    uint8_t  mm = (uptimeSec / 60)   % 60;
    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "00%02u%02uZ", hh, mm);

    int wndDir = 0;
    if (wx.windSpdKt >= 0.1f) {
        wndDir = (int)(wx.windDeg / 10.0f + 0.5f) * 10;
        if (wndDir == 0)   wndDir = 360;
        if (wndDir >= 360) wndDir = 0;
    }

    int windSpd = (int)(wx.windSpdKt + 0.5f);
    int windGst = (int)(wx.windGstKt + 0.5f);

    char wndStr[20];
    if ((windGst - windSpd) >= 5) {
        snprintf(wndStr, sizeof(wndStr), "%03d%02dG%02dKT", wndDir, windSpd, windGst);
    } else {
        snprintf(wndStr, sizeof(wndStr), "%03d%02dKT", wndDir, windSpd);
    }

    char tStr[16]  = "//";
    char dpStr[16] = "//";
    if (!isnan(wx.shtTempC) && !isnan(wx.shtHumPct)) {
        int   tRound = (int)roundf(wx.shtTempC);
        float a  = 17.625f, b = 243.04f;
        float rh = wx.shtHumPct / 100.0f;
        float dp = b * (logf(rh) + (a * wx.shtTempC) / (b + wx.shtTempC))
                     / (a - logf(rh) - (a * wx.shtTempC) / (b + wx.shtTempC));
        int   dpRound = (int)roundf(dp);
        if (tRound  < 0) snprintf(tStr,  sizeof(tStr),  "M%02d", -tRound);
        else             snprintf(tStr,  sizeof(tStr),  "%02d",   tRound);
        if (dpRound < 0) snprintf(dpStr, sizeof(dpStr), "M%02d", -dpRound);
        else             snprintf(dpStr, sizeof(dpStr), "%02d",   dpRound);
    }

    char altStr[8] = "A////";
    if (!isnan(wx.altimInHg)) {
        snprintf(altStr, sizeof(altStr), "A%04d", (int)(wx.altimInHg * 100.0f + 0.5f));
    }

    char rhStr[12] = "RH=//%";
    if (!isnan(wx.shtHumPct)) {
        snprintf(rhStr, sizeof(rhStr), "RH=%02d%%", (int)roundf(wx.shtHumPct));
    }

    char metar[160];
    snprintf(metar, sizeof(metar),
             "METAR CYND %s %s %s/%s %s %s",
             timeStr, wndStr, tStr, dpStr, altStr, rhStr);
    return String(metar);
}

// =============================================================================
//  DISABLE PSM — CFUN CYCLE
// =============================================================================
void disablePSM() {
    Serial.println("[PSM] Disabling PSM via CFUN cycle...");
    esp_task_wdt_reset();

    modem.sendAT(GF("+CPSMS=0"));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CEDRXS=0,4"));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CEDRXS=0,5"));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CSCLK=0"));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CPSMSTATUS=1"));
    modem.waitResponse(3000L);

    Serial.println("[PSM] CFUN=0 (detaching)...");
    modem.sendAT(GF("+CFUN=0"));
    modem.waitResponse(6000L);
    esp_task_wdt_reset();
    delay(2000);
    esp_task_wdt_reset();

    Serial.println("[PSM] CFUN=1 (re-attaching with PSM disabled)...");
    modem.sendAT(GF("+CFUN=1"));
    modem.waitResponse(6000L);
    esp_task_wdt_reset();

    Serial.print("[PSM] Waiting for registration");
    uint32_t start = millis();
    while (!modem.isNetworkConnected()) {
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset();
        if (millis() - start > 60000UL) {
            Serial.println("\n[PSM] Registration timeout after CFUN cycle.");
            return;
        }
    }
    Serial.println("\n[PSM] Registered. PSM disabled at network level.");
}

// =============================================================================
//  NETWORK WATCHDOG  (90 s)
// =============================================================================
void networkWatchdog() {
    Serial.println("[NET] Watchdog tick...");

    if (!modem.isNetworkConnected()) {
        Serial.println("[NET] Not registered — soft reset via CREBOOT...");
        modem.sendAT(GF("+CREBOOT"));
        delay(9000);
        esp_task_wdt_reset();
        int r = 0;
        while (!modem.testAT(1000)) {
            esp_task_wdt_reset();
            if (r++ > 10) break;
        }
        connectToNetwork();
    }

    disablePSM();
    configureSMS();
    checkForMissedSMS();

    Serial.printf("[NET] OK | RSSI=%d\n", modem.getSignalQuality());
}

// =============================================================================
//  CHECK FOR MISSED SMS  (called after every CFUN cycle)
// =============================================================================
/**
 * Any +CMTI URC that arrived while the modem was offline during a CFUN=0/1
 * cycle is silently lost. This scans all unread SIM slots and handles them.
 */
void checkForMissedSMS() {
    if (smsInFlight) return;
    Serial.println("[SMS] Scanning for missed messages...");

    modem.sendAT(GF("+CMGL=\"REC UNREAD\""));

    String response = "";
    uint32_t deadline = millis() + 5000;
    while (millis() < deadline) {
        while (SerialAT.available()) {
            char c = (char)SerialAT.read();
            response += c;
            if (response.length() > 1024) break;
        }
        if (response.indexOf("\nOK") >= 0 || response.indexOf("\nERROR") >= 0) break;
        delay(10);
        esp_task_wdt_reset();
    }

    int searchPos = 0;
    while (true) {
        int cmglPos = response.indexOf("+CMGL:", searchPos);
        if (cmglPos < 0) break;
        int commaPos = response.indexOf(",", cmglPos);
        if (commaPos < 0) break;
        String idxStr = response.substring(cmglPos + 6, commaPos);
        idxStr.trim();
        int idx = idxStr.toInt();
        Serial.printf("[SMS] Found missed message in slot %d\n", idx);
        handleIncomingSMS(idx);
        searchPos = commaPos + 1;
        esp_task_wdt_reset();
    }
}

// =============================================================================
//  SMS HANDLER
// =============================================================================
void handleIncomingSMS(int index) {
    smsInFlight = true;
    Serial.printf("\n[SMS] Reading slot %d...\n", index);

    modem.sendAT(GF("+CMGR="), index);
    if (modem.waitResponse(5000L, GF("+CMGR:")) != 1) {
        Serial.println("[SMS] ERROR: No +CMGR response.");
        smsInFlight = false;
        return;
    }

    String header = readLineTimeout(2000);
    String body   = readLineTimeout(2000);
    body.trim();
    Serial.println("[SMS] Header: " + header);
    Serial.println("[SMS] Body  : " + body);

    String sender = "";
    int p1 = header.indexOf(",\"");
    if (p1 != -1) {
        int p2 = header.indexOf("\"", p1 + 2);
        if (p2 != -1) sender = header.substring(p1 + 2, p2);
    }

    modem.waitResponse(2000L);

    if (sender.length() == 0) {
        Serial.println("[SMS] ERROR: Sender parse failed: " + header);
        modem.sendAT(GF("+CMGD="), index);
        modem.waitResponse(3000L);
        smsInFlight = false;
        return;
    }
    Serial.println("[SMS] From: " + sender);

    String reply;
    if (wx.valid) {
        reply = buildMETAR();
    } else {
        reply = "WX Station: No sensor data yet.";
    }
    if (reply.length() > 155) reply = reply.substring(0, 155);

    if (modem.sendSMS(sender.c_str(), reply.c_str())) {
        Serial.println("[SMS] Reply sent: " + reply);
    } else {
        Serial.println("[SMS] ERROR: sendSMS failed.");
    }

    modem.sendAT(GF("+CMGD="), index);
    modem.waitResponse(3000L);
    Serial.println("[SMS] Slot cleared.\n");

    smsInFlight = false;
}

// =============================================================================
//  SMS CONFIG
// =============================================================================
void configureSMS() {
    modem.sendAT(GF("+CMGF=1"));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CSCS=\"GSM\""));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CPMS=\"SM\",\"SM\",\"SM\""));
    modem.waitResponse(3000L);
    modem.sendAT(GF("+CNMI=1,1,0,0,0"));
    modem.waitResponse(3000L);
    Serial.println("[SMS] Configured.");
}

// =============================================================================
//  MODEM INIT
// =============================================================================
void initializeModem() {
    SerialAT.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
    pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);

    digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_MODEM_PWR_PIN, LOW);

    Serial.print("[MODEM] Waiting for AT");
    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.print(".");
        esp_task_wdt_reset();
        if (retry++ > 10) {
            Serial.println("\n[MODEM] Retrying power-on pulse...");
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);  delay(100);
            digitalWrite(BOARD_MODEM_PWR_PIN, HIGH); delay(1000);
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
            delay(3000);
            esp_task_wdt_reset();
            retry = 0;
        }
    }
    Serial.println("\n[MODEM] Ready.");
}

// =============================================================================
//  NETWORK CONNECT
// =============================================================================
void connectToNetwork() {
    Serial.print("[NET] Waiting for registration");
    uint32_t start = millis();
    while (!modem.isNetworkConnected()) {
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset();
        if (millis() - start > 60000UL) {
            Serial.println("\n[NET] Registration timeout — will retry next watchdog.");
            return;
        }
    }
    Serial.println();
    Serial.println("[NET] Operator : " + modem.getOperator());
    Serial.println("[NET] Signal   : " + String(modem.getSignalQuality()));
}

// =============================================================================
//  WIND SPEED — HALL EFFECT SENSOR (GPIO 9)
// =============================================================================
void IRAM_ATTR windPulseISR() {
    portENTER_CRITICAL_ISR(&windMux);
    windPulseCount++;
    portEXIT_CRITICAL_ISR(&windMux);
}

float getWindSpeedKnots(uint32_t pulses, uint32_t intervalMs) {
    if (intervalMs == 0 || pulses == 0) return 0.0f;
    float rps = (float)pulses / (intervalMs / 1000.0f);
    float mps = rps * WIND_CUP_CIRC_M;
    return mps * MS_TO_KNOTS;
}

float getWindAvgKnots() {
    if (windHistoryCount == 0) return 0.0f;
    uint8_t count = (windHistoryCount < WIND_AVG_MINUTES) ? windHistoryCount : WIND_AVG_MINUTES;
    float   sum   = 0.0f;
    for (int i = 0; i < count; i++) {
        int idx = ((int)windHistoryIndex - 1 - i + WIND_HISTORY_SIZE) % WIND_HISTORY_SIZE;
        sum += windSpeedHistory[idx];
    }
    return sum / (float)count;
}

float getWindGustKnots() {
    if (windHistoryCount == 0) return 0.0f;
    float   maxVal = 0.0f;
    uint8_t count  = (windHistoryCount < WIND_HISTORY_SIZE) ? windHistoryCount : WIND_HISTORY_SIZE;
    for (int i = 0; i < count; i++) {
        if (windSpeedHistory[i] > maxVal) maxVal = windSpeedHistory[i];
    }
    return maxVal;
}

// =============================================================================
//  HELPERS
// =============================================================================
String readLineTimeout(uint32_t ms) {
    String   line     = "";
    uint32_t deadline = millis() + ms;
    while (millis() < deadline) {
        while (SerialAT.available()) {
            char c = (char)SerialAT.read();
            if (c == '\n') return line;
            if (c != '\r') line += c;
        }
        delay(1);
        esp_task_wdt_reset();
    }
    return line;
}
