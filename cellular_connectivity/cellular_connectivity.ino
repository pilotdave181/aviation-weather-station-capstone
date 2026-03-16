/**
 * @file      SIM7080_SMS_Telus.ino
 * @brief     Connect to Telus network and send SMS messages
 * @author    Based on ATDebug.ino by Lewis He
 * @license   MIT
 * 
 * This sketch demonstrates how to:
 * 1. Initialize the SIM7080 modem
 * 2. Register on the Telus network
 * 3. Send SMS messages
 */

#include <Arduino.h>
#include "utilities.h"

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// Uncomment to see all AT commands being sent
#define DUMP_AT_COMMANDS

#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
XPowersPMU PMU;

// Configuration
const char* phoneNumber = "+1234567890";  // Replace with target phone number
const char* smsMessage = "KJFK 121856Z 18008KT 10SM FEW250 22/14 A3012 RMK TEST";

// Function prototypes
void initializePMU();
void initializeModem();
void connectToTelus();
void sendSMS();
void printModemInfo();

void setup()
{
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n================================");
    Serial.println("SIM7080 SMS to Telus Network");
    Serial.println("================================\n");

    // Initialize power management
    initializePMU();
    
    // Initialize modem
    initializeModem();
    
    // Print modem information
    printModemInfo();
    
    // Connect to Telus network
    connectToTelus();
    
    // Send SMS
    delay(2000);
    sendSMS();
}

void loop()
{
    // Echo AT commands from Serial to modem and back
    while (SerialAT.available()) {
        Serial.write(SerialAT.read());
    }
    while (Serial.available()) {
        SerialAT.write(Serial.read());
    }
    delay(1);
}

/**
 * @brief Initialize power management unit
 */
void initializePMU()
{
    Serial.println("[PMU] Initializing power management...");
    
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("[ERROR] Failed to initialize power!");
        while (1);
    }

    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);

    // Disable unused power domains
    PMU.disableDC2();
    PMU.disableDC4();
    PMU.disableDC5();
    PMU.disableALDO1();
    PMU.disableALDO2();
    PMU.disableALDO3();
    PMU.disableALDO4();
    PMU.disableBLDO2();
    PMU.disableCPUSLDO();
    PMU.disableDLDO1();
    PMU.disableDLDO2();

    // Power supplies
    PMU.setBLDO1Voltage(3300);    // Level conversion
    PMU.enableBLDO1();

    PMU.setDC3Voltage(3000);      // SIM7080 main power
    PMU.enableDC3();

    PMU.setBLDO2Voltage(3300);    // GPS antenna
    PMU.enableBLDO2();

    Serial.println("[PMU] Power initialized successfully");
}

/**
 * @brief Initialize modem communication
 */
void initializeModem()
{
    Serial.println("[MODEM] Starting modem initialization...");
    
    Serial1.begin(115200, SERIAL_8N1, BOARD_MODEM_RXD_PIN, BOARD_MODEM_TXD_PIN);
    pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.print(".");
        if (retry++ > 10) {
            // Power cycle the modem
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
            retry = 0;
            Serial.println("\n[ERROR] Modem connection failed. Retrying...");
        }
    }
    Serial.println("\n[MODEM] Modem initialized successfully");
}

/**
 * @brief Print modem information (IMEI, manufacturer, etc.)
 */
void printModemInfo()
{
    Serial.println("\n[INFO] Modem Information:");
    Serial.println("  Manufacturer: " + modem.getModemInfo());
    Serial.println("  Model: " + modem.getModemModel());
    Serial.println("  IMEI: " + modem.getIMEI());
    
    SimStatus sim = modem.getSimStatus();
    Serial.print("  SIM Status: ");
    switch (sim) {
        case SIM_READY:
            Serial.println("Ready");
            break;
        case SIM_LOCKED:
            Serial.println("Locked (PIN required)");
            break;
        default:
            Serial.println("Unknown");
    }
}

/**
 * @brief Connect to Telus network
 */
void connectToTelus()
{
    Serial.println("\n[NETWORK] Attempting to connect to Telus network...");
    
    // Unlock SIM if needed (optional - modify PIN as needed)
    // modem.simUnlock("1234");
    
    // Wait for network registration
    int attempts = 0;
    while (!modem.isNetworkConnected()) {
        Serial.print(".");
        delay(500);
        attempts++;
        if (attempts > 60) {  // 30 seconds timeout
            Serial.println("\n[ERROR] Network connection timeout!");
            break;
        }
    }
    
    if (modem.isNetworkConnected()) {
        Serial.println("\n[SUCCESS] Connected to network!");
        
        // Get network operator name
        String op = modem.getOperator();
        Serial.println("  Operator: " + op);
        
        // Get signal quality (0-31, higher is better)
        Serial.print("  Signal Quality: ");
        Serial.println(modem.getSignalQuality());
    } else {
        Serial.println("\n[ERROR] Failed to connect to network");
    }
}

/**
 * @brief Send SMS message
 */
void sendSMS()
{
    Serial.println("\n[SMS] Preparing to send SMS...");
    Serial.println("  To: " + String(phoneNumber));
    Serial.println("  Message: " + String(smsMessage));
    
    // Set SMS mode to text (not PDU)
    modem.sendAT(GF("+CMGF=1"));
    modem.waitResponse();
    
    // Send SMS
    if (modem.sendSMS(phoneNumber, smsMessage)) {
        Serial.println("\n[SUCCESS] SMS sent successfully!");
    } else {
        Serial.println("\n[ERROR] Failed to send SMS");
        
        // Print error details
        Serial.println("  Checking network status...");
        if (!modem.isNetworkConnected()) {
            Serial.println("  Not connected to network");
        }
        if (!modem.isGprsConnected()) {
            Serial.println("  GPRS not connected");
        }
    }
}
