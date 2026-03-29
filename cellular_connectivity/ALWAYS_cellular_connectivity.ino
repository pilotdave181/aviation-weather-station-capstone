/**
 * @file      cellular_connectivity.ino
 * @brief     Connect to Telus network, listen for incoming SMS, reply with hello
 * @author    Based on ATDebug.ino by Lewis He
 * @license   MIT
 */

#include <Arduino.h>
#include "utilities.h"

#define TINY_GSM_RX_BUFFER 1024
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

// --- Globals for SMS listening ---
String incomingBuffer = "";
String lastSenderNumber = "";

// --- Function prototypes ---
void initializePMU();
void initializeModem();
void connectToTelus();
void configureSMSListening();
void printModemInfo();
void checkForIncomingSMS(String line);
void sendSMSReply(String recipient);

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n================================");
    Serial.println("SIM7080 SMS Listener - Telus");
    Serial.println("================================\n");

    // Initialize power management
    initializePMU();

    // Initialize modem
    initializeModem();

    // Print modem information
    printModemInfo();

    // Connect to Telus network
    connectToTelus();

    // Configure modem to push incoming SMS to serial
    configureSMSListening();

    Serial.println("\n[READY] Listening for incoming SMS...\n");
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
    // Read every byte the modem sends, one character at a time
    while (SerialAT.available()) {
        char c = SerialAT.read();
        Serial.write(c);            // mirror to Serial Monitor for debugging

        incomingBuffer += c;

        // When we hit a newline we have a complete line — check it
        if (c == '\n') {
            checkForIncomingSMS(incomingBuffer);
            incomingBuffer = "";    // clear buffer for next line
        }
    }

    // Still allow manual AT commands from Serial Monitor
    while (Serial.available()) {
        SerialAT.write(Serial.read());
    }

    delay(1);
}

// ============================================================
//  SMS LISTENER
// ============================================================
void checkForIncomingSMS(String line)
{
    line.trim();

    if (line.length() == 0) return;

    // When SMS arrives modem sends:
    // +CMT: "+15551234567","","25/03/29,10:00:00+00"
    // followed by the message body on the next line
    if (line.startsWith("+CMT:")) {
        Serial.println("\n[SMS] >>> Incoming SMS detected!");

        // Extract sender number between the first pair of quotes
        int firstQuote  = line.indexOf('"');
        int secondQuote = line.indexOf('"', firstQuote + 1);

        if (firstQuote != -1 && secondQuote != -1) {
            lastSenderNumber = line.substring(firstQuote + 1, secondQuote);
            Serial.println("[SMS] Sender: " + lastSenderNumber);
        } else {
            Serial.println("[SMS][WARN] Could not parse sender number");
        }
        // Don't reply yet — wait for the message body on the next line
        return;
    }

    // If lastSenderNumber is set, this line is the message body
    if (lastSenderNumber.length() > 0) {
        Serial.println("[SMS] Message body: \"" + line + "\"");
        Serial.println("[SMS] Replying to: " + lastSenderNumber);

        sendSMSReply(lastSenderNumber);

        lastSenderNumber = "";  // reset — ready for next SMS
    }
}

// ============================================================
//  REPLY
// ============================================================
void sendSMSReply(String recipient)
{
    String reply = "Hello " + recipient + ", how are you?";

    Serial.println("[SMS] Sending: \"" + reply + "\"");

    // Make sure we are in text mode before sending
    modem.sendAT(GF("+CMGF=1"));
    modem.waitResponse();

    if (modem.sendSMS(recipient.c_str(), reply.c_str())) {
        Serial.println("[SMS] Reply sent successfully!");
    } else {
        Serial.println("[ERROR] Failed to send reply — check network");
    }
}

// ============================================================
//  CONFIGURE SMS LISTENING
// ============================================================
void configureSMSListening()
{
    Serial.println("[SMS] Configuring SMS text mode and push notifications...");

    // Set text mode (not PDU binary mode)
    modem.sendAT(GF("+CMGF=1"));
    if (modem.waitResponse() == 1) {
        Serial.println("[SMS] Text mode: OK");
    }

    // AT+CNMI=2,2,0,0,0
    // Tells the modem: when a new SMS arrives, push it directly
    // to the serial port as a +CMT: line instead of storing it silently
    modem.sendAT(GF("+CNMI=2,2,0,0,0"));
    if (modem.waitResponse() == 1) {
        Serial.println("[SMS] Push notifications: OK");
    } else {
        Serial.println("[ERROR] CNMI setup failed — SMS listening may not work");
    }
}

// ============================================================
//  PMU INIT — unchanged from your working code
// ============================================================
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

// ============================================================
//  MODEM INIT — unchanged from your working code
// ============================================================
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

// ============================================================
//  MODEM INFO — unchanged from your working code
// ============================================================
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

// ============================================================
//  NETWORK CONNECT — unchanged from your working code
// ============================================================
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