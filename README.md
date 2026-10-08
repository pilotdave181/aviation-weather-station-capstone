# Aviation Weather Station with ESP32-S3 + SMS (METAR-Style) Reply

## Overview
This weather station uses a LilyGo T-SIM7080G (ESP32-S3 with a SIM7080G cellular modem and AXP2101 PMU) along with several sensors to measure wind, temperature, humidity, and pressure. Instead of uploading to a cloud dashboard, the station answers by SMS: text it from any phone and it replies with a METAR-style observation. This makes it usable anywhere with cellular coverage, with no Wi-Fi required.
<img width="899" height="674" alt="image" src="https://github.com/user-attachments/assets/32b3f20a-f57d-4703-b94d-a24845458d90" />



## Features
- Wind Speed: Measures wind speed with a Hall effect sensor and cup anemometer, reported as a 2-minute average with a 10-minute gust peak.
- Wind Direction: Determines wind direction with an AS5048A 14-bit magnetic encoder over SPI.
- Pressure: Uses a BMP390 barometer over SPI, reduced to an altimeter setting (QNH) in inHg using the station altitude.
- Temperature and Humidity: Uses an SHT31 sensor for temperature and relative humidity, with dew point calculated on the device.
- METAR-Style Output: Formats wind, gusts, temperature/dew point, altimeter, and humidity in standard METAR notation.
- SMS Interface: Replies to any incoming text with the latest observation.
- Reliability: Hardware watchdog, a periodic modem cycle to keep power saving mode (PSM) off, and recovery of texts missed during modem restarts.

## Components
- LilyGo T-SIM7080G (ESP32-S3, SIM7080G modem, AXP2101 PMU)
- AS5048A (magnetic rotary encoder for wind direction)
- BMP390 (pressure and temperature sensor)
- SHT31 (temperature and humidity sensor)
- Hall Effect Sensor (wind speed, one pulse per cup rotation)
- Cellular SIM card with SMS enabled

## Pin Assignments
| Function | Pins |
|---|---|
| PMU I2C (Wire, bus 0) | SDA = GPIO15, SCL = GPIO7 |
| SHT31 I2C (Wire1, bus 1) | SDA = GPIO13, SCL = GPIO14 |
| SPI bus | SCK = GPIO18, MISO = GPIO16, MOSI = GPIO17 |
| AS5048A chip select | GPIO10 |
| BMP390 chip select | GPIO12 |
| Wind speed (Hall effect) | GPIO9 (active-low, falling edge) |
| Modem UART | RXD = GPIO4, TXD = GPIO5 |
| Modem PWRKEY | GPIO41 |

## Setup
1. SIM Card: Insert an active SIM with SMS service into the T-SIM7080G.
2. Station Altitude: Set `STATION_ALTITUDE_M` to your site elevation above mean sea level in metres. This is used for the altimeter setting.
3. Station Identifier: Change the station ID (currently `CYND`) in `buildMETAR()` to match your location.
4. Anemometer Geometry: Set `WIND_CUP_RADIUS_M` to match your cup anemometer so wind speed is scaled correctly.
5. Connect Sensors: Wire all sensors to the board as listed in the pin table above.

## Installation
1. Install the required libraries:
   - Adafruit BMP3XX
   - Adafruit SHT31
   - TinyGSM
   - XPowersLib
   - StreamDebugger (only needed if `DUMP_AT_COMMANDS` is enabled)
2. Select the ESP32-S3 board in your IDE and upload the sketch.
3. Open the Serial Monitor at 115200 baud to watch startup, network registration, and sensor updates.
4. Once the serial output shows `[READY]`, send a text message to the SIM's number.

## Code Overview
- The AS5048A 14-bit angle is converted to degrees and rounded to the nearest 10 degrees in the METAR wind group.
- The Hall effect sensor counts pulses in an interrupt. Once a minute these are converted to knots and stored in a 10-entry circular buffer.
- Reported wind speed is the 2-minute average. Gust is the peak of the 10-minute history and is only included when it exceeds the average by 5 knots or more.
- The BMP390 pressure is reduced to sea level using the station altitude and temperature, then converted to inHg for the `A` group.
- Temperature and dew point come from the SHT31 (dew point via the Magnus formula). Humidity is appended as an `RH=` remark.
- Sensors are read every 60 seconds. The latest snapshot is used to build the reply.
- Every 90 seconds a network watchdog cycles the modem (CFUN=0/1) so the network keeps PSM disabled, then reconfigures SMS and scans the SIM for texts that arrived during the brief offline period.
- A 60-second hardware watchdog reboots the ESP32 if the modem stops responding.
- Replies are capped at 155 characters to fit in a single SMS.

## Example Output

METAR CYND 00xxxxZ 27008KT 12/05 A3002 RH=55%

(Time is shown as `00HHMMZ`, derived from uptime. See Known Limitations.)

## Usage
After uploading and seeing `[READY]` in the serial output, text any message to the station's SIM number. The station replies within a few seconds with the current observation. Sensor values are also printed to the serial monitor every minute.

## Known Limitations
- The METAR timestamp is based on time since boot, not real UTC. Adding network time (for example via `modem.getNetworkTime` or NTP over the modem) would fix this.
- The station replies to any sender. Add a number allowlist if you want to restrict access.
- This is not a certified observation system and should not be used for flight planning decisions. 
