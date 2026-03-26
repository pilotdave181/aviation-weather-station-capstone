#include <SPI.h>

// Your specific pin mapping
#define SPI_SCK  18
#define SPI_MISO 16
#define SPI_MOSI 17
#define SPI_CS   21

// AS5048A Read Angle Command (0xFFFF with parity)
const uint16_t READ_ANGLE = 0x3FFF; 

void setup() {
  Serial.begin(115200);
  
  // Initialize SPI with your custom pins
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  
  Serial.println("AS5048A SPI Test Initialized...");
}

void loop() {
  uint16_t rawData = readAddress(0x3FFF); // Read Angle register
  
  // The AS5048A returns 14 bits of data
  // We mask the top two bits (error and parity)
  uint16_t angle = rawData & 0x3FFF;
  
  float degrees = (angle * 360.0) / 16384.0;

  Serial.print("Raw: ");
  Serial.print(angle);
  Serial.print(" | Degrees: ");
  Serial.println(degrees);

  delay(100);
}

uint16_t readAddress(uint16_t addr) {
  // SPI settings: 1MHz, MSB first, Mode 1 (Clock Polarity 0, Phase 1)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  digitalWrite(SPI_CS, LOW);
  uint16_t result = SPI.transfer16(addr);
  digitalWrite(SPI_CS, HIGH);
  
  SPI.endTransaction();
  return result;
}