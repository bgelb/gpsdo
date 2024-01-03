#include <Wire.h>
#include <Adafruit_I2CDevice.h>

#define SI514_ADDR 0x55

#define SI514_REG_LP 0
#define SI514_REG_M_FRAC1 5
#define SI514_REG_M_FRAC2 6
#define SI514_REG_M_FRAC3 7
#define SI514_REG_M_INT_FRAC 8
#define SI514_REG_M_INT 9
#define SI514_REG_HS_DIV 10
#define SI514_REG_LS_HS_DIV 11
#define SI514_REG_OE_STATE 14
#define SI514_REG_RESET 128
#define SI514_REG_CONTROL 132

#define SI514_OUTPUT_ENABLE = 0x04
#define SI514_RESET = 0x80
#define SI514_CALIBRATE = 0x01

Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(SI514_ADDR);
uint32_t cnt, last;

void IRAM_ATTR isr() {
  cnt++;
}

void setup() {
  delay(500);
  Serial.begin(115200);
  delay(5000);
  Serial.println("\n\n================================");
  Serial.print("Chip Model: ");
  Serial.println(ESP.getChipModel());
  Serial.print("Chip version: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Numer of cores: ");
  Serial.println(ESP.getChipCores());
  Serial.print("Flash Chip Size: ");
  Serial.println(ESP.getFlashChipSize());
  Serial.print("Flash Chip Speed: ");
  Serial.println(ESP.getFlashChipSpeed());

  Serial.println("================================");

  Wire.begin(10, 11);
  Serial.println("I2C address detection test");

  if (!i2c_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
    while (1);
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);

  cnt=0;
  last=0;
  pinMode(1, INPUT);
  attachInterrupt(1, isr, RISING);
}

void loop() {
  if (cnt > last) {
    last=cnt;
    Serial.printf("saw PPS! cnt=%d\n", cnt);
  }
  delay(100);
}
