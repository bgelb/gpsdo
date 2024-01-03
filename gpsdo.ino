#include <Adafruit_SI5351.h>
#include <Wire.h>

Adafruit_SI5351 clockgen = Adafruit_SI5351();
uint32_t cnt, last;

void IRAM_ATTR isr() {
  cnt++;
}

void setup() {
  delay(500);
  //Serial.begin(9600);
  delay(500);
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
  Serial.println("Si5351 Clockgen Test"); Serial.println("");

  Wire.begin(36, 35);
  if (clockgen.begin() != ERROR_NONE)
  {
    /* There was a problem detecting the IC ... check your connections */
    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("OK!");
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
