#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include "driver/pcnt.h"
#include "soc/pcnt_struct.h"

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

#define SI514_OUTPUT_ENABLE 0x04
#define SI514_RESET 0x80
#define SI514_CALIBRATE 0x01

#define TICK_ROLLOVER_LIMIT 10000

Adafruit_I2CDevice si514_dev = Adafruit_I2CDevice(SI514_ADDR);
uint32_t cnt, last;
int16_t tick;
uint32_t ext_tick;
uint32_t new_full_tick, last_full_tick;

void IRAM_ATTR pcnt_isr(void *arg) {
    uint32_t intr_status = PCNT.int_st.val;
    if(intr_status & 0x1) {
      ext_tick+=TICK_ROLLOVER_LIMIT;
    }
    PCNT.int_clr.val = intr_status;
}

void IRAM_ATTR isr() {
  pcnt_get_counter_value(PCNT_UNIT_0, &tick);
  cnt++;
  last_full_tick = new_full_tick;
  new_full_tick = tick + ext_tick;
}

void byte_write(Adafruit_I2CDevice &dev, uint8_t addr, uint8_t value) {
  Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(&si514_dev, addr);
  reg.write(value);
}

void calibrate_vcxo(Adafruit_I2CDevice &dev) {
  Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(&si514_dev, SI514_REG_CONTROL);
  uint8_t val;
  reg.read(&val);
  val |= SI514_CALIBRATE;
  reg.write(val);
}

void pcnt_init_channel (
  pcnt_unit_t pcnt_unit,
  int sig_io,
  int ctrl_io = PCNT_PIN_NOT_USED,
  pcnt_channel_t pcnt_channel = PCNT_CHANNEL_0,
  int h_lim_val = 10,
  int l_lim_val = -10,
  int thresh1_val = 10000,
  int thresh0_val = -10000
) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config;
        // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = sig_io;
        pcnt_config.ctrl_gpio_num = ctrl_io;
        pcnt_config.channel = pcnt_channel;
        pcnt_config.unit = pcnt_unit;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;
        pcnt_config.neg_mode = PCNT_COUNT_DIS;
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = h_lim_val;
        pcnt_config.counter_l_lim = l_lim_val;

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /* Configure and enable the input filter */
    //pcnt_set_filter_value(PCNT_UNIT, 100);
    //pcnt_filter_enable(PCNT_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    //pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    //pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(pcnt_unit, PCNT_EVT_H_LIM);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_isr, NULL, 0, NULL);
    pcnt_intr_enable(pcnt_unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(pcnt_unit);
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

  // Set up Si514
  Wire.begin(10, 11);
  Serial.println("I2C address detection test");

  if (!si514_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(si514_dev.address(), HEX);
    while (1);
  }
  Serial.print("Device found on address 0x");
  Serial.println(si514_dev.address(), HEX);

  // disable output
  byte_write(si514_dev, SI514_REG_CONTROL, 0x0);

  // 10 MHz
  byte_write(si514_dev, SI514_REG_LP, 0x22);
  byte_write(si514_dev, SI514_REG_M_FRAC1, 0x15);
  byte_write(si514_dev, SI514_REG_M_FRAC2, 0x2);
  byte_write(si514_dev, SI514_REG_M_FRAC3, 0x4d);
  byte_write(si514_dev, SI514_REG_M_INT_FRAC, 0x20);
  byte_write(si514_dev, SI514_REG_M_INT, 0x8);
  byte_write(si514_dev, SI514_REG_HS_DIV, 0xd0);
  byte_write(si514_dev, SI514_REG_LS_HS_DIV, 0x0);

  // enable
  calibrate_vcxo(si514_dev);
  byte_write(si514_dev, SI514_REG_CONTROL, SI514_OUTPUT_ENABLE);

  // set up PPS
  cnt=0;
  last=0;
  tick=0;
  ext_tick=0;
  last_full_tick=0;
  new_full_tick=0;
  pinMode(1, INPUT);
  attachInterrupt(1, isr, RISING);
  pcnt_init_channel (PCNT_UNIT_0, 14, PCNT_PIN_NOT_USED, PCNT_CHANNEL_0, TICK_ROLLOVER_LIMIT);
}

void loop() {
  if (cnt > last) {
    last=cnt;
    Serial.printf("saw PPS! cnt=%d tick=%d ext_tick=%d last_full_tick=%d new_full_tick=%d dt=%d\n", cnt, tick, ext_tick, last_full_tick, new_full_tick, new_full_tick-last_full_tick);
  }
  delay(100);
}
