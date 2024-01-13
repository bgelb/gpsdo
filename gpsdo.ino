#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
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

#define GPS_SERIAL Serial1

Adafruit_I2CDevice si514_dev = Adafruit_I2CDevice(SI514_ADDR);
SFE_UBLOX_GNSS gps;

uint32_t cnt, last;
int16_t tick;
uint32_t ext_tick;
uint32_t new_full_tick, last_full_tick;

enum cal_state_t {
  S_IDLE,
  S_COUNT,
  S_ADJUST
};

cal_state_t cal_state;
uint32_t cal_interval;
uint32_t cal_pps_count;
uint64_t cal_clock_count;

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

uint8_t byte_read(Adafruit_I2CDevice &dev, uint8_t addr) {
  uint8_t value;
  Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(&si514_dev, addr);
  reg.read(&value);
  return value;
}

void small_freq_change(Adafruit_I2CDevice &si514_dev, int32_t ppb) {

  uint8_t reg05, reg06, reg07, reg08, reg09;
  uint32_t int_m, frac_m;
  uint32_t dm;

  reg05 = byte_read(si514_dev, SI514_REG_M_FRAC1);
  reg06 = byte_read(si514_dev, SI514_REG_M_FRAC2);
  reg07 = byte_read(si514_dev, SI514_REG_M_FRAC3);
  reg08 = byte_read(si514_dev, SI514_REG_M_INT_FRAC);
  reg09 = byte_read(si514_dev, SI514_REG_M_INT);

  frac_m = reg05;
  frac_m += reg06 << 8;
  frac_m += reg07 << 16;
  frac_m += (reg08 & 0x1f)  << 24;

  int_m = reg08 >> 5;
  int_m += reg09 << 3;

  //Serial.printf("int_m: %d frac_m: %d\n", int_m, frac_m);
  //Serial.printf("frac_m: %x\n", frac_m);
  //Serial.printf("%x %x %x %x %x\n", reg09, reg08, reg07, reg06, reg05);

  dm = ((int64_t)frac_m * (int64_t)ppb / (int64_t)1000000000) + (((int64_t)int_m << 29) * (int64_t)ppb / (int64_t)1000000000);

  //Serial.printf("dm: %d\n", dm);
 
  // TODO: this is sketchy
  frac_m = (int32_t)frac_m + dm;
  //int_m = (int32_t)int_m + (dm >> 29);

  reg05 = frac_m & 0xff;
  reg06 = (frac_m >> 8) & 0xff;
  reg07 = (frac_m >> 16) & 0xff;
  reg08 = (frac_m >> 24) & 0x1f | ((int_m & 0x07) << 5);
  reg09 = (int_m >> 3);

  //Serial.printf("int_m: %d frac_m: %d\n", int_m, frac_m);
  //Serial.printf("%x\n", frac_m);
  //Serial.printf("%x %x %x %x %x\n", reg09, reg08, reg07, reg06, reg05);

  byte_write(si514_dev, SI514_REG_M_FRAC1, reg05);
  byte_write(si514_dev, SI514_REG_M_FRAC2, reg06);
  byte_write(si514_dev, SI514_REG_M_FRAC3, reg07);
  byte_write(si514_dev, SI514_REG_M_INT_FRAC, reg08);
  byte_write(si514_dev, SI514_REG_M_INT, reg09);

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
  byte_write(si514_dev, SI514_REG_M_INT_FRAC, 0x21);
  byte_write(si514_dev, SI514_REG_M_INT, 0x8);
  byte_write(si514_dev, SI514_REG_HS_DIV, 0xd0);
  byte_write(si514_dev, SI514_REG_LS_HS_DIV, 0x0);

  // enable
  calibrate_vcxo(si514_dev);
  byte_write(si514_dev, SI514_REG_CONTROL, SI514_OUTPUT_ENABLE);

  // set up GPS comms
  GPS_SERIAL.begin(9600);
  while (gps.begin(GPS_SERIAL) == false) //Connect to the u-blox module using mySerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }
  gps.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
  gps.setI2COutput(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)

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

  // init state machine
  cal_interval = 10;
  cal_pps_count = 0;
  cal_state = S_IDLE;
}

void loop() {
  int32_t ppb, qerr;
  if (cnt > last) {
    last=cnt;
    //Serial.printf("saw PPS! cnt=%d tick=%d ext_tick=%d last_full_tick=%d new_full_tick=%d dt=%d\n", cnt, tick, ext_tick, last_full_tick, new_full_tick, new_full_tick-last_full_tick);

    if (gps.getTIMTP() == true)
    {
      qerr = gps.getTIMTPqErr();
    }
    // State machine
    if(cal_state == S_IDLE) {
      Serial.printf("saw PPS! cnt=%d tick=%d ext_tick=%d last_full_tick=%d new_full_tick=%d dt=%d qerr=%d\n", cnt, tick, ext_tick, last_full_tick, new_full_tick, new_full_tick-last_full_tick, qerr);
      cal_pps_count += 1;
      if (cal_pps_count > 10) {
        Serial.printf("Running cal cycle interval = %d sec.", cal_interval);
        cal_state = S_COUNT;
        cal_pps_count = 0;
        cal_clock_count = 0;
      }
    } else if(cal_state == S_COUNT) {
      cal_pps_count += 1;
      cal_clock_count += (new_full_tick-last_full_tick);
      Serial.printf(".");
      if(cal_pps_count == cal_interval) {
        Serial.printf("\n");
        cal_state = S_ADJUST;
      }
    } else if(cal_state == S_ADJUST) {
      ppb = (int32_t)(((int64_t)10000000 * (int64_t)cal_interval - (int64_t)cal_clock_count) * (int64_t)100 / (int64_t)cal_interval);
      Serial.printf("Finished cal cycle interval = %d sec. Error = %d ppb.\n", cal_interval, ppb);
      Serial.printf("ppb: %x\n", ppb);
      small_freq_change(si514_dev, ppb);
      cal_state = S_IDLE;
      cal_pps_count = 0;
      if(cal_interval < 1000) {
        cal_interval *= 10;
      }
    } else {
      // should be unreachable
      Serial.println("State machine reached invalid state, resetting!");
      delay(1000);
      cal_state = S_IDLE;
      cal_pps_count = 0;
    }
  }
  //if (gps.getPVT() == true)
  //{
  //  int32_t latitude = gps.getLatitude();
  //  Serial.print(F("Lat: "));
  //  Serial.print(latitude);

  //  int32_t longitude = gps.getLongitude();
  //  Serial.print(F(" Long: "));
  //  Serial.print(longitude);
  //  Serial.print(F(" (degrees * 10^-7)"));

  //  int32_t altitude = gps.getAltitudeMSL(); // Altitude above Mean Sea Level
  //  Serial.print(F(" Alt: "));
  //  Serial.print(altitude);
  //  Serial.print(F(" (mm)"));

  //  Serial.println();
  //}
  //if (gps.getTIMTP() == true)
  //{
  //  uint32_t tow_ms = gps.getTIMTPtowMS();
  //  Serial.print(F("towMS: "));
  //  Serial.print(tow_ms);
  //  uint32_t tow_sub_ms = gps.getTIMTPtowSubMS();
  //  Serial.print(F(" towSubMS: "));
  //  Serial.print(tow_sub_ms);
  //  int32_t qerr = gps.getTIMTPqErr();
  //  Serial.print(F(" qErr: "));
  //  Serial.print(qerr);
  //  Serial.println();
  //}
  delay(1);
}
