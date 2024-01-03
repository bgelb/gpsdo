#!/bin/sh
arduino-cli upload --fqbn esp32:esp32:esp32s3 --port /dev/ttyACM0 gpsdo.ino
