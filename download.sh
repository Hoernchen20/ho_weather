#!/bin/bash
arm-none-eabi-objcopy -O ihex bin/bluepill/ho_weather.elf bin/bluepill/ho_weather.hex
~/Downloads/stm32flash/./stm32flash -w bin/bluepill/ho_weather.hex -v /dev/ttyUSB0 -b 576000
