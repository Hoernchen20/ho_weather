#!/bin/bash
~/Downloads/stm32flash/./stm32flash /dev/ttyUSB0 -j
~/Downloads/stm32flash/./stm32flash /dev/ttyUSB0 -k
~/Downloads/stm32flash/./stm32flash /dev/ttyUSB0 -e 0 -w bin/nucleo-l073rz/ho_weather.bin