#!/bin/sh
# Builds the joint firmware into sim/build/<OUT>.elf.   JOINT_ID=1 ENC_REST_ANGLE=3.14159f SPIN=15 OUT=joint sh firmware/build.sh
# SPIN: the power-up show's top speed in rad/s (default 12, 0 = off). BENCH=1: console on the ST-LINK port, no calibration, timing prints.
# Needs PlatformIO. Set PLATFORMIO_CORE_DIR to keep the toolchain out of ~/.platformio.
# Two build-time fixes to the downloaded SimpleFOC libraries (they pass a timer handle where the Arduino-STM32 core's
# enableTimerClock wants the timer instance); the libraries are not otherwise changed.
set -e
cd "$(dirname "$0")/joint"
pio pkg install -e nucleo_g474re
sed -i.bak 's/enableTimerClock(handle);/enableTimerClock(handle->Instance);/' ".pio/libdeps/nucleo_g474re/Simple FOC/src/drivers/hardware_specific/stm32/stm32_mcu.cpp"
sed -i.bak 's/enableTimerClock(&encoder_handle)/enableTimerClock(encoder_handle.Instance)/' .pio/libdeps/nucleo_g474re/SimpleFOCDrivers/src/encoders/stm32hwencoder/STM32HWEncoder.cpp
PLATFORMIO_BUILD_FLAGS="-include stm32g4xx_hal.h -DWAIT_FOR_SERIAL -DCAL_SPIN_RAD_S=${SPIN:-12}${BENCH:+ -DBENCH}${BENCH16:+ -DBENCH -DBENCH16} -DJOINT_ID=${JOINT_ID:-1} -DENC_REST_ANGLE=${ENC_REST_ANGLE:-3.14159f}" pio run -e nucleo_g474re
mkdir -p ../../sim/build
cp .pio/build/nucleo_g474re/firmware.elf "../../sim/build/${OUT:-joint}.elf"
