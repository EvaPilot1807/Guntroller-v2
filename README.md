# Guntroller-v2 

Version 2 of my older Guntroller, now powered by STM32F411CEU6 instead of Arduino.

## Whats new?

V2 is all about **speed** and **performance**. \
Thats why I optimizied and added a few things (and removed some).
Dynamic features (precision mode, caliberation reset) are still alive.

## New Features

* **Faster microcontroller:** Used the STM32F411CEU6 Black pill, running at **96MHz**, **twice** the speed of the V1.
* **Optimized using DMA:** Used Direct Memory Access (DMA) with I2C by using a custom modified the MPU6050 driver.
* **Digital low pass filter:** Configured the MPU6050's built in **DLPF** to **42Hz bandwidth** for minimal jitter during movements.
* **Changed endpoint variables of the CubeMX generated USB HID stack:** For faster report time.
* **Optimized I2C bus frequency:** Set I2C to Fast mode at 400KHz. This, combined with DMA, reduces the physical time required to extract the 14 bytes of raw sensor data from the MPU6050

### Achieved true stable 1000Hz polling rate due to above changes (>12x faster than V1, which had 60-70Hz stable)

## Other changes

* Added on board potentiometer for sensitivity control.
* Removed "B" and "esc" key (which was a feature in v1) to avoid clashing of USB HID reports and decreasing polling rate.


## Important advice if you plan to build and use it

This device is a high-resolution input peripheral (like a high-DPI gaming mouse). To prevent the cursor from jumping and moving too fast, you should lower the pointer sensitivy on your device. This allows the system to process the high-resolution input smoothly and not ignore micro-movements.
