## Guntroller-v2 (ongoing)

Version 2 of my older Guntroller, now powered by STM32F411CEU6 instead of Arduino.

# Progress till now

Tuned endpoint polling interval, increasing polling rate from 100hz to 300-350hz.
Rewrote mpu6050 driver from blocking mode to DMA mode, further increasing polling rate, and 
finally achieving true stable 1000hz polling rate
