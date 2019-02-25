### Program

1. Download NRF52 SDK 15.2 [link](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs)
2. Clone repo
3. Fix path to sdk in Makefile:

``SDK_ROOT := ..``

``PROJ_DIR := .``

4. Flash Softdevice
`` make flash_softdevice ``
5. Flash App
`` make flash ``

### Use

Sensor Advertise current values in manufacturer data field

#### Data format

| 0x0 8bit |  0x00 16bit | 0x00 16bit |
|-------------------------|-----------------------------|--------------------|
| "Battery capacity" |  "Temperature x 100" | 2x byte "PH x 100" |
| 100 | 3220 | 721 |
| 100%| 32.20 C| 7.21 |

#### Calibration

1. Put probe in 4.01 buffer
2. Wait 1-2 min
3. Connect to service f364adc9-b000-4042-ba50-05ca45bf8abc
4. Write to characteristic 0x1401 0x01 value
5. Wait until rgb led start blink green

6. Put probe in clean water, for clean from previous buffer solution

7. Put probe in 6.86 buffer
8. Wait 1-2 min
9. Connect to service f364adc9-b000-4042-ba50-05ca45bf8abc
10. Write to characteristic 0x1401 0x02 value
11. Wait until rgb led start blink green

12. Repeat 6

13. Put probe in 9.18 buffer
14. Wait 1-2 min
15. Connect to service f364adc9-b000-4042-ba50-05ca45bf8abc
16. Write to characteristic 0x1401 0x03 value
17. Wait until rgb led start blink green

If use another buffer edit in ph_sensor.c line 233, 238
