## Use

Sensor Advertise current values in manufacturer data field little endian 5 byte

|         | 0x0 (8bit)         |  0x01 - 0x02 (16bit) | 0x03 - 0x04 (16bit) | 0x04 - 0x05 (16bit)   |
|---------|--------------------|----------------------|---------------------|-----------------------|
| Name    | "Battery capacity" |  "Temperature x 100" | 2x byte "PH x 100"  | 2x byte "PH probe RAW"|
| example | 92                 | 3220                 | 721                 | 141                   |
| Real    | 92%                | 32.20                | 7.21                | 141 mv                |

or connect and read ble characteristics

 - NTC or MCU temp sensor: f3641402-00b0-4042-ba50-05ca45bf8abc
 - PH probe: f3641407-00b0-4042-ba50-05ca45bf8abc
 - PH probe RAW: f3641408-00b0-4042-ba50-05ca45bf8abc

#### Button Calibration
long press > 2s -> start calibrating in low PH (4.01) mode
short press change mode (4.01 | 6.86 | 11)

#### Calibration

1. Put probe in 4.01 buffer
2. Wait 1-2 min
3. Connect to service f3641400-00b0-4042-ba50-05ca45bf8abc
4. Write to characteristic 0x1401 0x01 value
5. Wait until rgb led start blink green
6. Put probe in clean water, for clean from previous buffer solution
7. Put probe in 6.86 buffer
8. Wait 1-2 min
9. Connect to service f3641400-00b0-4042-ba50-05ca45bf8abc
10. Write to characteristic 0x1401 0x02 value
11. Wait until rgb led start blink green
12. Repeat step 6
13. Put probe in 9.18 buffer
14. Wait 1-2 min
15. Connect to service f364adc9-b000-4042-ba50-05ca45bf8abc
16. Write to characteristic 0x1401 0x03 value
17. Wait until rgb led start blink green

If use another buffer edit in ph_sensor.c line 233, 238

## Program J-link
1. Download [NRF52 SDK 16](https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs)
2. Download [ARM GCC](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) 
3. Clone repo ``git clone https://github.com/9zigen/NRF52_PhSensor``
4. Fix path to sdk and gcc in Makefile: ``SDK_ROOT := $PATH_TO_SDK``
5. Flash Softdevice `` make flash_softdevice ``
6. Flash App `` make flash `` or `` make pkg`` for generating DFU package (generate keys first and flash bootloader with these keys)

## Ble DFU
1. Install nrfutil ``pip install nrfutil``
2. Generate keys ``nrfutil keys display --key pk --format code DFU/private.key --out_file DFU/public_key.c``
3. Flash bootloader ``cd DFU/bootloader && make flash``
4. Generate dfu package ``nrfutil pkg generate --hw-version 52 --sd-req 0 --application-version 0x0 --application _build/nrf52832_xxaa.hex --key-file DFU/private.key DFU/dfu.zip``
5. Use the Nordic nRF Connect app to upgrade firmware
