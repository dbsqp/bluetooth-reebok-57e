# Bluetooth Server Reebok 5.7e

This is an ESP32 based bluetooth metrics server for a Reebok 5.7e indoor exercise bike based on [esp32-fmts-server](https://github.com/jamesjmtaylor/esp32-ftms-server) and [esp32-fmts-server-ic7](https://github.com/damndemento/esp32-ftms-server-ic7). Key changes to migrage from FMTS to CSC were taken from [Multi-BLE-Sensor](https://github.com/BigJinge/Multi-BLE-Sensor/tree/master). It does the following:

1. Measures the flywheel rotation rate and resistance from the exercise bike.
1. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
1. Creates a Bluetooth server than can be connected to by a monitoring device.

The project targets the Apple Watch running watchOS 10 or later as a monitoring device.

## Bluetooth Service
The forked project uses the Bluetooth [Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server. Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS service. This service is currently not supported by my target of the Apple Watch running watchOS 10 as a monitoring device. There is a separate (untested) branch for this service incase it is supported by future versions of watchOS.

This branche focuses on the supported Bluetooth [Cycling Speed and Cadence (CSC) service](https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-service-1-0/) and the [Cycling Power Service](https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/).

## Status
Unfinished, under developement.

## Roadmap
### Done
1. Setup toolchain [Arduino IDE v2.2.1 + esp32 (Espressif Systems) v2.0.11 / NodeMCU-32S]
1. Update code header
1. Compile unedited code
1. Update readme
1. Test flash unedited code to ESP32
1. Update code for specific hardware identities [Reebok 5.7e]
1. Code for Bluetooth Cycling Speed and Cadence (CSC) service cf. FTMS
1. Test connect ESP32 to monitoring device
1. Update code to report simulated dynamic metrics [CSC]
1. Code for Bluetooth Cycling Power Service [CPS]

### Todo
1. Investigate bike hardware
1. Identify sensor source for rotation
1. Identify sensor source for resistance
1. Access direct rotation sensor tap, bike-computer interface sniff or additional external sensor
1. Create prototype interface circuit between rotation sensor and microcontroller input
1. Update code to report valid rotation metrics
1. Access direct resistance sensor tap, bike-computer interface sniff or additional external sensor
1. Create prototype interface circuit between resistance sensor and microcontroller input
1. Update code to report valid rotation and resistance metrics
1. Assess MCU power source from exercise bike computer
1. Assess power on/off with bike computer
1. Implement standby/disconnect on idle (robust power down)
