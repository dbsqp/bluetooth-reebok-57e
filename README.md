# FMTS Server Reebok 5.7e

This is an ESP32 based bluetooth metrics server for a Reebok 5.7e indoor exercise bike based on [esp32-fmts-server](https://github.com/jamesjmtaylor/esp32-ftms-server) and [esp32-fmts-server-ic7](https://github.com/damndemento/esp32-ftms-server-ic7). It does the following:

1. Measures the flywheel rotation rate and resistance from the exercise bike.
1. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
1. Creates a [Bluetooth Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server than can be connected to by a monitoring device.

Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS BLE service and targets the Apple Watch running watch of 10 or later as a monitoring device.

## Roadmap
### Done
1. Setup toolchain [Arduino IDE v2.2.1 + esp32 (Espressif Systems) v2.0.11 / NodeMCU-32S]
1. Update code header
1. Compile unedited code
1. Update readme

### Todo
1. Test flash unedited code to ESP32
1. Test connect ESP32 to monitoring device
1. Update code for specific hardware identities
1. Update code to report synthetic dynamic key metrics
1. Investigate bike hardware
1. Identify sensor source for rotation
1. Identify sensor source for resistance
1. Access direct rotation sensor tap, bike-computer interface sniff or additional external sensor
1. Create prototype interface circuit between rotation sensor and microcontroller input
1. Update code to report key rotation based metrics
1. Access direct resistance sensor tap, bike-computer interface sniff or additional external sensor
1. Create prototype interface circuit between resistance sensor and microcontroller input
1. Update code to report key rotation and resistance based metrics
1. Assess MCU power source from exercise bike computer
1. Assess power on/off with bike computer
1. Implement standby/disconnect on idle (robust power down)
