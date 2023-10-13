# FMTS Server Reebok 5.7e

This is an ESP32 based bluetooth metrics server for a Reebok 5.7e indoor exercise bike. It does the following:

1. Measures the flywheel rotation rate and resistance from the exercise bike.
2. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
3. Creates a [Bluetooth Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server than can be connected to by a monitoring device which support this protocol e.g. Apple Watch running watchOS 10 or later.

Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS BLE service.

## Roadmap
### Done
1. Setup toolchain [Arduino IDE v2.2.1 + esp32 (Espressif Systems) v2.0.11 / NodeMCU-32S]
2. Update code header
3. Compile unedited code

### Todo
3. Test flash unedited code to ESP32
4. Test connect ESP32 to monitoring device
5. Update code for specific hardware identities
6. Update code to report synthetic dynamic key metrics
7. Investigate bike hardware
8. Identify sensor source for rotation
9. Identify sensor source for resistance
10. Access direct rotation sensor tap, bike-computer interface sniff or additional external sensor
11. Create prototype interface circuit between rotation sensor and microcontroller input
12. Update code to report key rotation based metrics
13. Access direct resistance sensor tap, bike-computer interface sniff or additional external sensor
14. Create prototype interface circuit between resistance sensor and microcontroller input
15. Update code to report key rotation and resistance based metrics
16. Assess MCU power source from exercise bike computer
17. Assess power on/off with bike computer
18. Implement standby/disconnect on idle (robust power down)
