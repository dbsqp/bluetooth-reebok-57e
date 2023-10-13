# FMTS Server Reebok 5.7e

This is an ESP32 based bluetooth metrics server for a Reebok 5.7e indoor exercise bike. It does the following:

1. Measures the flywheel rotation rate and resistance from the exercise bike.
2. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
3. Creates a [Bluetooth Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server than can be connected to by a monitoring device which support this protocol e.g. Apple Watch running watchOS 10 or later.

Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS BLE service.

## Roadmap
1. Setup toolchain
2. Test flash unedited code to ESP32
3. Test connect ESP32 to monitoring device
4. Update code for specific hardware identities
5. Update code to report synthetic dynamic key metrics
6. Investigate bike hardware
7. Identify sensor source for rotation
8. Identify sensor source for resistance
9. Access direct rotation sensor tap, bike-computer interface sniff or additional external sensor
10. Create prototype interface circuit between rotation sensor and microcontroller input
11. Update code to report key rotation based metrics
12. Access direct resistance sensor tap, bike-computer interface sniff or additional external sensor
13. Create prototype interface circuit between resistance sensor and microcontroller input
14. Update code to report key rotation and resistance based metrics
15. Assess MCU power source from exercise bike computer
16. Assess power on/off with bike computer
17. Implement standby/disconnect on idle (robust power down)
