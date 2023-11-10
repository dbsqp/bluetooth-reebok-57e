# Bluetooth Server Reebok 5.7e

This is an ESP32 based bluetooth metrics server for a Reebok 5.7e indoor exercise bike based on [esp32-fmts-server](https://github.com/jamesjmtaylor/esp32-ftms-server) and [esp32-fmts-server-ic7](https://github.com/damndemento/esp32-ftms-server-ic7). Key changes to migrage from FMTS to CSC were taken from [Multi-BLE-Sensor](https://github.com/BigJinge/Multi-BLE-Sensor/tree/master). It does the following:

1. Measures the flywheel rotation rate and resistance from the exercise bike.
1. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
1. Creates a Bluetooth server than can be connected to by a monitoring device.

The project targets the Apple Watch running watchOS 10 or later as a monitoring device.

## Apple Watch and iOS Workout Details

<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-bluetooth.jpeg?raw=true" width="200" />
<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-cycling-metrics.jpeg?raw=true" width="200" />
<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-cycling-power.jpeg?raw=true" width="200" />

<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-details.jpeg?raw=true" width="200" />
<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-power.jpeg?raw=true" width="200" />

## Bluetooth Service
The forked project uses the Bluetooth [Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server. Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS service. This service is currently not supported by my target of the Apple Watch running watchOS 10 as a monitoring device. There is a separate (untested) branch for this service incase it is supported by future versions of watchOS.

This branche focuses on the supported Bluetooth [Cycling Speed and Cadence (CSC) service](https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-service-1-0/) and the [Cycling Power Service](https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/).

## General Use
Although developed for my use case this code can be esily implemented for any exercise bike and bluetooth client. The simplist implementation would be an external reed or hall sensor connected directly to the esp32 using approximate power mode.

## Status
Working, under developement.

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
1. Investigate bike hardware [needed crank puller]
1. Identify sensor source for rotation [reed not Hall sensor]
1. Assess MCU power source from exercise bike computer [5V from programming header GND/VDD]
1. Access direct rotation sensor [5V pulled to GND tap reed switch (RPM) contact on main PCB]
1. Identify sensor source for resistance [main PCB - optoisolator - power circuit - power board - rectified AC - triac]
1. Update code to read PWM [analogueRead]
1. Create prototype interface circuit from reed switch to ESP32 [TIP125 PNP common GND, base to GPIO]
1. Assess power on/off with bike computer [powers up on wake, powers down on sleep]
1. Access direct resistance sensor [5V 1 kHz PMW tap at MOSFET to optocoupler on main PCB]
1. Create interface board [proto board, socketed ESP32, power/sensor in headers, scope test points, strain relief, VCC in switch for USB power/debug]
1. Fix broken resistance function [burnt out resistor to optocoupler out]
1. Assess power on/off with bike computer [programming header always powered]
1. Create prototype interface circuit from PMW signal to ESP32 [TIP120 NPN, base to PWM]
1. Calculate primary resistance metric as PWM duty-cycle (D) [100% = high resistance, 0% = no resistance]
1. finalise interface board [implement PWM circuit, insulation, robust physical mount]
1. Implement sleep timer [deep sleep after inactivity, wake on crank sensor]
1. Measure magnet duty-cycle at given cadance [80,100,120] and power [100,160, 240]
1. Finalise code to report correct cadance [notify every 2 seconds]
1. Report approx power based on speed and terminal velocity
1. Report duty-cycle as power to collect bulk correlation data [download CSV for workout from iOS Health]
1. Added crank trigger via build in hall effect sensor

### Todo
1. Reverse engineer power function: P = f(C,D)
1. Decide to use f(C,D) or approximation for power
