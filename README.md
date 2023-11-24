# Bluetooth Server Reebok 5.7e

This is an ESP32 based Bluetooth metrics server for a Reebok 5.7e indoor exercise bike based on [esp32-fmts-server](https://github.com/jamesjmtaylor/esp32-ftms-server) and [esp32-fmts-server-ic7](https://github.com/damndemento/esp32-ftms-server-ic7). Key changes to migrate from FMTS to CSC were taken from [Multi-BLE-Sensor](https://github.com/BigJinge/Multi-BLE-Sensor/tree/master). It does the following:

1. Measures the crank rotation rate and resistance from the exercise bike.
1. Converts the raw measurements into key metrics such as cadence, speed, distance and power.
1. Creates a Bluetooth server that can be connected to by a monitoring device.

The project targets the Apple Watch running watchOS 10 or later as a monitoring device.

## Apple Watch and iOS Workout Details

<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-bluetooth.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-cycling-metrics.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-cycling-power.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-watchos-cycling-powerzone.jpeg?raw=true" width="200" />

<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-summary.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-details.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-power.jpeg?raw=true" width="200" /> <img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/apple-ios-workout-split.jpeg?raw=true" width="200" />


## Bluetooth Service
The forked project uses the Bluetooth [Fitness Machine Service (FMTS)](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) server. Specifically it uses the [Indoor Bike Data characteristic](https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml) of the FTMS service. This service is currently not supported by my target of the Apple Watch running watchOS 10 as a monitoring device. There is a separate (untested) branch for this service in case it is supported by future versions of watchOS.

This branch focuses on the supported Bluetooth [Cycling Speed and Cadence (CSC) service](https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-service-1-0/) and the [Cycling Power Service](https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/).

## Reverse Engineering
By dismantling the bike and reverse engineering the power board and the main board the general working principle of the exercise bike was determined. Cadence is measured using a reed switch, which switches a GPIO of the main MCU to ground when triggered.
The electromagnet used for resistance is controlled by the power board. This modulates a rectified mains AC voltage supplied to the electromagnet using an PWM input. The PMW input is controlled from the main MCU GPIO via an optoisolator and opamp.
The optoisolator output is used as an input to the opamp resulting in synchronous PMW output using a low-voltage AC signal from the power board. The main MCU is at 5V VCC.

Power for the ESP32 is taken from the interface header.

The reed switch is tapped for the cadence trigger at the MOSFET to the GPIO on the main board.
A PNP transistor is used to level shift and invert the signal for digital read by the ESP32.

The electro-magnet PWM control signal is tapped for resistance at the MOSFET from the GPIO main board.
An NPN transistor is used to level shift and invert the signal for analogue read by the ESP32.

Simple edge triggering is used on the reed signal, with a de-bounce delay, to count crank revolutions and crank revolution times.
Wheel revolutions and wheel revolution times are calculated using a gear ratio and wheel size such that client displayed speed/distance matches the bikes display.

Power is calculate from duty-cycle (voltage) of the electro-magnet control signal. An empirical relationship for power as a function of cadence and duty-cycle was determined.
For fixed power and cadence the duty-cycle was measured. A simple quadratic function was sufficient to model power, with coefficients determined through non-linear fitting.

<img src="https://github.com/dbsqp/bluetooth-reebok-57e/blob/AppleWatch-branch/documentation/interface-final.jpeg?raw=true" />

## General Use
Although developed for my use case this code can be easily implemented for any exercise bike and other Bluetooth clients. The simplest implementation would be an external reed sensor connecting GND directly to ESP32 GPIO (or an external hall sensor) using CSC mode (no USEPOWER) or estimated power (USEALTPWR) mode with appropriate trigger logic (USEDIRECT).

Triggering using the internal hall effect sensor of the ESP32 is also implemented, but not test in-situ. Trigger threshold (HALLTRIG) and logic would need to be adjusted for specific magnet orientation and distance from sensor.


## Status
Feature complete, final release.

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
1. Finalise interface board [implement PWM circuit, insulation, robust physical mount]
1. Implement sleep timer [deep sleep after inactivity, wake on crank sensor]
1. Measure magnet duty-cycle at given cadence [80,100,120] and power [60, 100, 150, 190, 240, 280]
1. Finalise code to report correct cadence [notify every 2 seconds]
1. Report estimated power based on speed only [not very accurate]
1. Report duty-cycle as power to collect data for correlation [manual]
1. Added crank trigger via build in hall effect sensor [because its there]
1. Added provision for simple reed trigger [GND - REED - GPIO]
1. Reverse engineer power function: *P = f(D,C)* [implemented]
1. Decide to use measured, *P = f(D,C)*, or estimated, *P = f(S)*, power [use measured]
1. Fix bug relating to incorrect crank time after integer rollover [use lastTrigger to calculate lastCrankK]

### Todo
