/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    based on esp32-ftms-server by jamesjmtaylor
    updated for Reebok 5.7e indoor exercise bike by dbsqp
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>



// options
//#define DEBUG
//#define CSC_MODE
//#define SIMULATION

#define PIN_REV 13  // Cadance reed-switch (trigger pulls low, sense via PNP)
#define PIN_MAG 14  // Resistance Magnet (5V 1kHz PWM, sense via NPN)
#define PIN_LED 32  // NOTE GPIO 2 onboard LED = ESP32 UART activity
#define SERVER_NAME "Reebok 5.7e Bike"



// globals
BLEServer *pServer;

// https://www.bluetooth.org/en-us/specification/assigned-numbers-overview
#if defined(CSC_MODE)
#define SERVICE_UUID BLEUUID((uint16_t)0x1816)
BLECharacteristic featureCharacteristics(BLEUUID((uint16_t)0x2A5C), BLECharacteristic::PROPERTY_READ);
BLECharacteristic measurementCharacteristics(BLEUUID((uint16_t)0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic scControlPointCharacteristics(BLEUUID((uint16_t)0x2A55), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);
BLEDescriptor scControlPointDescriptor(BLEUUID((uint16_t)0x2901));
#else
#define SERVICE_UUID BLEUUID((uint16_t)0x1818)
BLECharacteristic featureCharacteristics(BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic measurementCharacteristics(BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
#endif

bool revStateOld;

unsigned long elapsedTime;
unsigned long elapsedSampleTime;
unsigned long rev;  // rev trigger
unsigned long mag;  // mag PWM

uint16_t crankrev;   // Cadence RPM
uint16_t lastcrank;  // Last crank time
uint32_t wheelrev;   // Wheel revolutions
uint16_t lastwheel;  // Last crank time
uint16_t power;      // power [Watts]

uint16_t cadence;

bool deviceConnected = false;
bool oldDeviceConnected = false;



// connection status
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};



// main setup
void setup() {
  Serial.begin(115200);
  Serial.println("Startup");

  setupLED();
  setupBluetoothServer();
  setupRevSensor();
  setupMagSensor();

  elapsedTime = 0;
  elapsedSampleTime = 0;

  rev = 0;
  mag = 0;
  crankrev = 0;
  lastcrank = 0;
  wheelrev = 0;
  lastwheel = 0;
  power = 0;
}



// setup LED
void setupLED() {
  pinMode(PIN_LED, OUTPUT);
}



// setup BLE
void setupBluetoothServer() {

  // create BLE device
  BLEDevice::init(SERVER_NAME);
  Serial.print("Server : " SERVER_NAME);

  // create BLE server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


#if defined(CSC_MODE)
  Serial.print(" - Cadence [BLE/CSC]");
#else
  Serial.print(" - Power [BLE/CP]");
#endif

#if defined(SIMULATION)
  Serial.println(" - Simulation!");
#else
  Serial.println("");
#endif

  // create service & add measurement/features
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pService->addCharacteristic(&measurementCharacteristics);
  pService->addCharacteristic(&featureCharacteristics);

  // create descriptors for measurements
  BLEDescriptor measurementDescriptor(BLEUUID((uint16_t)0x2901));  //0x2901 is a custom user description
  measurementDescriptor.setValue("Exercise Bike Measurement");
  measurementCharacteristics.addDescriptor(&measurementDescriptor);
  measurementCharacteristics.addDescriptor(new BLE2902());

  // create descriptors for features
  BLEDescriptor featureDescriptor(BLEUUID((uint16_t)0x2901));
  featureDescriptor.setValue("Exercise Bike Feature");
  featureCharacteristics.addDescriptor(&featureDescriptor);

  // start service
  pService->start();
  Serial.print("Server : Started");

  // start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // help with iPhone connection
  pAdvertising->setMinPreferred(0x12);

  pServer->getAdvertising()->start();
  Serial.println(" & Advertising...");
}



// setup cadance sensor
void setupRevSensor() {
  pinMode(PIN_REV, INPUT);
  revStateOld = digitalRead(PIN_REV);
}



// setup resistance sensor
void setupMagSensor() {
  pinMode(PIN_MAG, INPUT);
}






// rising edge trigger function
inline bool risingEdge(bool &oldState, bool state) {
  bool result = (!oldState && state);  // !0 && 1

#if defined(DEBUG)
  if (result) {
    Serial.printf("edge: / %d%d = %d trigger\n", oldState, state, result);
  } else {
    Serial.printf("edge:  \\ %d%d = %d\n", oldState, state, result);
  }
#endif

  oldState = state;
  return result;
}



// falling edge trigger function
inline bool fallingEdge(bool &oldState, bool state) {
  bool result = (oldState && !state);  // 1 && !0

#if defined(DEBUG)
  if (!result) {
    Serial.printf("edge: / %d%d = %d\n", oldState, state, result);
  } else {
    Serial.printf("edge: \\ %d%d = %d trigger\n", oldState, state, result);
  }
#endif

  oldState = state;
  return result;
}



// calculations
double calculateKphFromRpm(double rpm) {
  double WHEEL_RADIUS = 0.00034;  // in km
  double KM_TO_MI = 0.621371;

  double circumfrence = 2 * PI * WHEEL_RADIUS;
  double metricDistance = rpm * circumfrence;
  double kph = metricDistance * 60;
  // Serial.printf("rpm: %2.2f, circumfrence: %2.2f, distance %2.5f , speed: %2.2f \n", rpm, circumfrence, metricDistance, kmh);
  return kph;
}

unsigned long caloriesTime = 0;
double calculatePowerFromKph(double kph) {
  double velocity = kph * 0.2777;  // m/s
  double riderWeight = 72.0;       // kg
  double bikeWeight = 12.0;        // kg
  double rollingRes = 0.004;
  double frontalArea = 0.445;  // Bartops
  double grade = 0;            // °
  double headwind = 0;         // m/s
  double temperature = 15.0;   // °C
  double elevation = 100;      // m
  double transv = 0.95;        // unknown

  double density = (1.293 - 0.00426 * temperature) * exp(-elevation / 7000.0);
  double twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
  double A2 = 0.5 * frontalArea * density;        // full air resistance parameter
  double tres = twt * (grade + rollingRes);       // gravity and rolling resistance

  // we calculate power from velocity
  double tv = velocity + headwind;       // terminal velocity
  double A2Eff = (tv > 0.0) ? A2 : -A2;  // reverse effect wind in face
  return (velocity * tres + velocity * tv * tv * A2Eff) / transv;
}



// notify CSC
void serviceNotifyCSC(int wheelrev, int lastwheel, int crankrev, int lastcrank) {

  // CSC 16bit/0-15 - 2/2:multi-location 1/1:crankRev 0/0:wheelRev
  byte feature[1] = { 0b0000000000000011 };
  byte measurement[11] = { 0b00000011, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // set measurement
  measurement[1] = wheelrev & 0xFF;
  measurement[2] = (wheelrev >> 8) & 0xFF;
  measurement[3] = (wheelrev >> 16) & 0xFF;
  measurement[4] = (wheelrev >> 24) & 0xFF;

  measurement[5] = lastwheel & 0xFF;
  measurement[6] = (lastwheel >> 8) & 0xFF;

  measurement[7] = crankrev & 0xFF;
  measurement[8] = (crankrev >> 8) & 0xFF;

  measurement[9] = lastcrank & 0xFF;
  measurement[10] = (lastcrank >> 8) & 0xFF;

  // monitor state
  bool disconnecting = !deviceConnected && oldDeviceConnected;
  bool connecting = deviceConnected && !oldDeviceConnected;

  // notify with measurements if connected
  if (deviceConnected) {
    measurementCharacteristics.setValue(measurement, 11);
    measurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("\nClient: disconnected");
    Serial.print("Server: Advertising...");

    oldDeviceConnected = deviceConnected;
  }

  // feature notify if connecting
  if (connecting) {
    oldDeviceConnected = deviceConnected;
    Serial.println("\nClient: connected");
    featureCharacteristics.setValue(feature, 1);
    Serial.print("Server: set features");
  }
}



// notify CP
void serviceNotifyCP(int power, int wheelrev, int lastwheel, int crankrev, int lastcrank) {

  // CP 32 bit/0-31 - 4/5:crankRev 3/4:wheelRev
  byte feature[1] = { 0b00000000000000000000000000001100 };
  byte measurement[16] = { 0b0000000000110000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // set measurement
  measurement[2] = power & 0xFF;
  measurement[3] = (power >> 8) & 0xFF;

  measurement[4] = wheelrev & 0xFF;
  measurement[5] = (wheelrev >> 8) & 0xFF;
  measurement[6] = (wheelrev >> 16) & 0xFF;
  measurement[7] = (wheelrev >> 24) & 0xFF;

  measurement[8] = lastwheel & 0xFF;
  measurement[9] = (lastwheel >> 8) & 0xFF;

  measurement[10] = crankrev & 0xFF;
  measurement[11] = (crankrev >> 8) & 0xFF;

  measurement[12] = lastcrank & 0xFF;
  measurement[13] = (lastcrank >> 8) & 0xFF;

  // connection state
  bool disconnecting = !deviceConnected && oldDeviceConnected;
  bool connecting = deviceConnected && !oldDeviceConnected;

  // notify with measurements if connected
  if (deviceConnected) {
    measurementCharacteristics.setValue(measurement, 16);
    measurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("\nClient: disconnected");
    Serial.print("Server: Advertising...");
    oldDeviceConnected = deviceConnected;
  }

  // feature notify if connecting
  if (connecting) {
    Serial.println("\nClient: connected");
    oldDeviceConnected = deviceConnected;
    featureCharacteristics.setValue(feature, 1);
    Serial.print("Server: set features");
  }
}






// main loop
void loop() {
  unsigned long intervalTime = millis() - elapsedTime;
  unsigned long sMag;
  unsigned long nMag;


  // count rev
  unsigned long sampleTime = millis() - elapsedSampleTime;
  bool revState = digitalRead(PIN_REV);

  if (sampleTime > 50 && revState != revStateOld) {
    if (risingEdge(revStateOld, revState)) {
      elapsedSampleTime = millis();
      rev++;
      digitalWrite(PIN_LED, HIGH);
    }
  }
  delay(20);
  digitalWrite(PIN_LED, LOW);


  // count mag
  sMag += (int)analogRead(PIN_MAG);
  nMag++;


  // notify every second
  if (intervalTime > 1000) {

    // cadence
#if defined(SIMULATION)
    // simulated
    cadence = 80;  // 80/100 > 30.0/37.5 kph
    crankrev = crankrev + 1;
    lastcrank = lastcrank + 1024 * 60 / cadence;
#else
    // measured
    crankrev = rev;
    lastcrank = elapsedSampleTime;
#endif

    // wheel rev - based on Apple Watch default wheel dimension 700c x 2.5mm
    wheelrev = crankrev * 3;

#if defined(CSC_MODE)
    lastwheel = lastcrank * 1;  // 1s/1024 granularity
#else
    lastwheel = lastcrank * 2;  // 1s/2048 granularity
#endif

    // power
#if defined(SIMULATION)
    power = 160;
#else
    power = (int)(100 * (1 - (sMag / (nMag * 4095))));
#endif

//double kph = calculateKphFromRpm(rpm);
//double power = calculatePowerFromKph(kph);

// serial output & notify
#if defined(CSC_MODE)
    Serial.printf("WR %4d WT %7d CR %4d CT %7d ", wheelrev, lastwheel, crankrev, lastcrank);
    serviceNotifyCSC(wheelrev, lastwheel, crankrev, lastcrank);
#else
    Serial.printf("PW %4d WR %4d WT %7d CR %4d CT %7d ", power, wheelrev, lastwheel, crankrev, lastcrank);
    serviceNotifyCP(power, wheelrev, lastwheel, crankrev, lastcrank);
#endif
    Serial.printf("\n");

    // loop varables
    sMag = 0;
    nMag = 0;
    elapsedTime = millis();
  }
}
