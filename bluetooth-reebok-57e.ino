/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    based on esp32-ftms-server by jamesjmtaylor
    edited for Reebok 5.7e indoor exercise bike by dbsqp
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>


// options
#define CSC_MODE
#define SERVER_NAME "Reebok 5.7e Bike"

// globals
BLEServer *pServer;

uint16_t crankrev;  // Cadence RPM
uint16_t lastcrank; // Last crank time
uint32_t wheelrev;  // Wheel revolutions
uint16_t lastwheel; // Last crank time
uint16_t power;     // power [Watts]

uint16_t cadence;

unsigned long elapsedTime;
unsigned long elapsedSampleTime;
int rev;
double intervalEntries;

// set bit:feature
// CSC 16bit/0-15 - 2/2:multi-location 1/1:crankRev 0/0:wheelRev
byte cscFeature[1] = { 0b0000000000000011 };
byte cscMeasurement[11] = { 0b00000011, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// CP 32 bit/0-31 - 3/5:crankRev
byte cpFeature[1] = { 0b00000000000000000000000000001000 };
byte cpMeasurement[11] = { 0b0000000000100000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

byte sensorLocation[1] = { 6 };

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool magStateOld;

int digitalPin = 18;

#define LED_BUILTIN LED_BUILTIN

// https://www.bluetooth.org/en-us/specification/assigned-numbers-overview

#define CSC_SERVICE_UUID BLEUUID ((uint16_t) 0x1816)

BLECharacteristic cscMeasurementCharacteristics( BLEUUID ((uint16_t) 0x2A5B), BLECharacteristic::PROPERTY_NOTIFY); // CSC Measurement Characteristic
BLECharacteristic cscFeatureCharacteristics(     BLEUUID ((uint16_t) 0x2A5C), BLECharacteristic::PROPERTY_READ);   // CSC Feature Characteristic
BLECharacteristic scControlPointCharacteristics( BLEUUID ((uint16_t) 0x2A55), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE); // SC Control Point Characteristic  

BLEDescriptor cscMeasurementDescriptor( BLEUUID ((uint16_t) 0x2901)); //0x2901 is a custom user description
BLEDescriptor cscFeatureDescriptor(     BLEUUID ((uint16_t) 0x2901));
BLEDescriptor scControlPointDescriptor( BLEUUID ((uint16_t) 0x2901));


#define CP_SERVICE_UUID BLEUUID ((uint16_t) 0x1818)

BLECharacteristic cpMeasurementCharacteristics(  BLEUUID ((uint16_t) 0x2A63), BLECharacteristic::PROPERTY_NOTIFY); // CP Measurement Characteristic
BLECharacteristic cpFeatureCharacteristics(      BLEUUID ((uint16_t) 0x2A65), BLECharacteristic::PROPERTY_READ);   // CP Feature Characteristic

//0x2901 is a custom user description
BLEDescriptor cpMeasurementDescriptor(  BLEUUID ((uint16_t) 0x2901));
BLEDescriptor cpFeatureDescriptor(      BLEUUID ((uint16_t) 0x2901));



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
  Serial.print("Mode: ");

  #if defined(CSC_MODE)
    Serial.println("Cadence");
  #else
    Serial.println("Power");
  #endif

  pinMode(LED_BUILTIN, OUTPUT);
  setupBluetoothServer();
  setupHallSensor();

        elapsedTime = 0;
  elapsedSampleTime = 0;
    intervalEntries = 0;

        rev = 0;
   crankrev = 0;
  lastcrank = 0;
   wheelrev = 0;
  lastwheel = 0;
      power = 0;
}



// setup BLE
void setupBluetoothServer() {
  Serial.println("Starting server");
  Serial.println("Name: " SERVER_NAME);

  // create BLE device
  BLEDevice::init(SERVER_NAME);
  
  // create BLE server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // create service & add features
  #if defined(CSC_MODE)
    BLEService *pService = pServer->createService(CSC_SERVICE_UUID);

    pService->addCharacteristic( &cscMeasurementCharacteristics);
    pService->addCharacteristic( &cscFeatureCharacteristics);

    cscMeasurementDescriptor.setValue( "Exercise Bike CSC Measurement");
    cscMeasurementCharacteristics.addDescriptor( &cscMeasurementDescriptor);
    cscMeasurementCharacteristics.addDescriptor( new BLE2902());

    cscFeatureDescriptor.setValue ("Exercise Bike CSC Feature");
    cscFeatureCharacteristics.addDescriptor (&cscFeatureDescriptor);
  #else
    BLEService *pService = pServer->createService(CP_SERVICE_UUID);

    pService->addCharacteristic( &cpMeasurementCharacteristics);
    pService->addCharacteristic( &cpFeatureCharacteristics);

    cpMeasurementDescriptor.setValue( "Exercise Bike Power Measurement");
    cpMeasurementCharacteristics.addDescriptor( &cpMeasurementDescriptor);
    cpMeasurementCharacteristics.addDescriptor( new BLE2902());

    cpFeatureDescriptor.setValue ("Exercise Bike Power Feature");
    cpFeatureCharacteristics.addDescriptor (&cpFeatureDescriptor);
  #endif

  // start service
  pService->start();

  // start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  #ifdef defined(CSC_MODE)
    pAdvertising->addServiceUUID(CSC_SERVICE_UUID);
  #else
    pAdvertising->addServiceUUID(CP_SERVICE_UUID);
  #endif

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  pServer->getAdvertising()->start();
  Serial.println("Advertising service to clients...");
}



// setup Hall sensor
void setupHallSensor() {
  pinMode(digitalPin, INPUT);
  magStateOld = digitalRead(digitalPin);
}



//incrementRevolutions() used to synchronously update rev rather than using an ISR.
inline bool positiveEdge(bool state, bool &oldState) {
  bool result = (state && !oldState);  //latch logic
  oldState = state;
  return result;
}



// calculations
double calculateRpmFromRevolutions(int revolutions, unsigned long revolutionsTime) {
  double ROAD_WHEEL_TO_TACH_WHEEL_RATIO = 10.0;
  double instantaneousRpm = revolutions * 60 * 1000 / revolutionsTime / ROAD_WHEEL_TO_TACH_WHEEL_RATIO;
  // Serial.printf("revolutionsTime: %d, rev: %d , instantaneousRpm: %2.9f \n", revolutionsTime, revolutions, instantaneousRpm);
  return instantaneousRpm;
}

double calculateKphFromRpm(double rpm) {
  double WHEEL_RADIUS = 0.00034;  // in km
  double KM_TO_MI = 0.621371;

  double circumfrence = 2 * PI * WHEEL_RADIUS;
  double metricDistance = rpm * circumfrence;
  double kph = metricDistance * 60;
  // Serial.printf("rpm: %2.2f, circumfrence: %2.2f, distance %2.5f , speed: %2.2f \n", rpm, circumfrence, metricDistance, kmh);
  return kph;
}

unsigned long distanceTime = 0;
double calculateDistanceFromKph(unsigned long distanceTimeSpan, double kph) {
  double incrementalDistance = distanceTimeSpan * kph / 60 / 60 / 1000;
  // Serial.printf("kph: %2.2f, distanceTimeSpan %d , incrementalDistance: %2.9f \n", kph, distanceTimeSpan, incrementalDistance);
  return incrementalDistance;
}

double tireValues[] = { 0.005, 0.004, 0.012 };                       //Clincher, Tubelar, MTB
double aeroValues[] = { 0.388, 0.445, 0.420, 0.300, 0.233, 0.200 };  //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;
double calculatePowerFromKph(double kph) {
  double velocity = kph * 0.277778;  // translates to meters/second
  double riderWeight = 72.6;         //165 lbs
  double bikeWeight = 11.1;          //Cannondale road bike
  int theTire = 0;                   //Clinchers
  double rollingRes = tireValues[theTire];
  int theAero = 1;  //Bartops
  double frontalArea = aeroValues[theAero];
  double grade = 0;
  double headwind = 0;         // converted to m/s
  double temperaturev = 15.6;  // 60 degrees farenheit
  double elevation = 100;      // Meters
  double transv = 0.95;        // no one knows what this is, so why bother presenting a choice?

  /* Common calculations */
  double density = (1.293 - 0.00426 * temperaturev) * exp(-elevation / 7000.0);
  double twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
  double A2 = 0.5 * frontalArea * density;        // full air resistance parameter
  double tres = twt * (grade + rollingRes);       // gravity and rolling resistance

  // we calculate power from velocity
  double tv = velocity + headwind;       //terminal velocity
  double A2Eff = (tv > 0.0) ? A2 : -A2;  // wind in face so you must reverse effect
  return (velocity * tres + velocity * tv * tv * A2Eff) / transv;
}

double calculateCaloriesFromPower(unsigned long caloriesTimeSpan, double powerv) {
  double JOULE_TO_KCAL = 0.238902957619;
  // From the formula: Energy (Joules) = Power (Watts) * Time (Seconds)
  double incrementalCalories = powerv * caloriesTimeSpan / 60 / 1000 * JOULE_TO_KCAL;
  double wl = incrementalCalories / 32318.0;  // comes from 1 lb = 3500 Calories
  return incrementalCalories;
}



void indicateRpmWithLight(int rpm) {
  if (rpm > 1) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn on LED
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // turn off LED
  }
}






void serviceNotifyCSC(int wheelrev, int lastwheel, int crankrev, int lastcrank) {

  // set measurement
  cscMeasurement[1] = wheelrev & 0xFF;
  cscMeasurement[2] = (wheelrev >> 8) & 0xFF; 

  cscMeasurement[3] = (wheelrev >> 16) & 0xFF; 
  cscMeasurement[4] = (wheelrev >> 24) & 0xFF; 
  
  cscMeasurement[5] = lastwheel & 0xFF;
  cscMeasurement[6] = (lastwheel >> 8) & 0xFF; 

  cscMeasurement[7] = crankrev & 0xFF;
  cscMeasurement[8] = (crankrev >> 8) & 0xFF; 

  cscMeasurement[9] = lastcrank & 0xFF;
  cscMeasurement[10] = (lastcrank >> 8) & 0xFF; 


  // monitor state
  bool disconnecting = !deviceConnected && oldDeviceConnected;
  bool connecting = deviceConnected && !oldDeviceConnected;

  // notify with data if connected
  if (deviceConnected) {
    cscMeasurementCharacteristics.setValue(cscMeasurement, 11);
    cscMeasurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
    Serial.print("nClient disconnected, re-advertising...");
    oldDeviceConnected = deviceConnected;
  }

  // feature notify if connecting
  if (connecting) {
    Serial.print("\nConnecting...");
    oldDeviceConnected = deviceConnected;
    cscFeatureCharacteristics.setValue(cscFeature, 1);
    Serial.print("notified client of features");
  }
}



void serviceNotifyCP(int power, int crankrev, int lastcrank) {

  // set measurement
  cpMeasurement[2] = power & 0xFF;
  cpMeasurement[3] = (power >> 8) & 0xFF; 
  
  cpMeasurement[5] = crankrev & 0xFF;
  cpMeasurement[6] = (crankrev >> 8) & 0xFF; 

  cpMeasurement[7] = lastcrank & 0xFF;
  cpMeasurement[8] = (lastcrank >> 8) & 0xFF; 


  // connection state
  bool disconnecting = !deviceConnected && oldDeviceConnected;
  bool connecting = deviceConnected && !oldDeviceConnected;

  // notify with data if connected
  if (deviceConnected) {
    cpMeasurementCharacteristics.setValue(cpMeasurement, 11);
    cpMeasurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
        Serial.print("\nClient disconnected, re-advertising...");
    oldDeviceConnected = deviceConnected;
  }

  // feature notify if connecting
  if (connecting) {
    Serial.print("\nConnecting...");
    oldDeviceConnected = deviceConnected;
    cpFeatureCharacteristics.setValue(cpFeature, 1);
    Serial.print("notified client of features");
  }
}



// main loop
void loop() {
  unsigned long intervalTime = millis() - elapsedTime;

  // lincrement rev
  unsigned long sampleTime = millis() - elapsedSampleTime;
  bool state = digitalRead(digitalPin);

  if (sampleTime > 5 && state != magStateOld) {
    rev += (int)positiveEdge(state, magStateOld);
    elapsedSampleTime = millis();
  }

  // notify every second
  if (intervalTime > 1000) {
    // simulation
    int rev = (int)(80 + 10 * sin(2.0 * 3.14159 * elapsedTime / 50.0));
    cadence = 50;

     crankrev = crankrev + 1;
    lastcrank = lastcrank + 1024*60/cadence;
     wheelrev = wheelrev + 1;
    lastwheel = lastwheel + 10*1024*60/cadence;

        power = rev*2;

    //double rpm = calculateRpmFromRevolutions(rev, intervalTime);
    //double kph = calculateKphFromRpm(rpm);
    //double power = calculatePowerFromKph(kph);

    // serial output & notify
    #if defined(CSC_MODE)
      Serial.printf("WR %4d WT %7d CR %4d CT %7d ", wheelrev, lastwheel, crankrev, lastcrank);
      serviceNotifyCSC(wheelrev,lastwheel,crankrev,lastcrank);
    #else
      Serial.printf("P %4d CR %4d CT %7d ", power, crankrev, lastcrank);
      serviceNotifyCP(power,crankrev,lastcrank);
    #endif

    // LED status
    indicateRpmWithLight(wheelrev);

    // loop varables
    Serial.printf("\n");
    rev = 0;
    intervalEntries++;
    elapsedTime = millis();
  }
}
