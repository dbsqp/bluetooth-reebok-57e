/*
    ESP32 interface for Reebok 5.7e indoor exercise bike

    Based on Neil Kolban example for IDF
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    based on esp32-ftms-server by jamesjmtaylor
    updated for Cadance/Speed, Power and Reebok 5.7 by dbsqp
*/

// NOTE : onboard LED indicates UART activity

// TODO : implement final power = f(cadance, resistance)
// TODO : only report power if pedelling

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>



// options
#define DEBUG     // add debug data to serial notify output
#define USEPOWER  // create Power, Cadence & Speed insted of basic Cadence & Speed 
#define USEAPPROX // use approx power calculated from speed instead of measurement
#define USESLEEP  // sleep and wake via PIN_REED [USEHALL overrides]
//#define USEHALL  // use builtin hall effect sensor of ESP32 to as crank trigger at > HALLTRIG

const char SERVER_NAME[] = "Reebok 5.7e Bike"; // Bluetooth device name
const int   PIN_REED = 13;  // reed-switch (trigger pulls low, digital sense via PNP) + sleep wake
const int   PIN_EMAG = 14;  // electromagnet (5V 1kHz PWM, analogue sense via NPN)
const float SLEEPMIN = 3.5; // minuites on inactivity until sleep - match bike sleep of 3.5 mins
const int   HALLTRIG = 100; // trigger threshold for hall sensor

#if defined(USEHALL)
  #undef(USESLEEP)  // no sleep with hall sensor as can not easily wake
#endif



// globals
RTC_DATA_ATTR int bootCount = 0;
int sleepTrigger = (int)( SLEEPMIN * 60 * 1000 );

BLEServer *pServer;

// https://www.bluetooth.org/en-us/specification/assigned-numbers-overview
#if defined(USEPOWER)
  #define SERVICE_UUID BLEUUID((uint16_t)0x1818)
  BLECharacteristic featureCharacteristics(BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
  BLECharacteristic measurementCharacteristics(BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
#else
  #define SERVICE_UUID BLEUUID((uint16_t)0x1816)
  BLECharacteristic featureCharacteristics(BLEUUID((uint16_t)0x2A5C), BLECharacteristic::PROPERTY_READ);
  BLECharacteristic measurementCharacteristics(BLEUUID((uint16_t)0x2A5B), BLECharacteristic::PROPERTY_NOTIFY);
  BLECharacteristic scControlPointCharacteristics(BLEUUID((uint16_t)0x2A55), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);
  BLEDescriptor scControlPointDescriptor(BLEUUID((uint16_t)0x2901));
#endif

bool deviceConnected = false;
bool oldDeviceConnected = false;


bool oldState;              // crank sensor state

unsigned long lastNotify;   // time last notify
unsigned long lastTrigger;  // time last trigger
unsigned long triggerCount; // trigger count
unsigned long mag;          // analogue input read of electromagnet PWM
unsigned long sMag;         //   sum of analogue input reads between notifications
unsigned long nMag;         // count of analogue input reads between notifications

uint16_t crankCount;        // count rank revolutions
uint16_t lastCrank;         // time last crank revolution
uint32_t wheelCount;        // count wheel revolutions
uint16_t lastWheel;         // time last wheel revolution 
uint16_t power;             // power in Watts
uint16_t cadence;           // crank revolutions per minuite

double distance;            // km
double speed;               // speed in km/h


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
  Serial.print("\n Start");

  #if defined(USESLEEP)
    setupSleep();
  #endif

  #if defined(USEHALL)
    setupHallSensor();
  #else
    setupRevSensor();
  #endif

  setupMagSensor();
  setupBluetoothServer();

  setupHeaders();

    lastNotify = 0;
   lastTrigger = 0;

  triggerCount = 0;
           mag = 0;

    crankCount = 0;
     lastCrank = 0;
    wheelCount = 0;
     lastWheel = 0;
         power = 0;

      distance = 0;
}

// setup sleep
void setupSleep() {
  ++bootCount;
  Serial.printf(" : %d\n", bootCount);
  Serial.printf(" Sleep : %.1f mins [ %'d ms ]\n", SLEEPMIN, sleepTrigger );
}

// setup cadance sensor reed > GND
void setupRevSensor() {
  pinMode(PIN_REED, INPUT);
  oldState = digitalRead(PIN_REED);
}

// setup resistance sensor PWM > analogue
void setupMagSensor() {
  pinMode(PIN_EMAG, INPUT);
}

// setup hall sensor
void setupHallSensor() {
  oldState = false;
}

// setup Bluetooth
void setupBluetoothServer() {

  // create device
  BLEDevice::init(SERVER_NAME);
  Serial.printf("Server : %s", SERVER_NAME);

  // create server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  #if defined(USEPOWER)
    Serial.print(" - Power [CP]");
    #if defined(USEAPPROX)
      Serial.print(" - Approx");
    #endif
  #else
    Serial.print(" - Cadence [CSC]");
  #endif

  #if defined(USEHALL)
    Serial.println(" - Hall");
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

// setup headers
void setupHeaders() {
  Serial.print("SAMPLE : CAD  KPH   KM : ");

  #if defined(DEBUG)
    Serial.print(" Vin Vpwm DUTY% : ");
  #endif

  #if defined(USEPOWER)
    Serial.print("WATT ");
  #endif
  
  Serial.println(" W-#  W-TIME  C-#  C-TIME");
}



// rising edge trigger function
inline bool risingEdge(bool &oldState, bool state) {
  bool result = (!oldState && state);  // !0 && 1

  #if defined(DEBUG)
    if (result) {
      Serial.println("DEBUG trigger _/");
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
      Serial.println("DEBUG trigger \\_");
    }
  #endif

  oldState = state;
  return result;
}



// calculations
double powerFromSpeed(double kph) {
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
    Serial.print(" >> client");
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
    Serial.print(" >> client");
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
  unsigned long sinceNotify = millis() - lastNotify;    // ms since last notify


  // count crank revolutions
  unsigned long sinceTrigger = millis() - lastTrigger;  // ms since last trigger
  bool state;

  #if defined(USEHALL)
    // ESP32 build-in hall sensor
    if ( abs(hallRead()) > HALLTRIG ) {
      state = true;
    } else {
      state = false;
    }
  #else
    // external reed sensor
    state = digitalRead(PIN_REED);
  #endif

  if (sinceTrigger > 400 && state != oldState) {
    if (fallingEdge(oldState, state)) {
      lastTrigger = millis();
      triggerCount++;
    }
  }

  // average electromagnet PWM voltage
  mag = (int)analogRead(PIN_EMAG);

  // add logic to report zero power when not pedelling
  // if pedeling (rev in last two seconds C = 30) and not max (PWM glitch)
  // if (sinceTrigger < 2000 && mag != 4095) {
     sMag += mag;
     nMag ++;
  // }


  // notify bluetooth client every 2 seconds
  if (sinceNotify >= 2000) {

    // cadence
    if ( lastCrank != 0 && triggerCount > crankCount ) {
      cadence = (int)( ( triggerCount - crankCount ) / ( ( lastTrigger - lastCrank ) / ( 1000*60.0 ) ) );
    } else {
      cadence = 0;
    }
    crankCount = triggerCount;
      lastCrank = lastTrigger;

    // wheel rev
    // NOTE : based on Apple Watch default wheel dimension 700c x 2.5mm
    // NOTE : 3 is theoretical crank:wheel gear ratio 
    wheelCount = crankCount * 3;

    #if defined(USEPOWER)
      lastWheel = lastCrank * 2;  // 1/2048 s granularity
    #else
      lastWheel = lastCrank * 1;  // 1/1024 s granularity
    #endif

    // speed & distance
    // NOTE : 2.13 m is circumference of 700c
    speed = cadence * 3 * 2.13 * 60 / 1000;
    distance = wheelCount * 2.13 / 1000;

    // power
    double aMag = sMag/nMag;
    double vPIN = 3.3 * aMag/4095;
    double vPWM = 5.0 - (5.0 * aMag/4095);
    double DUTY = 100.0 * vPWM/4.348;

    #if defined(USEAPPROX)
      power = (int) powerFromSpeed(speed);
    #else
      power = (int)(DUTY);
    #endif


    // serial output
    Serial.printf("%6d : ", sinceTrigger);
    Serial.printf("%3d %4.1f %4.1f : ", cadence, speed, distance);

    #if defined(DEBUG)
        Serial.printf("%4.2f %4.2f %5.1f : ", vPIN, vPWM, DUTY);
    #endif

    #if defined(USEPOWER)
      Serial.printf("%4d ", power);
    #endif

    Serial.printf("%4d %7d %4d %7d ", wheelCount, lastWheel, crankCount, lastCrank);


    // notify
    #if defined(USEPOWER)
      serviceNotifyCP(power, wheelCount, lastWheel, crankCount, lastCrank);
    #else
      serviceNotifyCSC(wheelCount, lastWheel, crankCount, lastCrank);
    #endif

    Serial.println("");


    // reset loop variables
    sMag = 0;
    nMag = 0;
    lastNotify = millis();


    // sleep 
    #if defined(USESLEEP)
      if (sinceTrigger >= sleepTrigger ) {
        Serial.println(" Sleep : trigger crank sensor to wake\n");

        // shutdown bluetooth
        esp_bt_controller_disable();

        // external wake via crank reed low > PNP high
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);

        // sleep
        esp_deep_sleep_start();
      }
    #endif

  }
}
