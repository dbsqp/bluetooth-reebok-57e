/*
    ESP32 interface for Reebok 5.7e indoor exercise bike

    Based on Neil Kolban example for IDF
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    based on esp32-ftms-server by jamesjmtaylor
    updated for Cadence/Speed, Power and Reebok 5.7 by dbsqp
*/

// NOTE : onboard LED indicates UART activity
// NOTE : https://www.bluetooth.org/en-us/specification/assigned-numbers-overview

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>



// options
#define USESLEEP  // use deep sleep with PIN_REED as wake [USEHALL overrides]
#define USEPOWER  // use power calculated empirically from cadence and electromagnet duty cycle P = f(D,C)
//#define USEALTPWR // use alternate power calculated from speed P = f(S)
//#define USEALTCOR // use alternate power correction factors c0 and c1
//#define USEDIRECT // invert reed sensor logic for direct connection to GPIO ie. GND-REED-GPIO
//#define USEHALL   // use ESP32 internal hall sensor as crank trigger at threshold HALLTRIG [ need to disable sleep ]
#define DEBUG     // add debug data to serial output
//#define DEBUG2    // add trigger events to serial output

// constants
const char  BTNAME[] = "Reebok 5.7e Bike"; // Bluetooth device name
const int   PIN_REED = 13;   // reed-switch. Crank event pulls GPIO high (indirect/PNP) or low (direct) + sleep wake
const int   PIN_EMAG = 14;   // electromagnet (5V 1kHz PWM, analogue sense via NPN)
const float SLEEPMIN = 3.5;  // minutes on inactivity until sleep - match bike sleep of 3.5 mins
const int   HALLTRIG = 100;  // trigger threshold for hall sensor

// estimated power correction factors
const float c0 = +57.03147; // offset
const float c1 = +0.222511; // linear

// emperical power function coefficients
const float cDD = +0.003426; // non-linear D^2
const float cCC = -0.003840; // non-linear C^2
const float cCD = +0.024750; // non-linear C*D
const float cD  = +3.941907; // linear D
const float cC  = +1.341997; // linear C
const float cO  = -105.5747; // offset



// option logic
#if defined(USEHALL)
  #undef USESLEEP
#endif



// globals
RTC_DATA_ATTR int bootCount = 0;
int sleepTrigger = (int)( SLEEPMIN * 60 * 1000 );

BLEServer *pServer;

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

bool connected = false;     //  current connection state
bool oldConnected = false;  // previous connection state
bool state = false;         //  current crank sensor state
bool oldState = false;      // previous crank sensor state

unsigned long lastNotify;   // time last notify
unsigned long lastTrigger;  // time last trigger
unsigned long triggerCount; // trigger count
unsigned long mag;          // analogue input read of electromagnet PWM
unsigned long sMag;         //   sum of analogue input reads between notifications
unsigned long nMag;         // count of analogue input reads between notifications

uint16_t crankCount;        // count rank revolutions
uint16_t lastCrankK;        // time last crank revolution 1/1024 s
uint32_t wheelCount;        // count wheel revolutions
uint16_t lastWheelK;        // time last wheel revolution 1/1024 s
uint16_t power;             // power in Watts

uint16_t lastCrank;         // time last crank revolution 1/1000 s = ms
uint16_t lastCrankCount;    // crankCount of previous notify
uint16_t cadence;           // crank rpm

double distance;            // total in km
double speed;               // speed in km/h
double powerM;              // power measured
double powerS;              // power from speed
double diffCrankTime;       // crank rotation time in ms

// connection status
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    connected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    connected = false;
  }
};



// main setup
void setup() {
  Serial.begin(115200);
  Serial.print("\n Start");

  #if defined(USESLEEP)
    setupSleep();
  #endif

  Serial.print("  Mode : ");
  #if defined(USEPOWER)
    Serial.print("Power [CP]");
    #if defined(USEALTPWR)
      Serial.print(" - Estimated");
      #if defined(USEALTCOR)
        Serial.print(" [Corrected]");
      #endif
    #endif
  #else
    Serial.print("Cadence [CSC]");
  #endif

  #if defined(USEHALL)
    Serial.println(" - Hall");
    setupHallSensor();
  #else    
    setupRevSensor();
  #endif

  setupMagSensor();
  setupBluetoothServer();
  setupHeaders();

      lastNotify = 0;
     lastTrigger = 0;
       lastCrank = 0;

    triggerCount = 0;
             mag = 0;

      crankCount = 0;
      lastCrankK = 0;
      wheelCount = 0;
      lastWheelK = 0;
           power = 0;

        distance = 0;
           speed = 0;
          powerM = 0;
          powerS = 0;
  lastCrankCount = 0;
}

// setup sleep
void setupSleep() {
  ++bootCount;
  Serial.printf(" : %d\n", bootCount);
  Serial.printf(" Sleep : %.1f mins [ %d s ]\n", SLEEPMIN, sleepTrigger/1000 );
}

// setup cadence sensor reed > GND
void setupRevSensor() {
  #if defined(USEDIRECT)
    Serial.println(" - Direct");
    pinMode(PIN_REED, INPUT_PULLUP);
  #else
    Serial.println("");
    pinMode(PIN_REED, INPUT_PULLDOWN);
  #endif

  oldState = digitalRead(PIN_REED);
}

// setup resistance sensor PWM > analogue
void setupMagSensor() {
  pinMode(PIN_EMAG, INPUT_PULLUP);
}

// setup hall sensor
void setupHallSensor() {
  oldState = false;
}

// setup Bluetooth
void setupBluetoothServer() {

  // create device
  BLEDevice::init(BTNAME);
  Serial.printf("Device : %s\n", BTNAME);

  // create server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

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

  #if defined(USESLEEP)
    Serial.print(" SLEEP : ");
  #else
    Serial.print("SAMPLE : ");
  #endif

  Serial.print("RPM  KPH     KM ");

  #if defined(DEBUG)
    Serial.print("- V-IN V-PM  D-PM - PWR-M PWR-S - ");
  #endif

  #if defined(USEPOWER)
    Serial.print("  PWR ");
  #endif
  
  Serial.println("  N-W   T-W  N-C   T-C");
}



// rising edge trigger function
inline bool risingEdge(bool &oldState, bool state) {
  bool result = (!oldState && state);  // !0 && 1

  #if defined(DEBUG2)
    if (result) {
      Serial.println(" Debug : trigger - rising");
    }
  #endif

  oldState = state;
  return result;
}

// falling edge trigger function
inline bool fallingEdge(bool &oldState, bool state) {
  bool result = (oldState && !state);  // 1 && !0

  #if defined(DEBUG2)
    if (!result) {
      Serial.println(" Debug : trigger - falling");
    }
  #endif

  oldState = state;
  return result;
}



// calculations
double powerFromSpeed(double kph) {
  double riderWeight = 72.0;   // kg
  double bikeWeight = 12.0;    // kg
  double rollingRes = 0.004;
  double frontalArea = 0.445;  // Bartops
  double grade = 0;            // °
  double headwind = 0;         // m/s
  double temperature = 15.0;   // °C
  double elevation = 100;      // m
  double transv = 0.95;        // unknown

  double velocity = kph * 0.2777;  // m/s
  double density = (1.293 - 0.00426 * temperature) * exp(-elevation / 7000.0);
  double twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
  double A2 = 0.5 * frontalArea * density;        // full air resistance parameter
  double tres = twt * (grade + rollingRes);       // gravity and rolling resistance
  double tv = velocity + headwind;                // terminal velocity
  double A2Eff = (tv > 0.0) ? A2 : -A2;           // reverse effect wind in face

  return (velocity * tres + velocity * tv * tv * A2Eff) / transv;
}

double powerFromDutyAndCadance(double D, double C ) {
  double Pdd = cDD* D * D;
  double Pcc = cCC * C * C;
  double Pcd = cCD * C * D;
  double Pc  = cC * C;
  double Pd  = cD * D;
  double Po  = cO;

  return Pdd + Pcc + Pcd + Pc + Pd + Po;
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
  bool disconnecting = !connected && oldConnected;
  bool connecting = connected && !oldConnected;

  // notify with measurements if connected
  if (connected) {
    measurementCharacteristics.setValue(measurement, 11);
    measurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("\nClient : disconnected");
    Serial.print("Server : Advertising...");

    oldConnected = connected;
  }

  // feature notify if connecting
  if (connecting) {
    oldConnected = connected;
    Serial.println("\nClient : connected");
    featureCharacteristics.setValue(feature, 1);
    Serial.print("Server : set features");
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
  bool disconnecting = !connected && oldConnected;
  bool connecting = connected && !oldConnected;

  // notify with measurements if connected
  if (connected) {
    measurementCharacteristics.setValue(measurement, 16);
    measurementCharacteristics.notify();
    Serial.print(">> client");
  }

  // restart advertising if disconnected
  if (disconnecting) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("\nClient : disconnected");
    Serial.print("Server : Advertising...");
    oldConnected = connected;
  }

  // feature notify if connecting
  if (connecting) {
    Serial.println("\nClient: connected");
    oldConnected = connected;
    featureCharacteristics.setValue(feature, 1);
    Serial.print("Server : set features");
  }
}



// main loop
void loop() {
  unsigned long sinceNotify = millis() - lastNotify;    // ms since last notify


  // count crank revolutions
  unsigned long sinceTrigger = millis() - lastTrigger;  // ms since last trigger
  bool edge;

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
    #if defined(USEDIRECT)
      edge =  risingEdge(oldState, state);
    #else
      edge = fallingEdge(oldState, state);
    #endif

    if ( edge ) {
      lastTrigger = millis();
      triggerCount++;
    }
  }

  // average electromagnet PWM voltage
  mag = (int)analogRead(PIN_EMAG);
  sMag += mag;
  nMag ++;


  // notify bluetooth client every second
  if (sinceNotify >= 1000) {

    // sleep - wake via crank reed sensor
    #if defined(USESLEEP)
      if (sinceTrigger >= sleepTrigger ) {
        Serial.println(" Sleep : trigger crank to wake\n");
        esp_bt_controller_disable();

        #if defined(USEDIRECT)
          esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,0);
        #else
          esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);
        #endif

        esp_deep_sleep_start();
      } else {
        Serial.printf("%6d : ", (sleepTrigger - sinceTrigger)/1000);
      }
    #else
        Serial.printf("%6d : ", sinceTrigger/1000);
    #endif


    // cadence
    uint16_t diffCrankCount = triggerCount - lastCrankCount;
    uint16_t diffCrank = lastTrigger - lastCrank;
    
    crankCount = triggerCount;
     lastCrank = lastTrigger;
    lastCrankK = (int)( 1.0 * 1024 * lastTrigger / 1000.0 );  // 1/1024 s granularity

    if ( diffCrank > 0 ) {
      cadence = (int)( diffCrankCount / ( diffCrank / ( 1000*60.0 ) ) );
      lastCrankCount = crankCount;
      if ( diffCrank > 9999 ){
        diffCrank = 9999;
        diffCrankTime = 9999;
      } else {
        diffCrankTime = (int)( diffCrank / diffCrankCount );
      }
    } else {
      cadence = 0;
      diffCrankTime = 0;
    }


    // speed & distance
    // NOTE : based on Apple Watch default wheel dimension 700c x 2.5mm
    // NOTE : 3 is theoretical crank:wheel gear ratio 
    // NOTE : 2.13 is circumference of 700c in meters

    wheelCount = crankCount * 3;
         speed = cadence * 3 * 2.13 * 60 / 1000;
      distance = wheelCount * 2.13 / 1000;

    #if defined(USEPOWER)
      lastWheelK = lastCrankK * 2;  // 1/2048 s granularity
    #else
      lastWheelK = lastCrankK * 1;  // 1/1024 s granularity
    #endif

    // power
    #if defined(USEALTCOR)
      powerS = powerFromSpeed(speed) * c1 + c0;
    #else
      powerS = powerFromSpeed(speed);
    #endif

    double aMag = sMag/nMag;
    double vPIN = 3.3 * aMag/4095;
    double vPWM = 5.0 - (5.0 * aMag/4095);
    double dPWM = 100.0 * (vPIN/0.564);

    if ( diffCrank > 0 ) {
      powerM = powerFromDutyAndCadance(dPWM, cadence);
    } else {
      powerM = 0;
    }

    #if defined(USEALTPWR)
      power = (int)( powerS + 0.5 );
    #else
      power = (int)( powerM + 0.5 );
    #endif


    // serial output
    Serial.printf("%3d %4.1f %6.3f ", cadence, speed, distance);

    #if defined(DEBUG)
        Serial.printf("- %4.2f %4.2f %5.1f - %5.1f %5.1f - ", vPIN, vPWM, dPWM, powerM, powerS);
    #endif

    #if defined(USEPOWER)
      Serial.printf("%5d ", power);
    #endif

    Serial.printf("%5d %5d %4d %5d ", wheelCount, lastWheelK, crankCount, lastCrankK);


    // notify
    #if defined(USEPOWER)
      serviceNotifyCP(power, wheelCount, lastWheelK, crankCount, lastCrankK);
    #else
      serviceNotifyCSC(wheelCount, lastWheelK, crankCount, lastCrankK);
    #endif

    Serial.println("");


    // reset loop variables
    sMag = 0;
    nMag = 0;
    lastNotify = millis();
  }
}
