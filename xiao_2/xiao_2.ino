//receiver

#include <Wire.h>
#include <ArduinoBLE.h>           // ArduinoBLE 1.3.2 by Arduino

#include "PluggableUSBHID.h"
#include "USBMouse.h"

USBMouse Mouse;

#define VBAT_LOWER  3.5           // battery voltage lower limit
#define VBAT_UPPER  4.2           // battery voltage upper limit
#define dataNum 8                 // receive data number : roll, pitch, yaw, Vbatt (4 data 8 byte)

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN;

int range = 12;             // output range of X or Y movement
int responseDelay = 5;      // response delay of the mouse, in ms
int threshold = range / 4;  // resting threshold
int center = range / 2;

int xValue, yValue; 

const char* versionNumber = "0.90"; // version number
int BTS1 ,BTS2 ,BTS3;
float roll, pitch, yaw;             // attitude
float ax, ay, az;   
float Vbatt;                        // battery voltage
int ss;                             // peripheral signal strength
bool readingFlag = false;           // data buffer in use flag
int err;                            // error code of scan_connect function
bool LED_state;                     // LED ON/OFF state 

union unionData {                   // Union for bit convertion 16 <--> 8
  int16_t   dataBuff16[dataNum/2];
  uint8_t   dataBuff8[dataNum];
};

union unionData ud;

// Characteristic UUID
#define myUUID(val) ("0dd7eb5a-" val "-4f45-bcd7-94c674c3b25f")
//BLEService        AttService(myUUID("0000"));
BLECharacteristic dataCharacteristic(myUUID("0010"), BLEWrite | BLENotify, dataNum);

BLEDevice peripheral;

void setup()
{
  //initialize serial port
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  //set I/O pins
  pinMode(LED_RED, OUTPUT);       // LOW:LED ON, HIGH:LED OFF
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);


    BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  BLE.advertise();

 
  // initialize BLE
  BLE.begin(); 
}

//******************************************************************************************
// IMU data is received every 80mS
//
void loop() {
  
  // scan and connect the peripheral
  // return value of this function is an error code 
  err = scan_connect();

  BLEDevice central = BLE.central();

    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0
          Serial.println("LED on");
          digitalWrite(ledPin, HIGH);         // will turn the LED on
        } else {                              // a 0 value
          Serial.println(F("LED off"));
          digitalWrite(ledPin, LOW);          // will turn the LED off
        }
      }
    }
  
  // if connected? 
  if(err == 0) {            
    // main task while connected to the peripheral 60~78mS
    while (peripheral.connected()) {     
      LED_state = !LED_state;
      digitalWrite(LED_GREEN, (LED_state ? LOW : HIGH));   // connect indicator blinking
      
      long timestamp = millis();    // for timer
      
      // restoration of received data
      // prohibit event handler access while data buffer is in use          
      readingFlag = true;  
       
        roll   = ud.dataBuff16[0] * 180.0 / 32768.0;
        pitch  = ud.dataBuff16[1] * 180.0 / 32768.0;
        yaw    = ud.dataBuff16[2] * 360.0 / 32768.0;
        xValue = ud.dataBuff8[3];
        yValue = ud.dataBuff8[4]; 
        BTS1   = ud.dataBuff8[5];
        BTS2   = ud.dataBuff8[6];
        BTS3   = ud.dataBuff8[7];
        
      readingFlag = false;

      Serial.print(BTS1);   Serial.print(", ");
      Serial.print(BTS2);   Serial.print(", ");
      Serial.print(BTS3);   Serial.print(", ");

      Serial.print(xValue); Serial.print(", ");
      Serial.print(yValue); Serial.print(", ");

      // for serial plotter
      Serial.print(roll); Serial.print(", ");
      Serial.print(pitch); Serial.print(", ");
      Serial.println(yaw);


      int xReading = xValue;
      int yReading = yValue;

      Mouse.move(yReading,xReading);

       if (BTS1 == 0) {
       Mouse.press(MOUSE_LEFT);
       } else {
       Mouse.release(MOUSE_LEFT);}

      while(millis() - timestamp < 80);     // wait for loop time 80mS
    } //While connected
      
  } //if scan & connect
    
  // if disconnected from the peripheral
  digitalWrite(LED_GREEN, HIGH);            // connect indicator OFF
  Serial.print("Disconnected from the peripheral: ");
  Serial.println(peripheral.address());
  Serial.print("ERROR : ");                 // return value of scan_connect
  Serial.println(err);
      
} //loop


int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}


//**************************************************************************************************
// scan and connect the peripheral
// return value of this function is an error code
//
int scan_connect(void) {
  // scanning peripherals
  BLE.scanForUuid(myUUID("0000"));  
  Serial.println("1.SCANNING ................");

  peripheral = BLE.available();
  
  if (!peripheral) {
    Serial.println("2x.Peripheral unavailable");
    return 2;
  }
  Serial.println("2.Peripheral is available");
    
  if (peripheral.localName() != "Att_Monitor") {
    Serial.println("3x.Peripheral local name miss match");
    return 3;
  }
  Serial.println("3.Got the right peripheral");

  // stop scanning, connect the peripheral
  BLE.stopScan();
  Serial.println("4.Stop scanning");
  
  Serial.println("5.CONNECTING ................");
  if (!peripheral.connect()) {
    Serial.println("5x.Can't connect");
    return 5;
  } 
  Serial.println("5.Connected");
  
  if (!peripheral.discoverAttributes()) {
    Serial.println("6x.Didn't discover attributes");
    peripheral.disconnect();
    return 6;
  }
  Serial.println("6.Discovered attributes");

  dataCharacteristic = peripheral.characteristic(myUUID("0010"));         //dataCaracteristic UUID
  dataCharacteristic.setEventHandler(BLEWritten, characteristicWritten);  //BLEWritten handler
  Serial.println("7.Char and eventhandler setting");  

  if (!dataCharacteristic.canSubscribe()) {
    Serial.println("8x.Can't subscribe");
    peripheral.disconnect();
    return 8;
  }
  Serial.println("8.Can subscribe");

  if (!dataCharacteristic.subscribe()) {
    Serial.println("9x.Can't Subscribe");
    peripheral.disconnect();
    return 9;
  }
  Serial.println("9.Subscribed");

  Serial.println("10.Success scanning and connecting");
  return 0;
}

//****************************************************************************************************
// Characteristic written event handler
//
void characteristicWritten(BLEDevice peripheral, BLECharacteristic thisChar) {

  digitalWrite(LED_RED, LOW);         // event indicator ON
    
  // wait while data buffer is accessed in main loop  
  while(readingFlag == true) {
  }    
  dataCharacteristic.readValue(ud.dataBuff8, dataNum);  // read data packet 3.5uS  
  ss = peripheral.rssi();                               // read signal strength 5~20mS
    
  long timestamp = millis();
  while(millis() - timestamp <= 1);   // Delay to make LED visible
    
  digitalWrite(LED_RED, HIGH);        // event indicator OFF
}
