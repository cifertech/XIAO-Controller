//transiver

#include <Wire.h>
#include <ArduinoBLE.h>     // ArduinoBLE 1.3.2 by Arduino       
#include <MadgwickAHRS.h>   // Madgwick 1.2.0 by Arduino
#include <LSM6DS3.h>        // Seeed Arduino LSM6DS3 2.0.3  by Seeed Studio
/*
#define AVENUM      32      // average number of ADC
#define HICHG       P0_13   // charge current setting pin Open:50mA Lo:100mA
#define CHG         P0_17   // charge indicator pin Open:discharge Lo:charge(LED ON)
#define VBAT_ENABLE P0_14   // battery vontage read enable Open:disable Lo:enable
#define VBAT_READ   P0_31   // battery voltage monitor pin
#define VBAT_LOWER  3.5     // battery voltage lower limit
#define VBAT_UPPER  4.2     // battery voltage upper limit
*/
#define dataNum 8           //send data number : rolll, pitch, yaw, Vbatt (4 data 8 byte)

LSM6DS3 myIMU(I2C_MODE, 0x6A); // IMU
Madgwick  filter;              // Madgwick filter

//#include <Adafruit_NeoPixel.h>
//const int PIN = 3; 
//const int NUMPIXELS = 12;

//uint16_t current_hue = 0; 

//Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int BT1 = 4; 
const int BT2 = 5; 
const int BT3 = 6; 

int BTS1 = LOW;
int BTS2 = LOW;
int BTS3 = LOW;

#define VRX_PIN  A0 // Arduino pin connected to VRX pin
#define VRY_PIN  A1 // Arduino pin connected to VRY pin

int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis

const char* versionNumber = "0.90"; // version number
float ax, ay, az;                   // Accel
float gx, gy, gz;                   // Gyro
float Vbatt;                        // battery voltage
float roll, pitch, yaw;             // attitude
uint8_t readData;                   // for reading IMU register
uint32_t timestamp;                 // for delay function
bool LED_state;                     // LED ON/OFF state

union unionData {                   //Union for bit convertion 16 <--> 8
  int16_t   dataBuff16[dataNum/2];
  uint8_t   dataBuff8[dataNum];
};

union unionData ud;

//generated Linux command uuidgen "0dd7eb5a-a8b9-4f45-bcd7-94c674c3b25f"
#define myUUID(val) ("0dd7eb5a-" val "-4f45-bcd7-94c674c3b25f")
BLEService        AttService(myUUID("0000"));
BLECharacteristic dataCharacteristic(myUUID("0010"), BLERead | BLENotify, dataNum);

void setup() {
  //initialize serial port
  Serial.begin(115200);
//  while (!Serial) { delay(1); }

  //pixels.begin();

  pinMode(BT1, INPUT_PULLUP);
  pinMode(BT2, INPUT_PULLUP);
  pinMode(BT3, INPUT_PULLUP);

  //set I/O pins
  pinMode(LED_RED, OUTPUT);       // LOW:LED ON, HIGH:LED OFF
  pinMode(LED_GREEN, OUTPUT);
//  pinMode(HICHG, OUTPUT);
  //pinMode(CHG, INPUT);
  //pinMode(VBAT_ENABLE, OUTPUT);
  //pinMode(VBAT_READ, INPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  //digitalWrite(HICHG, LOW);         // charge current 100mA
  //digitalWrite(VBAT_ENABLE, LOW);   // battery voltage read enable

  // initialize ADC 2.4V/4096
  //analogReference(AR_INTERNAL2V4);  // Vref=2.4V
  //analogAcquisitionTime(AT_10_US);
  //analogReadResolution(12);         // 4096

  // initialize and set IMU
  // refer to   LSM6D3.cpp:351
  myIMU.settings.gyroRange = 2000;  // calcGyro()
  myIMU.settings.accelRange = 4;    // calcAccel()
  
  if (myIMU.begin() != 0) {                                      
    Serial.println("IMU Device error");
    while(1);
  }
  
  Wire1.setClock(400000UL);  //SCL 400kHz
    
  // change defalt settings, refer to data sheet 9.13, 9.14, 9.19, 9.20
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x1C);    // 12.5Hz 2000dps
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x1A);   // 12.5Hz 4G 
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);    // HPF 16mHz
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);   // ODR/4

  // Maadgwick filter sampling rate
  filter.begin(12.5);  

  // initialize BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE module failed!");
    while (1);
  }

  // set the local name
  BLE.setLocalName("Att_Monitor");
  // set the device name
  BLE.setDeviceName("XIAO nRF52840 Sence");
  // set the UUID for the service
  BLE.setAdvertisedService(AttService);
  // add the characteristic to the service  
  AttService.addCharacteristic(dataCharacteristic);
  // add service  
  BLE.addService(AttService);

  // start advertising
  BLE.setAdvertisingInterval(160);    //0.625mS * 160 = 100mS
  BLE.setConnectionInterval(6, 3200); //1.25mS * 6 = 7.5mS, 1.25mS * 3200 = 4S
  BLE.advertise();

}

void loop() {

  /*current_hue = current_hue + 1; 
    
  for (int i = 0; i < NUMPIXELS; ++i) { 
    pixels.setPixelColor(i, pixels.ColorHSV(current_hue, 255, 255)); 
  }  
  pixels.show();
  */
  // connect the sentral
  Serial.println("Connecting to Central ........");
  BLEDevice central = BLE.central();

  if (central) {    // connected to Central? 

    // while connected Central
    Serial.println("Connected Central");
    while(central.connect()) {      
      LED_state = !LED_state;
      digitalWrite(LED_GREEN, (LED_state ? LOW : HIGH));   // connect indicator blinking

      // wait for IMU data to become valid
      // sample rate is 12.5Hz, so can read every 80mS    
      do {        
        myIMU.readRegister(&readData, LSM6DS3_ACC_GYRO_STATUS_REG);   //0,0,0,0,0,TDA,GDA,XLDA
      } while ((readData & 0x07) != 0x07);        

      digitalWrite(LED_RED, LOW);   // data read and send task indicator ON
               
      ax = myIMU.readFloatAccelX(); // Accel data
      ay = myIMU.readFloatAccelY();
      az = myIMU.readFloatAccelZ();
      gx = myIMU.readFloatGyroX();  // Gyro data
      gy = myIMU.readFloatGyroY();
      gz = myIMU.readFloatGyroZ();

      // calculate the attitude with Madgwick filter
      filter.updateIMU(gx, gy, gz, ax, ay, az);

      roll = filter.getRoll();    // -180 ~ 180deg
      pitch = filter.getPitch();  // -180 ~ 180deg
      yaw = filter.getYaw();      // 0 -3 60deg
/*
      // battery voltage averasing  32 averaging 1 ~ 6mS
      int Vadc = 0;
      for (int i = 0; i < AVENUM; i++) {        
        Vadc = Vadc + analogRead(VBAT_READ);    // analogRead() 32uS       
      }
*/     
      //Vadc = Vadc / AVENUM;
      //Vbatt = 2.961 * 2.4 * Vadc / 4096 * 1.0196;     // Vref=2.4, 1/attination=(510e3 + 1e6)/510e3=2.961
                                                      // correction=1.0196(option)
      // convert float data to uint16_t data
      ud.dataBuff16[0] = (roll * 32768) / 180; 
      ud.dataBuff16[1] = (pitch * 32768) / 180; 
      ud.dataBuff16[2] = (yaw * 32768) / 360;       
      ud.dataBuff8[3] = xValue;
      ud.dataBuff8[4] = yValue;
      ud.dataBuff8[5] = BTS1;
      ud.dataBuff8[6] = BTS2;
      ud.dataBuff8[7] = BTS3;

      
      BLEDevice peripheral = BLE.available();
      BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");


      int BTSC1 = digitalRead(BT1);
      int BTSC2 = digitalRead(BT2);
      int BTSC3 = digitalRead(BT3);
      

       if (BTS1!= BTSC1) { BTS1 = BTSC1;
       if (BTS1) { ledCharacteristic.writeValue((byte)0x01);
      } else { ledCharacteristic.writeValue((byte)0x00);}}


       if (BTS2!= BTSC2) { BTS2 = BTSC2;
       if (BTS2) { ledCharacteristic.writeValue((byte)0x03);
      } else { ledCharacteristic.writeValue((byte)0x02);}}


       if (BTS3!= BTSC3) { BTS3 = BTSC3;
       if (BTS3) { ledCharacteristic.writeValue((byte)0x05);
      } else { ledCharacteristic.writeValue((byte)0x04);}}


      xValue = analogRead(VRX_PIN);
      yValue = analogRead(VRY_PIN);
    
      // print data to Serial Monitor on Arduino IDE

      Serial.print(BTS1);
      Serial.print(BTS2);
      Serial.print(BTS3);
      Serial.print("==");
      
      Serial.print(" ");
      Serial.print(xValue);
      Serial.print(" ");
      Serial.print(yValue);
      Serial.print("==");

  
      // for Serial plotter 5~20mS
      Serial.print(yaw);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.print(roll);
      Serial.println();

      // write to Characteristic as byte data        
      dataCharacteristic.writeValue(ud.dataBuff8, dataNum);   // 40~100uS
       
      digitalWrite(LED_RED, HIGH);  // data read and send task indicator OFF
      
    } //while connected
      
    // if disconnected from Central
    digitalWrite(LED_GREEN, HIGH);                // connect indicator OFF
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  } //if connect
    
} //loop()
