<div align="center">

  <img src="https://user-images.githubusercontent.com/62047147/195847997-97553030-3b79-4643-9f2c-1f04bba6b989.png" alt="logo" width="100" height="auto" />
  <h1>XIAO Controller</h1>
  
  <p>
    Wireless Controller with XIAO nRF52840 Sense
  </p>
  
  
<!-- Badges -->

<a href="https://github.com/cifertech/XIAO-Controller" title="Go to GitHub repo"><img src="https://img.shields.io/static/v1?label=cifertech&message=XIAO-Controller&color=white&logo=github" alt="cifertech - XIAO-Controller"></a>
<a href="https://github.com/cifertech/XIAO-Controller"><img src="https://img.shields.io/github/stars/cifertech/XIAO-Controller?style=social" alt="stars - XIAO-Controller"></a>
<a href="https://github.com/cifertech/XIAO-Controller"><img src="https://img.shields.io/github/forks/cifertech/XIAO-Controller?style=social" alt="forks - XIAO-Controller"></a>
   
<h4>
    <a href="https://twitter.com/cifertech1">TWITTER</a>
  <span> · </span>
    <a href="https://www.instagram.com/cifertech/">INSTAGRAM</a>
  <span> · </span>
    <a href="https://www.youtube.com/c/cifertech">YOUTUBE</a>
  <span> · </span>
    <a href="https://cifertech.net/">WEBSITE</a>
  </h4>
</div>

<br />

<!-- Table of Contents -->
# :notebook_with_decorative_cover: Table of Contents

- [About the Project](#star2-about-the-project)
  * [Pictures](#camera-Pictures)
  * [Features](#dart-features)
- [Getting Started](#toolbox-getting-started)
  * [Schematic](#electric_plug-Schematic)
  * [Installation](#gear-installation)
- [Usage](#eyes-usage)
- [Contributing](#wave-contributing)
- [License](#warning-license)
- [Contact](#handshake-contact)

  

<!-- About the Project -->
## :star2: About the Project
The codes in this repository are for the wireless controller using XIAO nRF52840. I'm using a joystick module and internal LSM6DS3 to send HID commands to the receiver board and control the mouse of the target computer.


<!-- Pictures -->
### :camera: Pictures

<div align="center"> 
  <img src="https://github.com/cifertech/XIAO-Controller/assets/62047147/7e2ff796-3c27-4d46-b08b-25a7b3ea55ea" alt="screenshot" />
</div>


<!-- Features -->
### :dart: Features

- Control mouse via joystick module
- Control mouse via internal LSM6DS3 (roll, pitch, yaw)
- 2 extra buttons for sending commands
- light indicator using Neopixel ws2812b

<!-- Getting Started -->
## 	:toolbox: Getting Started

In this project, I decided to use the Seeed Studio XIAO nRF52840 board. Because a lot of the features I need are built into this board, for example, Bluetooth and gyroscope.

- XIAO nRF52840 Sense
- XIAO nRF52840 
- joystick module
- NeoPixel ws2812b
- Microswitch

<!-- Schematic -->
### :electric_plug: Schematic
Make the connections according to the table and schematic below.

* XIAO nRF52840 and joyStick.

| XIAO nRF52840 | joyStick |  
| ----   | -----|
| A0   | VRX|
| A1   | VRY|
| 4  | SW  |
| 5V  | +5v  |
| GND | GND |


Neopixel Din will be connected to pin 3 of XIAO nRF52840 (Receiver) also the other two Microswitches will be connected to pins 5 and 6.

 
* Complete Schematic

<img src="https://github.com/cifertech/XIAO-Controller/assets/62047147/d60a4bfc-6f49-47f1-b8e1-884d1a07b3bf" alt="screenshot" width="800" height="auto" />


<!-- Usage -->
## :eyes: Usage

after the uploads and we are able to control the mouse using the joystick and also we will be able to control the mouse via the movement of our hand.

<!-- Contributing -->
## :wave: Contributing

<a href="https://github.com/cifertech/XIAO-Controller/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=cifertech/XIAO-Controller" />
</a>


<!-- License -->
## :warning: License

Distributed under the MIT License. See LICENSE.txt for more information.


<!-- Contact -->
## :handshake: Contact

CiferTech - [@twitter](https://twitter.com/cifertech1) - [@instagram](https://www.instagram.com/cifertech/) - CiferTech@gmali.com

Project Link: [https://github.com/cifertech/XIAO-Controller](https://github.com/cifertech/XIAO-Controller)

<!-- Acknowledgments -->
## :gem: Acknowledgements 

 - Attitude monitor using Peripheral:XIAO_BLE_Sence and Central:XIAO_BLE with mbed 2.7.2 and ArduinoBLE [Click Here](https://forum.seeedstudio.com/t/attitude-monitor-using-peripheral-xiao-ble-sence-and-central-xiao-ble-with-mbed-2-7-2-and-arduinoble/266488).
- DAZZLER Light Organ Bracelet [Click Here](https://www.thingiverse.com/thing:377186).
