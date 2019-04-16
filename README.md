# SmartWinch-ESP32
Code to run a "smart winch", autonomously reeling and unreeling to keep the end of the tether a set length away. Position of winch and end of tether is received over ROS through wifi.

Code runs on an ESP32 using the Arduino IDE, requires ESP32Servo, ESP32Encoder, and ROS to be installed. 

Note, to get ROS to work over wifi with the version of rosserial I had, I had to modify ros.h and ArduinoTcpHardware.h to include the correct Wifi.h for ESP32, but the newer versions of rosserial seems to have made this change since.
"#define ROSSERIAL_ARDUINO_TCP_WIFI" isn't really needed any more as ESP32 should be defined, but this may be needed on other boards to get wifi to work.
