#define ROSSERIAL_ARDUINO_TCP_WIFI // Required to make the Wifi rosserial work for an ESP32

#include <ESP32Servo.h> 
#include <ESP32Encoder.h>
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <WiFi.h>
#include "PID.h"

#define PI 3.14159265

Servo servo;  // create servo object to control a servo
ESP32Encoder encoder;
PID winchPosPID;

int convertDistToEncReading (float distance);

/***************************************************************************************/
/**************************Wifi stuff***************************************************/
const char* ssid = "test"; // Wifi network name
const char* password =  "abcdefg1"; // Wifi password
IPAddress serverIp(10,42,0,1);      // rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411; // rosserial socket server port - NOT roscore socket!

void connectToNetwork() {
  WiFi.begin(ssid, password);

  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED) { // Wait until Wifi is connected
    Serial.println("Still Connecting...");
    delay(1000);
  }

  Serial.println("Connected!");
}
/***************************************************************************************/

/***************************************************************************************/
/**************************ROS Stuff****************************************************/
float dronePos [2];
float winchPos [3]; 
bool changed;

// Callback for subscriber, receives a 3 element Float32 array
void subdronePos_cb (const std_msgs::Float32MultiArray& data_rec) {
  for (int i=0; i<3; i++) {
    dronePos[i] = data_rec.data[i+4];
    switch (i) {
      case 0:
              Serial.print("X = ");
              break;
      case 1:
              Serial.print("Y = ");
              break;
      case 2:
              Serial.print("Z = "); 
              break;
      default:
              break;
    }
    Serial.println(dronePos[i]);
  }
  changed = 1;
}

// Callback for subscriber, receives a 3 element Float32 array
void subwinchPos_cb (const std_msgs::Float32MultiArray& data_rec) {
  for (int i=0; i<2; i++) {
    winchPos[i] = data_rec.data[i];
  }
  changed = 1;
}


ros::Subscriber<std_msgs::Float32MultiArray> datadrone_sub("loco_data_drone", &subdronePos_cb); 
ros::Subscriber<std_msgs::Float32MultiArray> datawinch_sub("winchPos_Data", &subwinchPos_cb);

ros::NodeHandle nh;
/***************************************************************************************/

void setup() {
  Serial.begin(115200);
  
  // Attach servo to pin 15 with pulse range 1000us to 2000us
  servo.attach(15, 1000, 2000); // servo of 90 is complete stop

  // Attach quadrature encoders to pins 34 and 35
  encoder.attachHalfQuad(34, 35); // 200ppr
  encoder.setCount(0);// set starting count value
  
  // Connect to the Wifi
  connectToNetwork(); 

  // can't do for real thing
  for (int i=0; i<2; i++) {
    winchPos[i] = 0;
    dronePos[i] = 0;
  }

  // Start up ROS
  nh.initNode();
  nh.getHardware()->setConnection(serverIp, serverPort); // Set the rosserial socket server info - uncomment for Wifi comms, comment for USB
  nh.subscribe(datadrone_sub);
//  nh.subscribe(datawinch_sub);

  // Set constants for PID
  winchPosPID.set_PID_constants(0.015, 0.0, 0.000000125);
  int pos = convertDistToEncReading(0); // Choose an initial desired value on startup, maybe wait for ROS data so we try to set to the correct value
  winchPosPID.set_desired_value(pos);
}

void loop() {
  static unsigned long start_time = micros();
  static unsigned long old_loop_time = micros();
  static float time_diff = 0.0;
  static int current_enc_reading = encoder.getCount();
  static float pid_output;
  static const uint16_t loop_time = 10; // Set desired loop time in ms
  static int enc_des = 0;

  // Spin ROS and collect new data
  nh.spinOnce();

  // If new data available
  if (changed) {
    // calc new distance using dronePos and winchPos and set as new desired value
    float dist = sqrt( sq(dronePos[0]- winchPos[0]) + sq(dronePos[1]- winchPos[1]) + sq(dronePos[2]- winchPos[2]) );
    Serial.println(dist);
    enc_des = convertDistToEncReading(dist);
    winchPosPID.set_desired_value(enc_des);
    changed = 0;
  }

  

  time_diff = (micros() - old_loop_time)/1000000.0;

  // Read Encoder
  current_enc_reading = encoder.getCount();

  Serial.print("Desired Encoder: "); Serial.print(enc_des);
  Serial.print("\tEncoder reading: "); Serial.println(current_enc_reading);

  // Calculate winch PID output based on current reading, time difference and desired encoder value
  pid_output = winchPosPID.compute_PID(current_enc_reading, time_diff); // current val, time_diff
  pid_output = pid_output*500 + 1500; // Scale and place at centre (so 0 error means 1500 pulse, ie. no movement)
  pid_output = constrain(pid_output, 1000, 2000);

//  Serial.print("PID output reading: "); Serial.println(pid_output);

  // Output the pidOutput to the motor
  servo.writeMicroseconds(pid_output); //write between 1000-2000

  // Ensure loop runs at desired speed
  time_diff = (micros() - old_loop_time)/1000.0;
  if (time_diff < loop_time) {
    delay (loop_time - (uint16_t)(time_diff));
  }
  old_loop_time = micros();  
}

// Use to convert a distance in metres to a desired encoder value.
int convertDistToEncReading (float distance) {
  static float drumDiam = 0.062; // Diameter of drum in metres
  static float circ = PI*drumDiam; // calculate circumference of drum
  static int ppr = 200; // pulses per revolution of encoder

  int encToReturn = (int)((distance/circ)*200.0);
  return encToReturn;
}
