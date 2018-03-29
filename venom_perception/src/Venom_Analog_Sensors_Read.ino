#include <ros.h>
#include <std_msgs/String.h>

//set ros communication:

//init ros handle
ros::NodeHandle nh;

//init a publisher with topic name 'analogOut'
//&str_msg is a reference to the message instance to be used for publishing.
std_msgs::String str_msg;
ros::Publisher analogOut("analogOut", &str_msg);

//currently sending hello world. Should publish the sensor measuring
char hello[13] = "hello world!";

/*
 Sharp GP2Y0A710K0F infrared proximity sensor (#28999)
 Collects an average of five readings from the sensor and displays the
 resulting value about every half second.
 
 See the Sharp_GP2Y0A710KOF_Simple demonstration for additional usage notes,
 including connection diagram.
 
 This example code is for the Arduino Uno and direct compatible boards, using the
 Arduino 1.0 or later IDE software. It has not been tested, nor designed for, other 
 Arduino boards, including the Arduino Due.

*/
//init sensors and their distances
const int ir1 = A1;          // Connect sensor to analog pin A1
int distance1 = 0;  
const int ir2 = A0;          // Connect sensor to analog pin A0
int distance2 = 0;
const int ir3 = A2;          // Connect sensor to analog pin A2
int distance3 = 0;

void setup() {
  //ROS:
  //initialize your ROS node handle
  nh.initNode();
  //advertise any topics being published
  nh.advertise(analogOut);
  //Sensors:
  Serial.begin(9600);            // Use Serial Monitor window
}

void loop() {
  //ROS:

  //publish hello world
  str_msg.data = hello;
  analogOut.publish( &str_msg );
  //spingOnce is where all of the ROS communication callbacks are handled.
  nh.spinOnce();
  delay(1000);

  //Sensors:
  Serial.println(ir1Read(), DEC); // Call irRead function to read sensor
  //Serial.println(ir2Read(), DEC); // Print value in Serial Monitor
  //Serial.println(ir3Read(), DEC);
           
  delay(250);                    // Wait another 1/4 second for the next read
                                 // (Note: Because of delays built into the
                                 //   irRead function the display of values will
                                 //   be slower than in the SharpGP2Y0A21_Simple
                                 //   sketch
}

// Take multiple readings, and average them out to reduce false readings
int ir1Read() {
  int averaging = 0;             //  Holds value to average readings
  
  // Get a sampling of 5 readings from sensor
  for (int i=0; i<5; i++) {
    distance1 = analogRead(ir1);
    averaging = averaging + distance1;
    delay(80);      // Wait 55 ms between each read
                    // According to datasheet time between each read
                    //  is -38ms +/- 10ms. Waiting 55 ms assures each
                    //  read is from a different sample
  }
  distance1 = averaging / 5;      // Average out readings
  return(distance1);              // Return value
}

// Take multiple readings, and average them out to reduce false readings
int ir2Read() {
  int averaging = 0;             //  Holds value to average readings
  
  // Get a sampling of 5 readings from sensor
  for (int i=0; i<5; i++) {
    distance1 = analogRead(ir2);
    averaging = averaging + distance2;
    delay(80);      // Wait 55 ms between each read
                    // According to datasheet time between each read
                    //  is -38ms +/- 10ms. Waiting 55 ms assures each
                    //  read is from a different sample
  }
  distance2 = averaging / 5;      // Average out readings
  return(distance2);              // Return value
}

// Take multiple readings, and average them out to reduce false readings
int ir3Read() {
  int averaging = 0;             //  Holds value to average readings
  
  // Get a sampling of 5 readings from sensor
  for (int i=0; i<5; i++) {
    distance3 = analogRead(ir3);
    averaging = averaging + distance3;
    delay(80);      // Wait 55 ms between each read
                    // According to datasheet time between each read
                    //  is -38ms +/- 10ms. Waiting 55 ms assures each
                    //  read is from a different sample
  }
  distance1 = averaging / 5;      // Average out readings
  return(distance3);              // Return value
}

