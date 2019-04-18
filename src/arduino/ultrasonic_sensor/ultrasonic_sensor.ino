/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int echoPin = A4;
int trigPin = A5;

ros::NodeHandle  nh;


sensor_msgs::Range msg;

ros::Publisher chatter("ultrasonic/raw", &msg);

void setup()
{
  Serial.begin(9600);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
}

double getSonarReadingMillimeters()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  double duration_us = pulseIn(echoPin, HIGH);
  double distance_mm = (duration_us / 58.0) * 10;
  return distance_mm;
}

void loop()
{
  //long us = getSonarReadingMillimeters(); //Get distance from wall with ultrasonic sensor
  double us = getSonarReadingMillimeters(); //Get distance from wall with ultrasonic sensor
  Serial.println(us/1000);
  delay(20);
  us = us/1000;
  
  msg.range = us;
  msg.min_range = 0.02;
  msg.max_range = 4;
  msg.field_of_view = 0.785;
  msg.header.frame_id = "ultrasonic";
  chatter.publish(&msg);
  nh.spinOnce();
  delay(1000);
}
