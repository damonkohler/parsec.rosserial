/*
 * rosserial::std_msgs::Time Test
 * Publishes the current time.
 */

#include <WProgram.h>

#include "arduino_hardware.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"

#include "std_msgs/Int32.h"

ArduinoHardware hardware;
ros::NodeHandle nh(&hardware);

std_msgs::Int32 test;
ros::Publisher p("my_topic", &test);

int i = 0;

void setup() {
  hardware.init();
  nh.advertise(p);
}

void loop() {
  test.data = i++ % 100;
  p.publish(&test);
  nh.spinOnce();
  delay(10);
}

