/*
 * Echoes float64 messages.
 */

#include <WProgram.h>

#include "arduino_hardware.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "std_msgs/Float64.h"

ArduinoHardware hardware;
ros::NodeHandle node_handle(&hardware);

std_msgs::Float64 publisher_message;
ros::Publisher publisher("~out", &publisher_message);

void callback(const std_msgs::Float64& incoming_message) {
  char message[40];
  sprintf(message, "Heard: %d", (int) incoming_message.data);
  node_handle.loginfo(message);
  publisher_message.data = incoming_message.data;
  publisher.publish(&publisher_message);
}

ros::Subscriber<std_msgs::Float64> subscriber(
    "~in", &callback);

void setup() {
  hardware.init();
  node_handle.subscribe(subscriber);
  node_handle.advertise(publisher);
}

void loop() {
  node_handle.spinOnce();
  delay(10);
}
