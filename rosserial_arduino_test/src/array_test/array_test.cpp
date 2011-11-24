/*
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum
 */

#include <WProgram.h>

#include "arduino_hardware.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

ArduinoHardware hardware;
ros::NodeHandle node_handle(&hardware);

geometry_msgs::Pose sum_msg;
ros::Publisher publisher("sum", &sum_msg);

void callback(const geometry_msgs::PoseArray& msg) {
  sum_msg.position.x = 0;
  sum_msg.position.y = 0;
  sum_msg.position.z = 0;
  char message[40];
  sprintf(message, "%d %d %d %d",
      msg.poses_length,
      (int) msg.poses[0].position.x,
      (int) msg.poses[0].position.y,
      (int) msg.poses[0].position.z);
  node_handle.loginfo(message);
  for(int i = 0; i < msg.poses_length; i++) {
    sum_msg.position.x += msg.poses[i].position.x;
    sum_msg.position.y += msg.poses[i].position.y;
    sum_msg.position.z += msg.poses[i].position.z;
  }
  publisher.publish(&sum_msg);
}

ros::Subscriber<geometry_msgs::PoseArray> subscriber(
    "poses", &callback);

void setup() {
  hardware.init();
  node_handle.subscribe(subscriber);
  node_handle.advertise(publisher);
}

void loop() {
  node_handle.spinOnce();
  delay(10);
}

