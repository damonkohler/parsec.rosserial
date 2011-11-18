/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
*    contributors may be used to endorse or promote prducts derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICulAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ROS_NODE_HANDLE_H_
#define ROS_NODE_HANDLE_H_

#include "subscriber.h"
#include "service_server.h"

#include "rosserial_msgs/RequestParam.h"

namespace ros {

class Hardware;
class Publisher;
class Time;

enum PacketState {
  STATE_FIRST_FF,
  STATE_SECOND_FF,
  STATE_TOPIC_LOW,  // Waiting for topic ID.
  STATE_TOPIC_HIGH,
  STATE_SIZE_LOW,  // Waiting for message size.
  STATE_SIZE_HIGH,
  STATE_MESSAGE,
  STATE_CHECKSUM,
};

class NodeHandle {
 public:
  NodeHandle(Hardware* hardware);

  Hardware* getHardware();

  // Start serial, initialize buffers.
  void initNode();

  void logdebug(const char* msg);
  void loginfo(const char* msg);
  void logwarn(const char* msg);
  void logerror(const char* msg);
  void logfatal(const char* msg);

  // This function goes in your loop() function, it handles
  // serial input and callbacks for subscribers.
  void spinOnce();

  int getErrorCount() const;

  void requestSyncTime();
  void syncTime(unsigned char* data);
  Time now() const;

  // Registration
  bool advertise(Publisher& publisher);

  // Register a subscriber with the node
  template<typename MsgT>
  bool subscribe(Subscriber<MsgT> &s) {
    return registerReceiver((MsgReceiver*) &s);
  }

  template<typename SrvReq, typename SrvResp>
  bool advertiseService(ServiceServer<SrvReq, SrvResp>& srv) {
    srv.node_output_ = &node_output_;
    return registerReceiver((MsgReceiver*) &srv);
  }

  void negotiateTopics();

  bool getParam(const char* name, int* param, int length=1);
  bool getParam(const char* name, float* param, int length=1);
  bool getParam(const char* name, char** param, int length=1);

  bool connected();

 private:
  static const int kMaxSubscribers = 25;
  static const int kMaxPublishers = 25;
  static const int kInputSize = 512;
  static const int kOuputSize = 512;

  bool connected_;
  bool param_received_;
  rosserial_msgs::RequestParamResponse req_param_resp;

  Hardware* hardware_;
  NodeOutput<kOuputSize> node_output_;

  // millis() when the time sync was requested.
  unsigned long time_sync_start_;
  unsigned long time_sync_end_;

  // Time epoch_time_offset_ from the host in milliseconds.
  unsigned long long epoch_time_offset_;

  unsigned char message_in[kInputSize];

  Publisher* publishers[kMaxPublishers];
  MsgReceiver* receivers[kMaxSubscribers];

  // State machine variables for spinOnce
  PacketState state_;
  int remaining_data_bytes_;
  int topic_;
  int data_index_;
  int checksum_;

  int error_count_;
  int total_receivers_;

  // used for syncing the time
  unsigned long last_msg_timeout_time;

  bool registerReceiver(MsgReceiver* receiver);

  // Reset state
  void reset();

  void log(char byte, const char* msg);
  bool requestParam(const char* name, int time_out=1000);
};

}  // namespace ros

#endif  // ROS_NODE_HANDLE_H_
