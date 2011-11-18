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

#include "node_handle.h"

#include <stdio.h>

#include "msg_receiver.h"
#include "node_output.h"
#include "publisher.h"
#include "rosserial_ids.h"
#include "service_server.h"
#include "subscriber.h"
#include "hardware.h"

#include "std_msgs/Time.h"
#include "rosserial_msgs/TopicInfo.h"
#include "rosserial_msgs/Log.h"
#include "rosserial_msgs/RequestParam.h"

#define SYNC_PERIOD 5  // Synchronize clocks every n seconds.
#define MSG_TIMEOUT 20  // Message must arrive within n milliseconds.

namespace ros {

using rosserial_msgs::TopicInfo;

NodeHandle::NodeHandle(Hardware* hardware)
    : hardware_(hardware),
      node_output_(hardware),
      connected_(false),
      param_received_(false),
      time_sync_start_(0),
      time_sync_end_(0),
      state_(STATE_FIRST_FF),
      remaining_data_bytes_(0),
      topic_(0),
      data_index_(0),
      checksum_(0),
      error_count_(0),
      total_receivers_(0),
      last_message_time_(0) {}

Hardware* NodeHandle::getHardware() {
  return hardware_;
}

void NodeHandle::initNode() {
  hardware_->init();
  error_count_ = 0;
  total_receivers_ = 0;
  connected_ = false;
  reset();
}

void NodeHandle::logdebug(const char* msg) {
  log(rosserial_msgs::Log::DEBUG, msg);
}

void NodeHandle::loginfo(const char* msg) {
  log(rosserial_msgs::Log::INFO, msg);
}

void NodeHandle::logwarn(const char* msg) {
  log(rosserial_msgs::Log::WARN, msg);
}

void NodeHandle::logerror(const char* msg) {
  log(rosserial_msgs::Log::ERROR, msg);
}

void NodeHandle::logfatal(const char* msg) {
  log(rosserial_msgs::Log::FATAL, msg);
}

bool NodeHandle::registerReceiver(MsgReceiver* receiver) {
  if (total_receivers_ >= kMaxSubscribers) {
    return false;
  }
  receivers[total_receivers_] = receiver;
  receiver->setId(100 + total_receivers_);
  total_receivers_++;
  return true;
}

void NodeHandle::reset() {
  state_ = STATE_FIRST_FF;
  remaining_data_bytes_ = 0;
  topic_ = 0;
  data_index_ = 0;
  checksum_ = 0;
  last_message_time_ = 0;
}

void NodeHandle::spinOnce() {
  unsigned long current_time = hardware_->time();

  if (last_message_time_ > 0 &&
      current_time > last_message_time_ + MSG_TIMEOUT) {
    reset();
  }

  while (true) {
    int inputByte = hardware_->read();
    if (inputByte < 0) {
      break;
    }
    checksum_ += inputByte;
    switch (state_) {
      case STATE_FIRST_FF:
        if (inputByte == 0xff) {
          state_ = STATE_SECOND_FF;
          last_message_time_ = current_time;
        } else {
          reset();
        }
        break;
      case STATE_SECOND_FF:
        if (inputByte == 0xff) {
          state_ = STATE_TOPIC_LOW;
        } else {
          reset();
        }
        break;
      case STATE_TOPIC_LOW:
        topic_ = inputByte;
        checksum_ = inputByte;  // This is the first byte to be included in the checksum.
        state_ = STATE_TOPIC_HIGH;
        break;
      case STATE_TOPIC_HIGH:
        topic_ += inputByte << 8;
        state_ = STATE_SIZE_LOW;
        break;
      case STATE_SIZE_LOW:
        remaining_data_bytes_ = inputByte;
        state_ = STATE_SIZE_HIGH;
        break;
      case STATE_SIZE_HIGH:  // top half of message size
        remaining_data_bytes_ += inputByte << 8;
        if (remaining_data_bytes_ == 0) {
          state_ = STATE_CHECKSUM;
        } else if (remaining_data_bytes_ <= kInputSize) {
          state_ = STATE_MESSAGE;
        } else {
          // Protect against buffer overflow.
          reset();
          ++error_count_;
        }
        break;
      case STATE_MESSAGE:  // message data being received
        message_in[data_index_++] = inputByte;
        remaining_data_bytes_--;
        if (remaining_data_bytes_ == 0) {  // is message complete? if so, checksum
          state_ = STATE_CHECKSUM;
        }
        break;
      case STATE_CHECKSUM:  // do checksum
        if ((checksum_ % 256) == 255) {
          if (topic_ == TOPIC_NEGOTIATION) {
            requestTimeSync();
            negotiateTopics();
            connected_ = true;
          } else if (topic_ == TopicInfo::ID_TIME) {
            completeTimeSync(message_in);
          } else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST) {
            req_param_resp.deserialize(message_in);
            param_received_ = true;
          } else if (topic_ >= 100 && topic_ - 100 < kMaxSubscribers &&
                     receivers[topic_ - 100] != 0) {
            receivers[topic_ - 100]->receive(message_in);
          } else {
            ++error_count_;
          }
        }
        reset();
        break;
      default:;
        // TODO(damonkohler): Crash?
    }
  }

  // Sync time every SYNC_PERIOD seconds.
  if (current_time - time_sync_end_ > SYNC_PERIOD * 1000) {
    requestTimeSync();
  }
}

int NodeHandle::getErrorCount() const {
  return error_count_;
}

void NodeHandle::requestTimeSync() {
  if (time_sync_start_ > 0) {
    // A time sync request is already in flight.
    return;
  }
  // TODO(damonkohler): Why publish an empty message here?
  std_msgs::Time time;
  node_output_.publish(rosserial_msgs::TopicInfo::ID_TIME, &time);
  time_sync_start_ = hardware_->time();
}

void NodeHandle::completeTimeSync(unsigned char* data) {
  // TODO(damonkohler): Use micros() for higher precision?
  time_sync_end_ = hardware_->time();
  unsigned long offset = (time_sync_end_ - time_sync_start_) / 2;
  std_msgs::Time time;
  time.deserialize(data);
  sync_time_ = time.data;
  sync_time_.sec += offset / 1000;
  sync_time_.nsec += (offset % 1000) * 1000000ul;
  time_sync_start_ = 0;
  char message[40];
  snprintf(message, 40, "Time: %lu %lu", sync_time_.sec, sync_time_.nsec);
  logdebug(message);
}

Time NodeHandle::now() const {
  unsigned long offset = hardware_->time() - time_sync_end_;
  Time time;
  time.sec = sync_time_.sec + offset / 1000;
  time.nsec = sync_time_.nsec + (offset % 1000) * 1000000ul;
  normalizeSecNSec(time.sec, time.nsec);
  return time;
}

// Registration
bool NodeHandle::advertise(Publisher& publisher) {
  // TODO(damonkohler): Pull out a publisher registry or keep track of
  // the next available ID.
  for (int i = 0; i < kMaxPublishers; i++) {
    if (publishers[i] == 0) {  // empty slot
      publishers[i] = &publisher;
      publisher.setId(i + 100 + kMaxSubscribers);
      publisher.setNodeOutput(&node_output_);
      return true;
    }
  }
  return false;
}

void NodeHandle::negotiateTopics() {
  rosserial_msgs::TopicInfo topic_info;
  // Slots are allocated sequentially and contiguously. We can break
  // out early.
  for (int i = 0; i < kMaxPublishers && publishers[i] != 0; i++) {
    topic_info.topic_id = publishers[i]->getId();
    topic_info.topic_name = const_cast<char*>(publishers[i]->getTopicName());
    topic_info.message_type = const_cast<char*>(publishers[i]->getMessageType());
    node_output_.publish(TOPIC_PUBLISHERS, &topic_info);
  }
  for (int i = 0; i < kMaxSubscribers && receivers[i] != 0; i++) {
    topic_info.topic_id = receivers[i]->getId();
    topic_info.topic_name = const_cast<char*>(receivers[i]->getTopicName());
    topic_info.message_type = const_cast<char*>(receivers[i]->getMessageType());
    node_output_.publish(TOPIC_SUBSCRIBERS, &topic_info);
  }
}

void NodeHandle::log(char byte, const char* msg) {
  rosserial_msgs::Log l;
  l.level = byte;
  l.msg = const_cast<char*>(msg);
  this->node_output_.publish(rosserial_msgs::TopicInfo::ID_LOG, &l);
}

bool NodeHandle::requestParam(const char* name, int time_out) {
  param_received_ = false;
  rosserial_msgs::RequestParamRequest req;
  req.name  = (char*)name;
  node_output_.publish(TopicInfo::ID_PARAMETER_REQUEST, &req);
  int start_time = hardware_->time();
  while(!param_received_) {
    spinOnce();
    if (hardware_->time() > start_time + time_out) {
      return false;
    }
  }
  return true;
}

bool NodeHandle::getParam(const char* name, int* param, int length) {
  if (requestParam(name) && length == req_param_resp.ints_length) {
    for (int i = 0; i < length; i++) {
      param[i] = req_param_resp.ints[i];
    }
    return true;
  }
  return false;
}

bool NodeHandle::getParam(const char* name, float* param, int length) {
  if (requestParam(name) && length == req_param_resp.floats_length) {
    for (int i = 0; i < length; i++) {
      param[i] = req_param_resp.floats[i];
    }
    return true;
  }
  return false;
}

bool NodeHandle::getParam(const char* name, char** param, int length) {
  if (requestParam(name) && length == req_param_resp.strings_length) {
    for (int i = 0; i < length; i++) {
      strcpy(param[i], req_param_resp.strings[i]);
    }
    return true;
  }
  return false;
}

bool NodeHandle::connected() {
  return connected_;
}

}  // namespace ros
