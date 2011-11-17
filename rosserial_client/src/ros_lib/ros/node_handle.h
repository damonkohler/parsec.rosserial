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

#include <stdio.h>

#include <std_msgs/Time.h>
#include <rosserial_msgs/TopicInfo.h>
#include <rosserial_msgs/Log.h>
#include <rosserial_msgs/RequestParam.h>

#define SYNC_SECONDS 5

#define MSG_TIMEOUT 20  // 20 now to receive all of message data

#include "msg_receiver.h"
#include "node_output.h"
#include "publisher.h"
#include "rosserial_ids.h"
#include "service_server.h"
#include "subscriber.h"

namespace ros {

  using rosserial_msgs::TopicInfo;

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

  // Node Handle
  template<class Hardware,
           int MAX_SUBSCRIBERS=25,
           int MAX_PUBLISHERS=25,
           int INPUT_SIZE=512,
           int OUTPUT_SIZE=512>
  class NodeHandle_ {
    public:
      NodeHandle_() : node_output_(&hardware_) {}

      Hardware* getHardware() {
        return &hardware_;
      }

      // Start serial, initialize buffers
      void initNode() {
        hardware_.init();
        error_count_ = 0;
        total_receivers_ = 0;
        reset();
      }

      void logdebug(const char* msg) {
        log(rosserial_msgs::Log::DEBUG, msg);
      }

      void loginfo(const char* msg) {
        log(rosserial_msgs::Log::INFO, msg);
      }

      void logwarn(const char* msg) {
        log(rosserial_msgs::Log::WARN, msg);
      }

      void logerror(const char* msg) {
        log(rosserial_msgs::Log::ERROR, msg);
      }

      void logfatal(const char* msg) {
        log(rosserial_msgs::Log::FATAL, msg);
      }

    protected:
      Hardware hardware_;
      NodeOutput<Hardware, OUTPUT_SIZE> node_output_;

      // millis() when the time sync was requested.
      unsigned long time_sync_start_time_;

      // Time time_offset_ from the host in milliseconds.
      unsigned long time_offset_;

      unsigned char message_in[INPUT_SIZE];

      Publisher* publishers[MAX_PUBLISHERS];
      MsgReceiver* receivers[MAX_SUBSCRIBERS];

      // State machine variables for spinOnce
      PacketState state_;
      int remaining_data_bytes_;
      int topic_;
      int data_index_;
      int checksum_;

      int error_count_;
      int total_receivers_;

      // used for syncing the time
      unsigned long last_sync_time;
      unsigned long last_time_sync_time_;
      unsigned long last_msg_timeout_time;

      bool registerReceiver(MsgReceiver* receiver) {
        if (total_receivers_ >= MAX_SUBSCRIBERS) {
          return false;
        }
        receivers[total_receivers_] = receiver;
        receiver->setId(100 + total_receivers_);
        total_receivers_++;
        return true;
      }

      // Reset state
      void reset() {
        state_ = STATE_FIRST_FF;
        remaining_data_bytes_ = 0;
        topic_ = 0;
        data_index_ = 0;
        checksum_ = 0;
      }

    public:
      // This function goes in your loop() function, it handles
      // serial input and callbacks for subscribers.
      virtual void spinOnce() {
        // restart if timed out
        unsigned long current_time = hardware_.time();  // now
        // TODO(damonkohler): Why *2200?
        if ((current_time - last_time_sync_time_) > (SYNC_SECONDS * 2200)) {
          node_output_.setConfigured(false);
        }
        // Reset state if message has timed out.
        if (state_ != STATE_FIRST_FF && current_time > last_msg_timeout_time) {
          reset();
        }

        // while available buffer, read data
        while (true) {
          int inputByte = hardware_.read();
          if (inputByte < 0) {
            // No data available to read.
            break;
          }
          checksum_ += inputByte;
          switch (state_) {
            case STATE_FIRST_FF:
              if (inputByte == 0xff) {
                state_ = STATE_SECOND_FF;
                last_msg_timeout_time = current_time + MSG_TIMEOUT;
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
              } else if (remaining_data_bytes_ <= INPUT_SIZE) {
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
                  requestSyncTime();
                  negotiateTopics();
                  last_sync_time = current_time;
                  last_time_sync_time_ = current_time;
                } else if (topic_ == TopicInfo::ID_TIME) {
                  syncTime(message_in);
                } else if (topic_ == TopicInfo::ID_PARAMETER_REQUEST) {
                  req_param_resp.deserialize(message_in);
                  param_received = true;
                } else if (topic_ >= 100 && topic_ - 100 < MAX_SUBSCRIBERS &&
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

        // occasionally sync time
        // TODO(damonkohler): Why *500?
        if (node_output_.configured() &&
            ((current_time - last_sync_time) > (SYNC_SECONDS * 500))) {
          requestSyncTime();
          last_sync_time = current_time;
        }
      }

      int getErrorCount() const {
        return error_count_;
      }

      // Are we connected to the PC?
      bool connected() {
        return node_output_.configured();
      };

      // Time functions
      void requestSyncTime() {
        // TODO(damonkohler): Why publish an empty message here?
        std_msgs::Time time;
        node_output_.publish(rosserial_msgs::TopicInfo::ID_TIME, &time);
        time_sync_start_time_ = hardware_.time();
      }

      void syncTime(unsigned char* data) {
        time_offset_ = (hardware_.time() - time_sync_start_time_) / 2;
        std_msgs::Time time;
        time.deserialize(data);
        time.data.sec += time_offset_ / 1000;
        time.data.nsec += (time_offset_ % 1000) * 1000000ul;
        last_time_sync_time_ = hardware_.time();
        char message[40];
        snprintf(message, 40, "Time: %lu %lu", time.data.sec, time.data.nsec);
        loginfo(message);
      }

      Time now() const {
        unsigned long now = hardware_.time() + time_offset_;
        Time current_time;
        return current_time.fromSec(now / 1000);
      }

      // Registration
      bool advertise(Publisher& publisher) {
        // TODO(damonkohler): Pull out a publisher registry or keep track of
        // the next available ID.
        for (int i = 0; i < MAX_PUBLISHERS; i++) {
          if (publishers[i] == 0) {  // empty slot
            publishers[i] = &publisher;
            publisher.setId(i + 100 + MAX_SUBSCRIBERS);
            publisher.setNodeOutput(&node_output_);
            return true;
          }
        }
        return false;
      }

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

      void negotiateTopics() {
        node_output_.setConfigured(true);
        rosserial_msgs::TopicInfo topic_info;
        // Slots are allocated sequentially and contiguously. We can break
        // out early.
        for (int i = 0; i < MAX_PUBLISHERS && publishers[i] != 0; i++) {
          topic_info.topic_id = publishers[i]->getId();
          topic_info.topic_name = const_cast<char*>(publishers[i]->getTopicName());
          topic_info.message_type = const_cast<char*>(publishers[i]->getMessageType());
          node_output_.publish(TOPIC_PUBLISHERS, &topic_info);
        }
        for (int i = 0; i < MAX_SUBSCRIBERS && receivers[i] != 0; i++) {
          topic_info.topic_id = receivers[i]->getId();
          topic_info.topic_name = const_cast<char*>(receivers[i]->getTopicName());
          topic_info.message_type = const_cast<char*>(receivers[i]->getMessageType());
          node_output_.publish(TOPIC_SUBSCRIBERS, &topic_info);
        }
      }

    private:
      bool param_received;
      rosserial_msgs::RequestParamResponse req_param_resp;

      void log(char byte, const char* msg) {
        rosserial_msgs::Log l;
        l.level = byte;
        l.msg = const_cast<char*>(msg);
        this->node_output_.publish(rosserial_msgs::TopicInfo::ID_LOG, &l);
      }

      bool requestParam(const char* name, int time_out=1000) {
        param_received = false;
        rosserial_msgs::RequestParamRequest req;
        req.name  = (char*)name;
        node_output_.publish(TopicInfo::ID_PARAMETER_REQUEST, &req);
        int start_time = hardware_.time();
        while(!param_received) {
          spinOnce();
          if (hardware_.time() > start_time + time_out) {
            return false;
          }
        }
        return true;
      }

    public:
      bool getParam(const char* name, int* param, int length=1) {
        if (requestParam(name) && length == req_param_resp.ints_length) {
          for (int i = 0; i < length; i++) {
            param[i] = req_param_resp.ints[i];
          }
          return true;
        }
        return false;
      }

      bool getParam(const char* name, float* param, int length=1) {
        if (requestParam(name) && length == req_param_resp.floats_length) {
          for (int i = 0; i < length; i++) {
            param[i] = req_param_resp.floats[i];
          }
          return true;
        }
        return false;
      }

      bool getParam(const char* name, char** param, int length=1) {
        if (requestParam(name) && length == req_param_resp.strings_length) {
          for (int i = 0; i < length; i++) {
            strcpy(param[i], req_param_resp.strings[i]);
          }
          return true;
        }
        return false;
      }
  };

}  // namespace ros

#endif
