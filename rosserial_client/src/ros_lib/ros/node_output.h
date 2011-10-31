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
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_NODE_OUTPUT_H_
#define ROS_NODE_OUTPUT_H_

#include "msg.h"

namespace ros {

   // This class is responsible for controlling the node ouput.
   // It it is the object that is passed to Publishers and services
  class NodeOutput_ {
    public:
      // Publishes the provided message and returns the number of bytes
      // written.
      virtual int publish(int id, Msg* msg) = 0;
  };

  template<class Hardware, int BUFFER_SIZE>
  class NodeOutput : public NodeOutput_ {
    public:
      NodeOutput(Hardware* hardware) : hardware_(hardware) {}

      void setConfigured(bool configured) {
        configured_ = configured;
      }

      bool configured() {
        return configured_;
      }

      virtual int publish(int id, Msg* msg) {
        if (!configured_) {
          return 0;
        }

        // TODO(damonkohler): The serialization should check that we don't
        // overflow our buffer.
        int length = msg->serialize(message_out + 6);
        if (length + 7 > BUFFER_SIZE) {
          // It would be better to crash horribly. That will probably happen anyway though...
          return 0;
        }

        // Build the header
        // Sync flags
        message_out[0] = 0xff;
        message_out[1] = 0xff;
        // Topic ID
        message_out[2] = (unsigned char) id & 255;
        message_out[3] = (unsigned char) id >> 8;
        // Data length
        message_out[4] = (unsigned char) length & 255;
        message_out[5] = ((unsigned char) length >> 8);

        // calculate checksum
        int chk = 0;
        for (int i = 2; i < length + 6; i++) {
          chk += message_out[i];
        }
        length += 6;  // Include the header length.
        message_out[length++] = 255 - (chk % 256);  // Add checksum byte and increase length.
        hardware_->write(message_out, length);
        return length;
      }

    private:
      NodeOutput(const NodeOutput&);
      void operator=(const NodeOutput&);

      Hardware* hardware_;
      bool configured_;
      unsigned char message_out[BUFFER_SIZE];
  };

}  // namespace ros

#endif
