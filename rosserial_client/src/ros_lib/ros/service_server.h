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
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *  copyright notice, this list of conditions and the following
 *  disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *  contributors may be used to endorse or promote prducts derived
 *  from this software without specific prior written permission.
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

#ifndef ROS_SERVICE_SERVER_H_
#define ROS_SERVICE_SERVER_H_

#include "node_output.h"

namespace ros {

  template<typename SrvRequest, typename SrvResponse>
  class ServiceServer : MsgReceiver {
    public:
      typedef void(*CallbackT)(const SrvRequest&, SrvResponse&);

      ServiceServer(const char* topic_name, CallbackT callback) {
        topic_name_ = topic_name;
        callback_ = callback;
      }

      virtual ~ServiceServer() {}

      virtual void receive(unsigned char* data) {
        req.deserialize(data);
        callback_(req, resp);
        node_ouput_->publish(id_, &resp);
      }

      virtual const char* getMessageType() {
        return req.getType();
      }

      SrvRequest req;
      SrvResponse resp;

    private:
      CallbackT callback_;
      NodeOutput_* node_ouput_;

      ServiceServer(const ServiceServer&);
      void operator=(const ServiceServer&);
  };

}  // namespace ros

#endif
