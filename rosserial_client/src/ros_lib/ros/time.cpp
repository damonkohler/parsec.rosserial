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

#include "ros/time.h"

#include <math.h>

namespace ros {

Time::Time() : sec(0), nsec(0) {}

Time::Time(unsigned long sec, unsigned long nsec) : sec(sec), nsec(nsec) {
  normalize();
}

double Time::toSec() const {
  return (double) sec + 1e-9 * (double) nsec;
}

Time Time::fromSec(double seconds) {
  unsigned long sec = floor(seconds);
  unsigned long nsec = round((seconds - sec) * 1e9);
  return Time(sec, nsec);
}

Time& Time::operator+=(const Duration &rhs) {
  sec += rhs.sec;
  nsec += rhs.nsec;
  normalize();
  return *this;
}

Time& Time::operator-=(const Duration &rhs){
  sec += -1 - rhs.sec;
  nsec += 1000000000ul - rhs.nsec;
  normalize();
  return *this;
}

Time Time::operator+(const Duration &rhs) const {
  return Time(sec + rhs.sec, nsec + rhs.nsec);
}

void Time::normalize() {
  unsigned long sec_part = nsec / 1000000000ul;
  nsec %= 1000000000ul;
  sec += sec_part;
}

}  // namespace ros
