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

#include <limits.h>
#include <math.h>

namespace ros {

Time::Time() : sec(0), nsec(0) {}

Time::Time(unsigned long sec, unsigned long nsec) : sec(sec), nsec(nsec) {
  normalize();
}

float Time::toSec() const {
  return sec + 1e-9f * nsec;
}

Time Time::fromSec(float seconds) {
  unsigned long sec = floor(seconds);
  unsigned long nsec = round((seconds - sec) * 1e9f);
  return Time(sec, nsec);
}

Time& Time::operator+=(const Duration &rhs) {
  sec += rhs.sec;
  nsec += rhs.nsec;
  normalize();
  return *this;
}

Time& Time::operator-=(const Duration &rhs){
  // rhs.nsec is nonnegative, nsec is unsigned. We move one second from
  // rhs.sec to rhs.nsec to avoid overflow when subtracting from nsec.
  sec += -1 - rhs.sec;
  nsec += 1000000000ul - rhs.nsec;
  normalize();
  return *this;
}

Time Time::operator+(const Duration &rhs) const {
  return Time(sec + rhs.sec, nsec + rhs.nsec);
}

Time Time::operator-(const Duration &rhs) const {
  // Avoid overflow as for operator-=().
  return Time(sec - 1 - rhs.sec, nsec + 1000000000ul - rhs.nsec);
}

Duration Time::operator-(const Time &rhs) const {
  // Avoid overflow in nsec as for operator-=().
  unsigned long unsigned_sec_delta = sec - 1 - rhs.sec;
  // signed long sec_delta = unsigned_sec_delta;  // Two's complement.
  signed long sec_delta = unsigned_sec_delta < LONG_MAX
      ? unsigned_sec_delta
      : -static_cast<signed long>(-unsigned_sec_delta);
  return Duration(sec_delta, nsec + 1000000000ul - rhs.nsec);
}

void Time::normalize() {
  unsigned long sec_part = nsec / 1000000000ul;
  nsec %= 1000000000ul;
  sec += sec_part;
}

}  // namespace ros
