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

#include "ros/duration.h"

#include <math.h>

namespace ros {

Duration::Duration() : sec(0), nsec(0) {}

Duration::Duration(long sec, long nsec) : sec(sec), nsec(nsec) {
  normalize();
}

double Duration::toSec() const {
  return (double) sec + 1e-9 * (double) nsec;
}

Duration Duration::fromSec(double seconds) {
  long sec = floor(seconds);
  long nsec = round((seconds - sec) * 1e9);
  return Duration(sec, nsec);
}

Duration Duration::fromMillis(long millis) {
  return Duration(millis / 1000, (millis % 1000) * 1000000l);
}

Duration& Duration::operator+=(const Duration &rhs) {
  sec += rhs.sec;
  nsec += rhs.nsec;
  normalize();
  return *this;
}

Duration& Duration::operator-=(const Duration &rhs) {
  sec -= rhs.sec;
  nsec -= rhs.nsec;
  normalize();
  return *this;
}

Duration& Duration::operator*=(double scale) {
  sec *= scale;
  nsec *= scale;
  normalize();
  return *this;
}

void Duration::normalize() {
  while (nsec > 1000000000l) {
    nsec -= 1000000000l;
    ++sec;
  }
  while (nsec < 0) {
    nsec += 1000000000l;
    --sec;
  }
}

}  // namespace ros
