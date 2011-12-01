// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

package org.ros.rosserial;

import com.google.common.annotations.VisibleForTesting;

import org.apache.commons.logging.Log;

import java.io.IOException;
import java.io.OutputStream;

/**
 * Protocol handler for rosserial.
 * 
 * @author adasta@gmail.com (Adam Stambler)
 */
class DefaultPacketSender implements PacketSender {

  /**
   * Flags for marking beginning of packet transmission.
   */
  private static final byte[] FLAGS = { (byte) 0xff, (byte) 0xff };

  private final Log log;
  private final OutputStream outputStream;

  public DefaultPacketSender(OutputStream outputStream, Log log) {
    this.log = log;
    this.outputStream = outputStream;
  }

  @Override
  public void send(byte[] data) {
    try {
      outputStream.write(FLAGS);
      outputStream.write(data);
      outputStream.write(calculateChecksum(data));
      outputStream.flush();
    } catch (IOException e) {
      log.error("IO error while writing packet: " + BinaryUtils.byteArrayToHexString(data), e);
    }
  }

  @VisibleForTesting
  static byte calculateChecksum(byte[] data) {
    int chk = 0;
    for (int i = 0; i < data.length; i++) {
      chk += 0xff & data[i];
    }
    chk = 255 - chk % 256;
    return (byte) chk;
  }
}