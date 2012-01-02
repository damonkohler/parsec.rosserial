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

import org.ros.exception.RosRuntimeException;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class PacketBuilder {

  private static final boolean DEBUG = false;

  private static final int HEADER_BUFFER_SIZE = 4;
  private static final int DATA_BUFFER_SIZE = 512;

  private final PacketReceiver packetReceiver;
  private final ByteBuffer header;
  private final ByteBuffer data;

  private PacketState packetState;
  private short topicId;
  private short dataLength;

  /**
   * Used for debugging output only.
   */
  private int byteNumber;

  public PacketBuilder(PacketReceiver packetReceiver) {
    this.packetReceiver = packetReceiver;
    header = ByteBuffer.allocate(HEADER_BUFFER_SIZE);
    header.order(ByteOrder.LITTLE_ENDIAN);
    data = ByteBuffer.allocate(DATA_BUFFER_SIZE);
    data.order(ByteOrder.LITTLE_ENDIAN);
    reset();
  }

  public void addByte(byte inputByte) {
    if (DEBUG) {
      System.out.println(String.format("%8s (byte %3d): %x", packetState.name(), byteNumber,
          inputByte));
      byteNumber++;
    }
    switch (packetState) {
      case FLAGA:
        if (inputByte != (byte) 0xff) {
          reset();
          throw new IllegalStateException("First flag byte missing.");
        }
        packetState = PacketState.FLAGB;
        break;
      case FLAGB:
        if (inputByte != (byte) 0xff) {
          reset();
          throw new IllegalStateException("Second flag byte missing.");
        }
        packetState = PacketState.HEADER;
        break;
      case HEADER:
        header.put(inputByte);
        if (header.position() == 4) {
          header.flip();
          topicId = header.getShort();
          dataLength = header.getShort();
          if (DEBUG) {
            System.out.println("Topic ID: " + topicId);
            System.out.println("Data length: " + dataLength);
          }
          if (dataLength > DATA_BUFFER_SIZE) {
            reset();
            throw new IllegalStateException("Data size exceeds maximum.");
          }
          if (dataLength > 0) {
            packetState = PacketState.DATA;
          } else {
            packetState = PacketState.CHECKSUM;
          }
        }
        break;
      case DATA:
        data.put(inputByte);
        if (data.position() == dataLength) {
          packetState = PacketState.CHECKSUM;
        }
        break;
      case CHECKSUM:
        int checksum = 0;
        for (int i = 0; i < HEADER_BUFFER_SIZE; i++) {
          checksum += header.get(i);
        }
        for (int i = 0; i < dataLength; i++) {
          checksum += data.get(i);
        }
        checksum = 255 - (checksum % 256);
        if ((byte) checksum != inputByte) {
          reset();
          throw new IllegalStateException(String.format("Invalid checksum: %x != %x", checksum,
              inputByte));
        }
        byte buffer[] = new byte[dataLength];
        data.flip();
        data.get(buffer);
        packetReceiver.receive(topicId, buffer);
        reset();
        break;
      default:
        throw new RosRuntimeException("Unknown packet state.");
    }
  }

  /**
   * Reset packet parsing state machine.
   */
  private void reset() {
    header.clear();
    data.clear();
    dataLength = 0;
    byteNumber = 0;
    packetState = PacketState.FLAGA;
  }

  @VisibleForTesting
  PacketState getPacketState() {
    return packetState;
  }
}
