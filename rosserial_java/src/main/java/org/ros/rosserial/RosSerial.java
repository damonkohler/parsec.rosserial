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

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.BufferOverflowException;

/**
 * The host endpoint for a rosserial connection.
 * 
 * @author adasta@gmail.com (Adam Stambler)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class RosSerial implements NodeMain {

  private static final boolean DEBUG = false;

  private static final int STREAM_BUFFER_SIZE = 8192;

  /**
   * Output stream for the serial line used for communication.
   */
  private final OutputStream outputStream;

  /**
   * Input stream for the serial line used for communication.
   */
  private final InputStream inputStream;

  private Protocol protocol;

  /**
   * It is not necessary to provide buffered streams. Buffering is handled
   * internally.
   * 
   * @param inputStream
   *          the {@link InputStream} for the connected device
   * @param outputStream
   *          the {@link OutputStream} for the connected device
   */
  public RosSerial(InputStream inputStream, OutputStream outputStream) {
    this.inputStream = new BufferedInputStream(inputStream, STREAM_BUFFER_SIZE);
    this.outputStream = new BufferedOutputStream(outputStream, STREAM_BUFFER_SIZE);
  }

  @Override
  public GraphName getDefaultNodeName() {
    return new GraphName("rosserial");
  }
  
  @Override
  public void onStart(final Node node) {
    PacketSender packetSender = new DefaultPacketSender(outputStream, node.getLog());
    protocol = new Protocol(node, packetSender);
    PacketReceiver packetReceiver = new DefaultPacketReceiver(protocol);
    final PacketBuilder packetBuilder = new PacketBuilder(packetReceiver);
    protocol.start();
    node.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void loop() throws InterruptedException {
        try {
          int inputByte = inputStream.read();
          if (inputByte == -1) {
            // The connection has been closed.
            cancel();
            return;
          }
          packetBuilder.addByte((byte) inputByte);
        } catch (IllegalStateException e) {
          if (DEBUG) {
            node.getLog().error("Protocol error.", e);
          }
        } catch (IOException e) {
          throw new RosRuntimeException(e);
        } catch (BufferOverflowException e) {
          // TODO(damonkohler): Because there is no checksum on the data
          // length, it's possible to try to read too little or too much
          // data for the current packet.
          throw new RosRuntimeException(e);
        }
      }
    });
  }

  @Override
  public void onShutdown(Node node) {
  }

  @Override
  public void onShutdownComplete(Node node) {
    if (protocol != null) {
      protocol.shutdown();
      protocol = null;
    }
    try {
      inputStream.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      outputStream.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
