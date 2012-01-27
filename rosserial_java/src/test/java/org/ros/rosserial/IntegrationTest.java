/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.rosserial;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.RosCore;
import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.Holder;
import org.ros.message.MessageListener;
import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.namespace.GraphName;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class IntegrationTest {

  private RosCore rosCore;
  private NodeConfiguration nodeConfiguration;
  private NodeMainExecutor nodeMainExecutor;

  @Before
  public void setUp() throws InterruptedException {
    rosCore = RosCore.newPrivate();
    rosCore.start();
    assertTrue(rosCore.awaitStart(1, TimeUnit.SECONDS));
    nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
    nodeConfiguration = NodeConfiguration.newPrivate(rosCore.getUri());
  }

  @After
  public void tearDown() {
    nodeMainExecutor.shutdown();
    rosCore.shutdown();
  }

  @Test
  public void testSubscribeToSerialPublisher() throws IOException, InterruptedException {
    final CountDownLatch latch = new CountDownLatch(1);
    final Holder<Node> holder = Holder.newEmpty();
    nodeMainExecutor.execute(new NodeMain() {
      @Override
      public void onStart(Node node) {
        holder.set(node);
        Subscriber<org.ros.message.std_msgs.String> subscriber =
            node.newSubscriber("hello_world", "std_msgs/String");
        subscriber.addMessageListener(new MessageListener<org.ros.message.std_msgs.String>() {
          @Override
          public void onNewMessage(org.ros.message.std_msgs.String message) {
            assertEquals("Hello, world!", message.data);
            latch.countDown();
          }
        });
      }

      @Override
      public GraphName getDefaultNodeName() {
        return new GraphName("node");
      }

      @Override
      public void onShutdown(Node node) {
      }

      @Override
      public void onShutdownComplete(Node node) {
      }
    }, nodeConfiguration);

    // Create client (e.g. Arduino) and host (e.g. tablet) streams.
    PipedInputStream clientInputStream = new PipedInputStream();
    PipedOutputStream hostOutputStream = new PipedOutputStream();
    clientInputStream.connect(hostOutputStream);

    PipedInputStream hostInputStream = new PipedInputStream();
    PipedOutputStream clientOutputStream = new PipedOutputStream();
    hostInputStream.connect(clientOutputStream);

    RosSerial rosSerial = new RosSerial(new BufferedInputStream(hostInputStream), hostOutputStream);
    nodeConfiguration.setNodeName("rosserial");
    nodeMainExecutor.execute(rosSerial, nodeConfiguration);

    // Topic negotiation request.
    byte[] expectedTopicNegotiationBuffer =
        new byte[] { (byte) 0xFF, (byte) 0xFF, 0, 0, 0, 0, (byte) 0xFF };
    byte[] topicNegotiationBuffer = new byte[expectedTopicNegotiationBuffer.length];
    clientInputStream.read(topicNegotiationBuffer);
    assertArrayEquals(expectedTopicNegotiationBuffer, topicNegotiationBuffer);

    final PacketSender packetSender =
        new DefaultPacketSender(clientOutputStream, holder.get().getLog());

    // Topic negotiation response.
    TopicInfo topicInfo = new TopicInfo();
    topicInfo.message_type = "std_msgs/String";
    topicInfo.topic_id = 101;
    topicInfo.topic_name = "hello_world";
    byte[] data = Protocol.constructMessage(0, topicInfo);
    packetSender.send(data);

    // Publish hello world over serial continuously.
    nodeMainExecutor.getScheduledExecutorService().execute(new CancellableLoop() {
      @Override
      public void loop() throws InterruptedException {
        org.ros.message.std_msgs.String helloWorld = new org.ros.message.std_msgs.String();
        helloWorld.data = "Hello, world!";
        byte[] data = Protocol.constructMessage(101, helloWorld);
        packetSender.send(data);
        Thread.sleep(500);
      };
    });
    assertTrue(latch.await(5, TimeUnit.SECONDS));
  }
}
