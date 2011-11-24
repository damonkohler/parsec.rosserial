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

import com.google.common.collect.Maps;

import org.ros.message.Message;
import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Map;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
class Proxy {

  /**
   * Node hosting the subscribers and publishers.
   */
  private final Node node;

  /**
   * Topic ID to publisher.
   * 
   * The {@link Publisher}s are parameterized with {@link Object} so that we can
   * handle arbitrary message definition classes (e.g. new style or old style
   * messages).
   */
  private final Map<Integer, Publisher<Object>> publishers;

  /**
   * Topic ID to subscriber.
   */
  private final Map<Integer, Subscriber<?>> subscribers;

  public Proxy(Node node) {
    this.node = node;
    publishers = Maps.newHashMap();
    subscribers = Maps.newHashMap();
  }

  /**
   * Registers a publisher being transmitted over the serial port.
   * 
   * @param topic
   *          the TopicInfo message describing the topic
   */
  public void registerPublisher(int topicId, String topicName, String messageType) {
    Publisher<Object> publisher = node.newPublisher(topicName, messageType);
    getPublishers().put(topicId, publisher);
    node.getLog().info(
        String.format("Registered publisher: %d %s %s", topicId, topicName, messageType));
  }

  /**
   * Registers a subscriber being transmitted over the serial port.
   * 
   * @param topic
   *          the TopicInfo message describing the topic
   */
  public void registerSubscriber(int topicId, String topicName, String messageType,
      MessageListener<?> listener) {
    Subscriber<?> subscriber = node.newSubscriber(topicName, messageType, listener);
    getSubscribers().put(topicId, subscriber);
    node.getLog().info(
        String.format("Registered subscriber: %d %s %s", topicId, topicName, messageType));
  }

  public void publish(int topicId, Message message) {
    getPublishers().get(topicId).publish(message);
  }

  public Map<Integer, Publisher<Object>> getPublishers() {
    return publishers;
  }

  public Map<Integer, Subscriber<?>> getSubscribers() {
    return subscribers;
  }
}
