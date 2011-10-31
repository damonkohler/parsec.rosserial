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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Map;

import com.google.common.collect.Maps;

import org.ros.message.Duration;
import org.ros.message.Message;
import org.ros.message.MessageDeserializer;
import org.ros.message.Time;
import org.ros.message.rosserial_msgs.Log;
import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.node.Node;

/**
 * Protocol handler for rosserial.
 * 
 * @author adasta@gmail.com (Adam Stambler)
 */
class Protocol {

	private static final boolean DEBUG = false;

	// All IDS greater than 100 are publishers or subscribers.
	static final int TOPIC_PUBLISHERS = 0;
	static final int TOPIC_SUBSCRIBERS = 1;
	static final int TOPIC_TIME = 10;

	// Topic negotiation will be retried after the sync timeout. The Arduino
	// implementation of rosserial tries to sync every 5 seconds.
	static final int SYNC_TIMEOUT = 10000; // ms

	// Topic negotiation is a special request with topic ID 0 and 0 length data.
	private static final byte[] NEGOTIATE_TOPICS_REQUEST = { (byte) 0,
			(byte) 0, (byte) 0, (byte) 0 };

	/**
	 * This timer handles monitoring handles monitoring the connection to the
	 * device;
	 */
	private final WatchdogTimer watchdogTimer;

	/**
	 * Node hosting the subscribers and publishers.
	 */
	private final Node node;

	/**
	 * Map of topic names to the IDs being sent down the channel for them.
	 */
	private final Map<String, Integer> topicIds;

	/**
	 * Topic ID to message deserializer for the associated topic message.
	 */
	private final Map<Integer, MessageDeserializer<?>> messageDeserializers;

	/**
	 * {@link PacketSender} for writing to the other endpoint.
	 */
	private final PacketSender packetSender;

	private final Proxy proxy;
	
	/**
	 * time offset to add when sending time to the micro controller.
	 */
	private Duration timeOffset;
	
	public Protocol(final Node node, PacketSender packetSender) {
		this.node = node;
		this.packetSender = packetSender;
		proxy = new Proxy(node);
		topicIds = Maps.newHashMap();
		messageDeserializers = Maps.newHashMap();
		timeOffset = new Duration(0);
		watchdogTimer = new WatchdogTimer(SYNC_TIMEOUT, new Runnable() {
			@Override
			public void run() {
				node.getLog().info("Connection to client timed out.");
				negotiateTopics();
			}
		});
	}

	/**
	 * Ask the remote endpoint for any topics it wants to publish or subscribe
	 * to.
	 */
	public void negotiateTopics() {
		node.getLog().info("Starting topic negotiation.");
		packetSender.send(NEGOTIATE_TOPICS_REQUEST);
	}

	/**
	 * Construct a valid protocol message. This take the id and m, serializes
	 * them and return the raw bytes to be sent
	 */
	public static byte[] constructMessage(int topicId, Message message) {
		// TODO(damonkohler): Switch to using MessageSerializer.
		int serializationLength = message.serializationLength();
		// Allocate an additional 5 bytes for the topic ID and serialized
		// message length.
		ByteBuffer buffer = ByteBuffer.allocate(serializationLength + 4);
		buffer.order(ByteOrder.LITTLE_ENDIAN);
		buffer.putShort((short) topicId);
		buffer.putShort((short) serializationLength);
		message.serialize(buffer, 0);
		return buffer.array();
	}

	/**
	 * Registers a topic being transmitted over the serial port.
	 * 
	 * @param topic
	 *            the TopicInfo message describing the topic
	 */
	private void registerTopic(TopicInfo topicInfo) {
		String topicName = topicInfo.topic_name;
		int topicId = topicInfo.topic_id;
		if (topicIds.containsKey(topicName)
				&& topicIds.get(topicName) == topicId) {
			return;
		}
		messageDeserializers.put(topicId, node.getMessageSerializationFactory()
				.newMessageDeserializer(topicInfo.message_type));
		topicIds.put(topicName, topicId);
	}

	/**
	 * Registers a publisher being transmitted over the serial port.
	 * 
	 * @param topic
	 *            the TopicInfo message describing the topic
	 */
	private void registerPublisher(TopicInfo topicInfo) {
		registerTopic(topicInfo);
		proxy.registerPublisher(topicInfo.topic_id, topicInfo.topic_name,
				topicInfo.message_type);
	}

	/**
	 * Registers a subscriber being transmitted over the serial port.
	 * 
	 * @param topic
	 *            the TopicInfo message describing the topic
	 */
	private void registerSubscriber(TopicInfo topicInfo) {
		registerTopic(topicInfo);
		int topicId = topicInfo.topic_id;
		MessageListenerForwarding listener = new MessageListenerForwarding(
				topicId, packetSender);
		proxy.registerSubscriber(topicId, topicInfo.topic_name,
				topicInfo.message_type, listener);
	}

	public void start() {
		watchdogTimer.start();
		negotiateTopics();
	}

	public void shutdown() {
		watchdogTimer.cancel();
	}

	/**
	 * Parse a packet from the remote endpoint.
	 * 
	 * @param topicId
	 *            ID of the message topic
	 * @param data
	 *            the serialized message data
	 */
	public void receivePacket(int topicId, byte[] data) {
		if (DEBUG) {
			System.out.println("Received data packet for topic ID: " + topicId);
			System.out.println(BinaryUtils.byteArrayToHexString(data));
		}
		TopicInfo topicInfo = new TopicInfo();
		switch (topicId) {
		case TopicInfo.ID_PUBLISHER:
			topicInfo.deserialize(data);
			registerPublisher(topicInfo);
			break;
		case TopicInfo.ID_SUBSCRIBER:
			topicInfo.deserialize(data);
			registerSubscriber(topicInfo);
			break;
		case TopicInfo.ID_SERVICE_SERVER:
			throw new UnsupportedOperationException();
		case TopicInfo.ID_SERVICE_CLIENT:
			throw new UnsupportedOperationException();
		case TopicInfo.ID_PARAMETER_REQUEST:
			throw new UnsupportedOperationException();
		case TopicInfo.ID_LOG:
			handleLogging(data);
			break;
		case TopicInfo.ID_TIME:
			org.ros.message.std_msgs.Time time = new org.ros.message.std_msgs.Time();
			time.data = node.getCurrentTime().add(timeOffset);
			packetSender.send(constructMessage(TOPIC_TIME, time));
			node.getLog().info("Sending time, offset " + timeOffset + " new time " + time.data);
			watchdogTimer.pulse();
			break;
		default:
			MessageDeserializer<?> messageDeserializer = messageDeserializers
					.get(topicId);
			if (messageDeserializer != null) {
				ByteBuffer buffer = ByteBuffer.wrap(data);
				buffer.order(ByteOrder.LITTLE_ENDIAN);
				Message message = (Message) messageDeserializer
						.deserialize(buffer);
				proxy.publish(topicId, message);
			} else {
				node.getLog()
						.info("Tried to publish to an unregistered topic: "
								+ topicId);
				negotiateTopics();
			}
			break;
		}
	}

	/**
	 * Handle Logging takes the log message from rosserial and rebroadcasts it
	 * via rosout at the appropriate logging level.
	 * 
	 * @param data
	 *            the serialized message data
	 */
	private void handleLogging(byte[] data) {
		Log log = new Log();
		log.deserialize(data);
		switch (log.level) {
		case Log.DEBUG:
			node.getLog().debug(log.msg);
			break;
		case Log.INFO:
			node.getLog().info(log.msg);
			break;
		case Log.WARN:
			node.getLog().warn(log.msg);
			break;
		case Log.ERROR:
			node.getLog().error(log.msg);
			break;
		case Log.FATAL:
			node.getLog().fatal(log.msg);
			break;
		}
	}
	
	/**
	 * Set the time used for all ROS time stamps on the Arduino. Since it can be
	 * pretty hard to set the clock correctly on Android based hardware, this
	 * method provides a way to overwrite the system time.
	 * 
	 * @param time
	 *            the new time
	 */
	public void setTime(Time time)	{
		timeOffset = time.subtract(node.getCurrentTime());
	}
}
