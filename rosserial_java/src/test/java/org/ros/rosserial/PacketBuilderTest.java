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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.junit.Test;
import org.ros.message.rosserial_msgs.TopicInfo;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PacketBuilderTest {

	@Test
	public void testTopicNegotiation() {
		PacketBuilder builder = new PacketBuilder(new PacketReceiver() {
			@Override
			public void receive(int topicId, byte[] data) {
				assertEquals(0, topicId);
				assertEquals(0, data.length);
			}
		});
		assertEquals(PacketState.FLAGA, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.FLAGB, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.CHECKSUM, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.FLAGA, builder.getPacketState());
	}

	@Test
	public void testInvalidChecksum() {
		PacketBuilder builder = new PacketBuilder(new PacketReceiver() {
			@Override
			public void receive(int topicId, byte[] data) {
				assertEquals(0, topicId);
				assertEquals(0, data.length);
			}
		});
		assertEquals(PacketState.FLAGA, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.FLAGB, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.CHECKSUM, builder.getPacketState());
		try {
			builder.addByte((byte) 0);
			fail("Should have thrown an IllegalStateException.");
		} catch (IllegalStateException e) {
			// The checksum is invalid.
		}
		assertEquals(PacketState.FLAGA, builder.getPacketState());
	}

	@Test
	public void testTopicInfo() {
		final TopicInfo topicInfo = new TopicInfo();
		topicInfo.message_type = "std_msgs/String";
		topicInfo.topic_id = 101;
		topicInfo.topic_name = "hello_world";
		// Reserve 2 additional bytes for the data length.
		ByteBuffer serializedTopicInfo = ByteBuffer.allocate(topicInfo
				.serializationLength() + 2);
		serializedTopicInfo.order(ByteOrder.LITTLE_ENDIAN);
		serializedTopicInfo.putShort((short) topicInfo.serializationLength());
		topicInfo.serialize(serializedTopicInfo, 0);

		PacketBuilder builder = new PacketBuilder(new PacketReceiver() {
			@Override
			public void receive(int topicId, byte[] data) {
				assertEquals(0, topicId);
				assertEquals(topicInfo.serializationLength(), data.length);
				TopicInfo receivedTopicInfo = new TopicInfo();
				receivedTopicInfo.deserialize(data);
				assertEquals(topicInfo.message_type,
						receivedTopicInfo.message_type);
				assertEquals(topicInfo.topic_id, receivedTopicInfo.topic_id);
				assertEquals(topicInfo.topic_name, receivedTopicInfo.topic_name);
			}
		});

		assertEquals(PacketState.FLAGA, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.FLAGB, builder.getPacketState());
		builder.addByte((byte) 0xFF);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		assertEquals(PacketState.HEADER, builder.getPacketState());
		builder.addByte((byte) 0);
		for (int i = 0; i < serializedTopicInfo.capacity(); i++) {
			builder.addByte(serializedTopicInfo.get(i));
		}
		// It's ok to ignore the topic ID since it's 0 in this case.
		builder.addByte(DefaultPacketSender
				.calculateChecksum(serializedTopicInfo.array()));
		assertEquals(PacketState.FLAGA, builder.getPacketState());
	}
}
