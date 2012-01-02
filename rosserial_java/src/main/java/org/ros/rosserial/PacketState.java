package org.ros.rosserial;

// parsing state machine variables/enumes
enum PacketState {
	FLAGA, FLAGB, HEADER, DATA, CHECKSUM
}