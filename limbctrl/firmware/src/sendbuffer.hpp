#ifndef SUPREME_LIMBCTRL_SENDBUFFER_HPP
#define SUPREME_LIMBCTRL_SENDBUFFER_HPP

#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

//TODO make type of buffer as typedef
//TODO unify with sendbuffer from sensorimotor

template <typename Interface, uint8_t BufferSize, uint8_t SyncByte>
class Sendbuffer {

	uint8_t send_buffer[BufferSize];

public:

	Sendbuffer(uint8_t board_id)
	{
		/* clear buffer */
		memset(send_buffer, 0, BufferSize);

		/* add preamble */
		send_buffer[0] = SyncByte;
		send_buffer[1] = SyncByte;
		send_buffer[2] = board_id;
	}

	void prepare(uint8_t min_id,
	             uint8_t last_min_id,
	             uint8_t board_list,
	             uint8_t packets,
	             uint8_t errors,
	             uint8_t cycles)
	{
		send_buffer[3] = min_id;
		send_buffer[4] = last_min_id;
		send_buffer[5] = board_list;
		send_buffer[6] = packets;
		send_buffer[7] = errors;
		send_buffer[8] = cycles;

		/* create checksum */
		uint8_t checksum = 0;
		for (unsigned i = 0; i < BufferSize-1; ++i)
			checksum += send_buffer[i];
		send_buffer[BufferSize-1] = ~checksum + 1;
	}

	void transmit(void) {
		Interface::send_mode();
		Interface::uart::write(send_buffer, BufferSize);
		Interface::uart::flushWriteBuffer();
		Interface::recv_mode();
	}
};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_SENDBUFFER_HPP */
