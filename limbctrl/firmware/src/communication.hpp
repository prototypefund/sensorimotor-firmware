#ifndef SUPREME_LIMBCTRL_COMMUNICATION_HPP
#define SUPREME_LIMBCTRL_COMMUNICATION_HPP

#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

/* Handles communication with other limbs via spinal cord
*/
template <typename TimerType, uint8_t SyncByte, uint8_t MaxID, unsigned BytesPerSlot>
class CommunicationController {
public:

	enum recv_state_t {
		initializing = 0,
		synchronizing,
		awaiting_id,
		reading_data,
		validating,
		success,
		error,
		done
	};

	recv_state_t state = initializing;

	uint8_t bytecount = 0;
	uint8_t checksum = 0;
	uint8_t packets = 0;
	uint8_t errors = 0;

	uint8_t received_id = 255;

	uint8_t data = 0;
	uint8_t buffer[BytesPerSlot];

	bool sync_state = false;

	CommunicationController()
	{
		memset(buffer, 0, BytesPerSlot);
	}

	recv_state_t reset_buffer() {
		memset(buffer, 0, bytecount); /* clear what has been written so far */
		bytecount = 0;
		return recv_state_t::synchronizing;
	}

	bool byte_received(void) {
		bool result = rs485_spinalcord::uart::read(data);
		if (result) {
			checksum += data;
			buffer[bytecount++] = data;
		}
		return result;
	}

	recv_state_t verify_checksum()
	{
		if (checksum == 0) {
			received_id = buffer[2];
			return success;
		}
		return error;
	}

	recv_state_t get_sync_bytes()
	{
		if (data != SyncByte) {
			sync_state = false;
			return recv_state_t::initializing;
		}

		if (sync_state) {
			/* Sync found, set timeout and continue reading. */
			sync_state = false;
			reset_and_start_timer<TimerType>();
			return recv_state_t::awaiting_id;
		}

		sync_state = true;
		return recv_state_t::synchronizing;
	}

	recv_state_t get_id()
	{	/* check for valid id */
		return (data < MaxID) ? reading_data : error;
	}

	recv_state_t get_data()
	{
		return (bytecount < BytesPerSlot) ? reading_data : validating;
	}

	uint8_t get_received_id() const { return received_id; }

	bool read_slot(bool rx_timed_out /*TODO can we ask the timer?*/)
	{
		bool result = false;

		if (rx_timed_out) {
			state = recv_state_t::initializing;
			++errors;
			rx_timed_out = false;
		}

		switch(state)
		{
			case initializing:
				state = reset_buffer();
				/* fall through */

			case synchronizing:
				if (byte_received())
					state = get_sync_bytes();
				break;

			case awaiting_id:
				if (byte_received())
					state = get_id();
				break;

			case reading_data:
				if (byte_received())
					state = get_data();
				break;

			case validating:
				state = verify_checksum();
				break;

			case success: result = true;  state = recv_state_t::done; ++packets; break;
			case error:   result = false; state = recv_state_t::done; ++errors;  break;

			case done:
			default:
				TimerType::pause(); // stop timer before leaving
				state = recv_state_t::initializing; // start over again
				break;

		} // switch(state)

		return result; // return true of valid slot could be read
	}

};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_COMMUNICATION_HPP */
