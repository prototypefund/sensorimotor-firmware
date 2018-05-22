#ifndef SUPREME_LIMBCTRL_COMMUNICATION_HPP
#define SUPREME_LIMBCTRL_COMMUNICATION_HPP

#include <array>
#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

/* Handles communication with other limbs via spinal cord
*/
template <typename TimerType, uint8_t SyncByte, uint8_t MaxID, unsigned BytesPerSlot, unsigned SlotTime_us>
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
	typedef std::array<uint8_t, BytesPerSlot> Buffer_t;
	Buffer_t buffer;

	bool sync_state = false;
	volatile bool *rx_timed_out;

	CommunicationController(volatile bool *timed_out)
	: rx_timed_out(timed_out)
	{
		buffer.fill(0);
		eat();
		init_timer<TimerType, SlotTime_us>();
		*rx_timed_out = false;
		assert((*timed_out == false),3);
	}

	Buffer_t const& get(void) const { return buffer; }

	recv_state_t reset_buffer() {
		buffer.fill(0); /* TODO only clear what has been written so far */
		bytecount = 0;
		checksum = 0;
		return recv_state_t::synchronizing;
	}

	void eat() {
		uint8_t dump;
		while(rs485_spinalcord::uart::read(dump));
	}

	bool byte_received(void) {
		bool result = rs485_spinalcord::uart::read(data);
		if (result) {
			checksum += data;
			assert(bytecount < BytesPerSlot, 16);
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
			return recv_state_t::done;
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

	bool read_slot(void)
	{
		bool result = false;

		if (*rx_timed_out) {
			state = recv_state_t::done;
			++errors;
			*rx_timed_out = false;
		}

		switch(state)
		{
			case initializing:
				sync_state = false;
				state = reset_buffer();
				*rx_timed_out = false;
				TimerType::applyAndReset();
				TimerType::pause();
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
				TimerType::applyAndReset();
				TimerType::pause(); // stop timer before leaving
				state = recv_state_t::initializing; // start over again
				break;

		} // switch(state)

		return result; // return true of valid slot could be read
	}

};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_COMMUNICATION_HPP */
