/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCTRL_COMMUNICATION_HPP
#define SUPREME_LIMBCTRL_COMMUNICATION_HPP

#include <array>
#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

/* Handles communication with other limbs via spinal cord
*/
template <typename TimerType, uint8_t SyncByte, uint8_t MaxID, unsigned BytesPerSlot, unsigned SlotTime_us, typename target_voltage_t>
class CommunicationController {
public:

	static const unsigned TranspDataSize = 28;

	enum recv_state_t {
		initializing = 0,
		synchronizing,
		awaiting_id,
		reading_spinal_data,
		reading_transp_data,
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

	target_voltage_t& target_voltages;
	bool transp_mode = false;

	CommunicationController(volatile bool *timed_out, target_voltage_t& target_voltages)
	: buffer()
	, rx_timed_out(timed_out)
	, target_voltages(target_voltages)
	{
		static_assert(TranspDataSize < BytesPerSlot);
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
		return recv_state_t::error;
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
		if (data < MaxID) {
			return reading_spinal_data;
		}
		else if (data == 0xff) {
			transp_mode = true;
			return reading_transp_data;
		}
		else return error;
	}

	recv_state_t get_spinal_data()
	{
		return (bytecount < BytesPerSlot) ? reading_spinal_data : validating;
	}

	recv_state_t get_transp_data()
	{
		return (bytecount < TranspDataSize) ? reading_transp_data : validating;
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
				transp_mode = false;
				TimerType::applyAndReset();
				TimerType::pause();
				/* fall through */

			case synchronizing      : if (byte_received()) state = get_sync_bytes();   break;
			case awaiting_id        : if (byte_received()) state = get_id();           break;
			case reading_spinal_data: if (byte_received()) state = get_spinal_data();  break;
			case reading_transp_data: if (byte_received()) state = get_transp_data();  break;
			case validating         :                      state = verify_checksum();  break;

			case success:
				if (transp_mode) {
					result = false; // trans data is not visible outside
					copy_transparent_data();
				} else {
					result = true;
				}
				state = recv_state_t::done;
				++packets;
				break;
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

	void copy_transparent_data(void) {
		for (unsigned i = 0; i < 12; ++i) {
			unsigned offset = 3; // 2 x sync + id
			target_voltages[i] = (uint16_t) (buffer[2*i+offset] << 8 | buffer[2*i+1+offset]);
		}
	}

};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_COMMUNICATION_HPP */
