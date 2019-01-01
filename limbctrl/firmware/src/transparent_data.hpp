/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCTRL_TRANSPARENT_DATA_HPP
#define SUPREME_LIMBCTRL_TRANSPARENT_DATA_HPP

#include <src/transceivebuffer.hpp>

namespace supreme {

template <typename RXInterface, typename TXInterface, unsigned NumTransparentBytes = 28, uint8_t SyncByte = 0x55>
class TransparentData
{
	bool sync_state = false;

	recvbuffer<RXInterface, NumTransparentBytes> recv;
	sendbuffer<TXInterface, NumTransparentBytes, SyncByte>        send;

public:

	enum recv_state_t {
		initializing = 0,
		synchronizing,
		reading_id,
		reading_data,
		validating,
		success,
		error,
		done
	};

	recv_state_t state = initializing;

	TransparentData() : recv(), send() {}

	void clear(void) { recv.reset(); }

	bool read(void)
	{
		bool result = false;

		switch(state)
		{
			case initializing:
				sync_state = false;
				state = reset_buffer();
				/* fall through */

			case synchronizing: if (recv.read_byte()) state = get_sync_bytes();   break;
			case reading_id   : if (recv.read_byte()) state = get_id();           break;
			case reading_data : if (recv.read_byte()) state = get_data();         break;
			case validating   :                       state = verify_checksum();  break;
			case success      :
				result = true;
				state = recv_state_t::done;
				//TODO copy data to targetvoltages
				break;

			case error        : result = false;  state = recv_state_t::done; break;
			case done:
			default:
				state = recv_state_t::initializing; // start over again
				break;

		} // switch(state)

		return result; // return true of valid slot could be read
	}

	void write(void) {
		for (unsigned i = 2; i < recv.get_buffer().size()-1; ++i)
			send.add_byte(recv.get_buffer()[i]);

		assert(send.size() == NumTransparentBytes-1, 18);
		send.transmit();
	}

private:
	recv_state_t reset_buffer() { recv.reset(); return recv_state_t::synchronizing; }

	recv_state_t verify_checksum() {
		return recv.verify() ? recv_state_t::success : recv_state_t::error;
	}

	recv_state_t get_sync_bytes()
	{
		if (recv.get_data() != SyncByte) {
			sync_state = false;
			return recv_state_t::done;
		}

		if (sync_state) {
			/* Sync found, set timeout and continue reading. */
			sync_state = false;
			return recv_state_t::reading_id;
		}

		sync_state = true;
		return recv_state_t::synchronizing;
	}

	recv_state_t get_id() { return (recv.get_data() == 0xff) ? reading_data : error; }

	recv_state_t get_data() {
		return (recv.bytes_received() < NumTransparentBytes) ? reading_data : validating;
	}

};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_TRANSPARENT_DATA_HPP */
