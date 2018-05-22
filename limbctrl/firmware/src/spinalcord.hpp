#ifndef SUPREME_LIMBCTRL_SPINALCORD_HPP
#define SUPREME_LIMBCTRL_SPINALCORD_HPP

#include <xpcc/architecture/platform.hpp>
#include <src/transceivebuffer.hpp>

using namespace Board;

namespace supreme {


template <typename Interface, uint8_t BufferSize, uint8_t SyncByte, typename Motorcord_t, unsigned BoardID>
class SpinalCord : public sendbuffer<Interface, BufferSize, SyncByte> {

	Motorcord_t const& motorcord;

public:

	SpinalCord(Motorcord_t const& motorcord)
	: sendbuffer<Interface, BufferSize, SyncByte>()
	, motorcord(motorcord)
	{}

	void prepare(uint8_t min_id,
	             uint8_t last_min_id,
	             uint8_t board_list,
	             uint8_t packets,
	             uint8_t errors,
	             uint8_t cycles)
	{
		this->add_byte(BoardID);

		/*temporary testing data*/
		this->add_byte(min_id);
		this->add_byte(last_min_id);
		this->add_byte(board_list);
		this->add_byte(packets);
		this->add_byte(errors);
		this->add_byte(cycles);
		this->add_byte(0);

		auto const& motors = motorcord.get_motors();
		for (auto const& m: motors) {
			this->add_byte(m.get_id());
			this->add_byte(m.get_connection_status());
			auto const& s = m.get_status_data();
			this->add_word(s.position);
			this->add_word(s.current);
			this->add_word(s.voltage_back_emf);
			this->add_word(s.voltage_supply);
			this->add_word(s.temperature);
			//TODO add velocity and target voltage
		}
		/*reserved*/
		for (unsigned i = this->size(); i < BufferSize-1; ++i)
			this->add_byte(0xEE);
		/* checksum is added automagically */
		assert(this->size() == BufferSize-1,19);
	}

};

template <typename Interface>
class SpinalCordFull {
	// std::array<uint8_t,BufferSize> buffer;
	// unsigned ptr;

	bool transmission_pending = false;

public:

	SpinalCordFull() {
		// reset();
	}

	// void reset(void) {
	// 	buffer.fill(0);
	// 	ptr = 0;
	// }

	// template <typename Buffer_t>
	// void add(Buffer_t const& append_buf)
	// {
	// 	const unsigned size = append_buf.size();
	// 	assert((ptr + size) < BufferSize, 21);
	// 	for (unsigned i = 0; i < size; ++i)
	// 		buffer[ptr++] = append_buf[i];
	// }

	template <typename Buffer_t>
	void start_transmission(Buffer_t const& buffer)
	{
		while(transmission_pending);
		Interface::send_mode();
		Interface::uart::write(buffer.data(), buffer.size());
		transmission_pending = true;
	}

	void check_transmission_finished(void)
	{
		if (transmission_pending && Interface::uart::isWriteFinished()) {
			Interface::recv_mode();
			transmission_pending = false;
		}
		//TODO transceiver disable could be done with interrupt
	}
};

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_SPINALCORD_HPP */
