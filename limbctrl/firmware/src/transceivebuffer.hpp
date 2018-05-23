#ifndef SUPREME_LIMBCTRL_TRANSCEIVEBUFFER_HPP
#define SUPREME_LIMBCTRL_TRANSCEIVEBUFFER_HPP

#include <array>
#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {


template <typename Interface, unsigned N, uint8_t SyncByte>
class sendbuffer {
	static const unsigned NumSyncBytes = 2;
	static constexpr uint8_t chk_init = (uint8_t) (NumSyncBytes*SyncByte);
	typedef std::array<uint8_t, N> Buffer_t;
	uint16_t ptr = NumSyncBytes;
	Buffer_t buffer;
	uint8_t  checksum = chk_init;

public:
	sendbuffer()
	: buffer()
	{
		static_assert(N > NumSyncBytes, "Invalid buffer size.");
		for (uint8_t i = 0; i < NumSyncBytes; ++i)
			buffer[i] = SyncByte; // init sync bytes once
	}

	void add_byte(uint8_t byte) {
		assert(ptr < (N-1), 1);
		buffer[ptr++] = byte;
		checksum += byte;
	}

	void add_word(uint16_t word) {
		add_byte((word  >> 8) & 0xff);
		add_byte( word        & 0xff);
	}

	void discard(void) { ptr = NumSyncBytes; }

	void transmit() {
		if (ptr == NumSyncBytes) return;
		add_checksum();
		Interface::send_mode();
		Interface::uart::write(buffer.data(), ptr);
		Interface::uart::flushWriteBuffer();
		Interface::recv_mode();
		/* prepare next */
		ptr = NumSyncBytes;
	}

	uint16_t size(void) const { return ptr; }
	Buffer_t const& get(void) const { return buffer; }

private:
	void add_checksum() {
		assert(ptr < N, 8);
		buffer[ptr++] = ~checksum + 1; /* two's complement checksum */
		checksum = chk_init;
	}
};

template <typename Interface_t, unsigned N>
class recvbuffer {
public:
	typedef std::array<uint8_t, N> Buffer_t;

	Buffer_t buffer;   // TODO make these vars private
	uint8_t  checksum;
	uint8_t  data;
	unsigned ptr = 0;

	recvbuffer() : buffer() {
		buffer.fill(0);
	}

	Buffer_t const& get(void) const { return buffer; }

	bool read_byte(void) {
		bool result = Interface_t::uart::read(data);
		if (result) {
			assert(ptr < N, 9);
			checksum += data;
			buffer[ptr++] = data;
		}
		return result;
	}

	unsigned bytes_received(void) const { return ptr; }

	void reset(void) {
		ptr = 0;
		checksum = 0;
	}

	uint16_t get_word(unsigned offset) const {
		assert((offset+1) < N, 11);
		uint16_t result = buffer[offset] << 8;
		return result + buffer[offset+1];
	}
};

} /* namespace supreme */

#endif /*SUPREME_LIMBCTRL_TRANSCEIVEBUFFER_HPP */
