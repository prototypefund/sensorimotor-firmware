#include <xpcc/architecture/platform.hpp>
#include <src/communication.hpp>

//TODO rename Board to Limbcontroller

void motorcord_send_mode() {
	xpcc::delayNanoseconds(50); // wait for signal propagation
	Board::motorcord::read_disable::set();
	Board::motorcord::drive_enable::set();
	xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
}
void motorcord_receive_mode() {
	xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
	Board::motorcord::read_disable::reset();
	Board::motorcord::drive_enable::reset();
	xpcc::delayNanoseconds(70); // wait for signal propagation
}

void spinalcord_send_mode() {
	xpcc::delayNanoseconds(50); // wait for signal propagation
	Board::spinalcord::read_disable::set();
	Board::spinalcord::drive_enable::set();
	xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
}
void spinalcord_receive_mode() {
	xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
	Board::spinalcord::read_disable::reset();
	Board::spinalcord::drive_enable::reset();
	xpcc::delayNanoseconds(70); // wait for signal propagation
}
