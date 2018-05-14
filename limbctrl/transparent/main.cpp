#include <xpcc/architecture/platform.hpp>
//#include <src/communication_interface.hpp>
//#include <src/sensorimotor.hpp>
//#include <src/rs485_controller.hpp>

using namespace Board;


void motorcord_send_mode() {
	xpcc::delayNanoseconds(50); // wait for signal propagation
	motorcord::read_disable::set();
	motorcord::drive_enable::set();
	xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
}
void motorcord_receive_mode() {
	xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
	motorcord::read_disable::reset();
	motorcord::drive_enable::reset();
	xpcc::delayNanoseconds(70); // wait for signal propagation
}
// void external_send_mode() {
// 	xpcc::delayNanoseconds(50); // wait for signal propagation
// 	external::read_disable::set();
// 	external::drive_enable::set();
// 	xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
// }
// void external_receive_mode() {
// 	xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
// 	external::read_disable::reset();
// 	external::drive_enable::reset();
// 	xpcc::delayNanoseconds(70); // wait for signal propagation
// }
void spinalcord_send_mode() {
	xpcc::delayNanoseconds(50); // wait for signal propagation
	spinalcord::read_disable::set();
	spinalcord::drive_enable::set();
	xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
}
void spinalcord_receive_mode() {
	xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
	spinalcord::read_disable::reset();
	spinalcord::drive_enable::reset();
	xpcc::delayNanoseconds(70); // wait for signal propagation
}

enum Sendmode {
	none     = 0,
	sc_to_mc = 1,
	mc_to_sc = 2
};

int
main()
{
	Board::initialize();
	mot_pwr_en::setOutput();
	com_pwr_en::setOutput();
	led_ylw::setOutput();
	led_red::setOutput();

	led_ylw::reset();
	led_red::reset();

	mot_pwr_en::reset(); // workaround see ISSUE-x

	com_pwr_en::set(); // enable rs485 interfaces
	//supreme::rs485_controller com;
	//supreme::sensorimotor ux0 = supreme::sensorimotor{0, com};

	motorcord::drive_enable::setOutput();
	motorcord::read_disable::setOutput();
	// external::drive_enable::setOutput();
	// external::read_disable::setOutput();
	spinalcord::drive_enable::setOutput();
	spinalcord::read_disable::setOutput();

	motorcord_receive_mode();
	spinalcord_receive_mode();
	//external_receive_mode();


	//spinalcord_send_mode();

	//spinalcord::drive_enable::set();
	uint8_t recv_buffer = 0;



	Sendmode mode = Sendmode::none;

	//spinalcord_send_mode();
	//motorcord_send_mode();
	while (1)
	{

		switch(mode)
		{
			case sc_to_mc:
				motorcord::uart::write(recv_buffer);
				motorcord::uart::flushWriteBuffer();
				xpcc::delayMicroseconds(20);
				if (!spinalcord::uart::read(recv_buffer))
				{
					motorcord_receive_mode();
					mode = none;
					led_ylw::reset();
				}
				break;

			case mc_to_sc:
				spinalcord::uart::write(recv_buffer);
				spinalcord::uart::flushWriteBuffer(); // or wait for defined time, eg. time of a byte to follow + x
				xpcc::delayMicroseconds(20);
				if (!motorcord::uart::read(recv_buffer)) {
					spinalcord_receive_mode();
					mode = none;
					led_red::reset();
				}
				break;

			case none:
			default:
				if (spinalcord::uart::read(recv_buffer))
				{
					motorcord_send_mode();
					mode = sc_to_mc;
					led_ylw::set();
				}
				else if (motorcord::uart::read(recv_buffer)) {
					spinalcord_send_mode();
					mode = mc_to_sc;
					led_red::set();
				}
				break;
		}
		/*
		if (motorcord::uart::read(recv_buffer)) {
			if (!sc_is_sendmode) {
				spinalcord_send_mode();
				sc_is_sendmode = true;
				led_ylw::set();
			}
			spinalcord::uart::write(recv_buffer);
			//external::flushWriteBuffer();
		}
		else {
			if (sc_is_sendmode) {
				spinalcord::uart::flushWriteBuffer();
				spinalcord_receive_mode();
				sc_is_sendmode = false;
				led_ylw::reset();
			}
		}

		// on byte received via external
		if (spinalcord::uart::read(recv_buffer)) {
			if (!mc_is_sendmode) {
				motorcord_send_mode();
				mc_is_sendmode = true;
				led_red::set();
			}
			motorcord::uart::write(recv_buffer);
			//motorcord::flushWriteBuffer();
		}
		/// nothing received, so nothing to send
		else {
			if (mc_is_sendmode) {
				motorcord::uart::flushWriteBuffer();
				motorcord_receive_mode();
				mc_is_sendmode = false;
				led_red::reset();
			}
		}*/

		//led_red::toggle();
		//xpcc::delayMilliseconds(1); // maybe not delaying???
		//spinalcord::uart::write(foo++);
	//	ux0.execute_cycle();
	}
	return 0;
}
