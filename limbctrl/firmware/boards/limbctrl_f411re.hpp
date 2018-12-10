/*---------------------------------+
 | Supreme Machines                |
 | Limb Controller \w STM32F411RE  |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCTRL_F411RE_HPP
#define SUPREME_LIMBCTRL_F411RE_HPP

#include <xpcc/architecture/platform.hpp>

using namespace xpcc::stm32;

namespace Board
{
	/* STM32F411RE is running at 96MHz
	   generated from the internal 16MHz crystal */

struct systemClock {
	static constexpr uint32_t Frequency = 96 * MHz1;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 2;
	static constexpr uint32_t Apb2 = Frequency;

//	static constexpr uint32_t Adc    = Apb2;

//	static constexpr uint32_t Spi1   = Apb2;
//	static constexpr uint32_t Spi2   = Apb1;
//	static constexpr uint32_t Spi3   = Apb1;
//	static constexpr uint32_t Spi4   = Apb2;
//	static constexpr uint32_t Spi5   = Apb2;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart6 = Apb2;

//	static constexpr uint32_t I2c1   = Apb1;
//	static constexpr uint32_t I2c2   = Apb1;
//	static constexpr uint32_t I2c3   = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 2;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1  = Apb2Timer;
	static constexpr uint32_t Timer2  = Apb1Timer;
	static constexpr uint32_t Timer3  = Apb1Timer;
	static constexpr uint32_t Timer4  = Apb1Timer;
	static constexpr uint32_t Timer5  = Apb1Timer;
	static constexpr uint32_t Timer9  = Apb2Timer;
	static constexpr uint32_t Timer10 = Apb2Timer;
	static constexpr uint32_t Timer11 = Apb2Timer;

	static bool inline
	enable()
	{
		//ClockControl::enableInternalClock();	// 16MHz
		ClockControl::enableExternalCrystal();
		ClockControl::enablePll(
			ClockControl::PllSource::ExternalCrystal,
			3,	// 12MHz / N=3 -> 4MHz
			96,	// 4MHz * M=96 -> 384MHz
			4,	// 384MHz / P=4 -> 96MHz = F_cpu
			8	// 384MHz / Q=8 -> 48MHz = F_usb
		);
		// set flash latency for 96MHz
		ClockControl::setFlashLatency(Frequency);
		// switch system clock to PLL output
		ClockControl::enableSystemClock(ClockControl::SystemClockSource::Pll);
		ClockControl::setAhbPrescaler(ClockControl::AhbPrescaler::Div1);
		// APB1 has max. 50MHz
		ClockControl::setApb1Prescaler(ClockControl::Apb1Prescaler::Div2);
		ClockControl::setApb2Prescaler(ClockControl::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		xpcc::clock::fcpu     = Frequency;
		xpcc::clock::fcpu_kHz = Frequency / 1000;
		xpcc::clock::fcpu_MHz = Frequency / 1000000;
		xpcc::clock::ns_per_loop = ::round(3000 / (Frequency / 1000000));

		return true;
	}
};


/* Limbcontroller */
using led_ylw    = GpioB5;
using led_red    = GpioA15;

using ain_ub     = GpioC0;
using ain_vbat   = GpioC1;

using stat_vbat  = GpioA0;

using mot_pwr_en = GpioA7;
using com_pwr_en = GpioA6;


namespace i2c {
	using sda = GpioC9;
	using scl = GpioA8;
}



template <typename Interface, unsigned baudrate>
struct rs485_interface {

	using read_output  = typename Interface::read_output;
	using drive_input  = typename Interface::drive_input;
	using read_disable = typename Interface::read_disable;
	using drive_enable = typename Interface::drive_enable;
	using uart         = typename Interface::uart;

	static void initialize(void) {
		drive_input::connect(uart::Tx);
		read_output::connect(uart::Rx);
		uart::template initialize<systemClock, baudrate>(12); // 1Mbaud/s
		drive_enable::setOutput();
		read_disable::setOutput();
		read_disable::reset();     // set to receive mode
		drive_enable::reset();
	}
	static void send_mode(void) {
		xpcc::delayNanoseconds(50); // wait for signal propagation
		read_disable::set();
		drive_enable::set();
		xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
	}
	static void recv_mode(void) {
		xpcc::delayMicroseconds(10); // wait at least one byte before disabling the driver
		read_disable::reset();
		drive_enable::reset();
		xpcc::delayNanoseconds(70); // wait for signal propagation
	}
};


/* declare pins for interfaces */
struct spinalcord {
	using read_output  = GpioInputA10;
	using drive_input  = GpioOutputA9;
	using read_disable = GpioOutputA11;
	using drive_enable = GpioOutputA12;
	using uart = Usart1;
};

struct motorcord {
	using read_output  = GpioInputA3;
	using drive_input  = GpioOutputA2;
	using read_disable = GpioOutputA4;
	using drive_enable = GpioOutputA5;
	using uart = Usart2;
};

struct external {
	using read_output  = GpioInputC7;
	using drive_input  = GpioOutputC6;
	using read_disable = GpioOutputC8;
	using drive_enable = GpioOutputA13;
	using uart = Usart6;
};

/* declare interfaces */
using rs485_spinalcord = rs485_interface<spinalcord, 3000000>; // 3 Mbaud/s
using rs485_motorcord  = rs485_interface<motorcord , 1000000>; // 1 Mbaud/s
using rs485_external   = rs485_interface<external  , 3000000>; // 3 Mbaud/s



inline void
initialize()
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();

	/* power-up sensorimotors */
	mot_pwr_en::setOutput();
	mot_pwr_en::reset(); // Note: This can currently not be shut down, and is a work-around for the flipped MOSFET, see ISSUE-x

	/* init LEDs */
	led_ylw::setOutput();
	led_red::setOutput();
	led_ylw::reset();
	led_red::reset();


	/* setup rs485 interfaces */
	rs485_spinalcord::initialize();
	rs485_motorcord ::initialize();
	rs485_external  ::initialize();

	/* enable power for rs485 interfaces */
	com_pwr_en::setOutput();
	com_pwr_en::set();

} /* initialize */

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_F411RE_HPP */
