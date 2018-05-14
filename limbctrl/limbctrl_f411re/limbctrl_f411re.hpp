/* 
   Supreme Machines
   Limb Controller
   STM32 Cortex F411RE 
*/

#ifndef SUPREME_LIMBCTRL_F411RE_HPP
#define SUPREME_LIMBCTRL_F411RE_HPP

#include <xpcc/architecture/platform.hpp>
//#include <xpcc/debug/logger.hpp>
//#define XPCC_BOARD_HAS_LOGGER

using namespace xpcc::stm32;


namespace Board
{

/// STM32F411RE running at 96MHz generated from the internal 16MHz crystal
// Dummy clock for devices

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


/* spinal cord communication */
namespace spinalcord {
    using ro = GpioInputA10;
    using di = GpioOutputA9;
    using read_disable = GpioOutputA11;
    using drive_enable = GpioOutputA12;
	using uart = Usart1;
}

/* motor cord communication */
namespace motorcord {
	using ro = GpioInputA3;
	using di = GpioOutputA2;
    using read_disable = GpioOutputA4;
    using drive_enable = GpioOutputA5;
	using uart = Usart2;
}

/* external interface communication */
namespace external {
    using ro = GpioInputC7;
    using di = GpioOutputC6;
    using read_disable = GpioOutputC8;
    using drive_enable = GpioOutputA13;
	using uart = Usart6;
}

namespace i2c {
    using sda = GpioC9;
    using scl = GpioA8;
}


//using Button = xpcc::GpioInverted<GpioInputC13>;
//using LedD13 = D13;
//using Leds = xpcc::SoftwareGpioPort< LedD13 >;





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
	spinalcord::di::connect(spinalcord::uart::Tx);
	spinalcord::ro::connect(spinalcord::uart::Rx);
	spinalcord::uart::initialize<systemClock, 1000000>(12); // 1Mbaud/s
	spinalcord::drive_enable::setOutput();
	spinalcord::read_disable::setOutput();

	motorcord::di::connect(motorcord::uart::Tx);
	motorcord::ro::connect(motorcord::uart::Rx);
	motorcord::uart::initialize<systemClock, 1000000>(12); // 1Mbaud/s
	motorcord::drive_enable::setOutput();
	motorcord::read_disable::setOutput();

	external::di::connect(external::uart::Tx);
	external::ro::connect(external::uart::Rx);
	external::uart::initialize<systemClock, 1000000>(12); // 1Mbaud/s
	external::drive_enable::setOutput();
	external::read_disable::setOutput();

	/* enable rs485 interfaces */
	com_pwr_en::setOutput();
	com_pwr_en::set(); 



//qb:	Button::setInput();
//qb:	Button::setInputTrigger(Gpio::InputTrigger::RisingEdge);
//qb:	Button::enableExternalInterrupt();
//	Button::enableExternalInterruptVector(12);

}

}

#endif	// SUPREME_LIMBCTRL_F411RE_HPP
