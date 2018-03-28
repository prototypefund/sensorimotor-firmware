/* 
   Supreme Machines
   Limb Controller
   STM32 Cortex F411RE 
*/

#ifndef SUPREME_LIMBCTRL_F411RE_HPP
#define SUPREME_LIMBCTRL_F411RE_HPP

#include <xpcc/architecture/platform.hpp>
#include <xpcc/debug/logger.hpp>
#define XPCC_BOARD_HAS_LOGGER

using namespace xpcc::stm32;


namespace Board
{

/// STM32F411RE running at 96MHz generated from the internal 16MHz crystal
// Dummy clock for devices
/*
struct systemClock {
	static constexpr uint32_t Frequency = 96 * MHz1;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 2;
	static constexpr uint32_t Apb2 = Frequency;

	static constexpr uint32_t Adc    = Apb2;

	static constexpr uint32_t Spi1   = Apb2;
	static constexpr uint32_t Spi2   = Apb1;
	static constexpr uint32_t Spi3   = Apb1;
	static constexpr uint32_t Spi4   = Apb2;
	static constexpr uint32_t Spi5   = Apb2;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart6 = Apb2;

	static constexpr uint32_t I2c1   = Apb1;
	static constexpr uint32_t I2c2   = Apb1;
	static constexpr uint32_t I2c3   = Apb1;

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
		ClockControl::enableInternalClock();	// 16MHz
		ClockControl::enablePll(
			ClockControl::PllSource::InternalClock,
			4,	// 16MHz / N=4 -> 4MHz
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
*/

struct systemClock {
	static constexpr uint32_t Frequency = MHz64;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 2;
	static constexpr uint32_t Apb2 = Frequency;

	static constexpr uint32_t Adc1   = Apb2;
	static constexpr uint32_t Adc2   = Apb2;
	static constexpr uint32_t Adc3   = Apb2;

	static constexpr uint32_t Spi1   = Apb2;
	static constexpr uint32_t Spi2   = Apb1;
	static constexpr uint32_t Spi3   = Apb1;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart3 = Apb1;
	static constexpr uint32_t Uart4  = Apb1;
	static constexpr uint32_t Uart5  = Apb1;

	static constexpr uint32_t Can1   = Apb1;

	static constexpr uint32_t I2c1   = Apb1;
	static constexpr uint32_t I2c2   = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 2;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1  = Apb2Timer;
	static constexpr uint32_t Timer2  = Apb1Timer;
	static constexpr uint32_t Timer3  = Apb1Timer;
	static constexpr uint32_t Timer4  = Apb1Timer;
	static constexpr uint32_t Timer5  = Apb1Timer;
	static constexpr uint32_t Timer6  = Apb1Timer;
	static constexpr uint32_t Timer7  = Apb1Timer;
	static constexpr uint32_t Timer8  = Apb2Timer;

	static bool inline
	enable()
	{
		ClockControl::enableInternalClock();	// 8MHz
		// internal clock / 2 * 16 = 64MHz, => 64/1.5 = 42.6 => bad for USB
		ClockControl::enablePll(ClockControl::PllSource::InternalClock, ClockControl::UsbPrescaler::Div1_5, 16);
		// set flash latency for 64MHz
		ClockControl::setFlashLatency(Frequency);
		// switch system clock to PLL output
		ClockControl::enableSystemClock(ClockControl::SystemClockSource::Pll);
		ClockControl::setAhbPrescaler(ClockControl::AhbPrescaler::Div1);
		// APB1 has max. 36MHz
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

// Accelboard
using LedBlue  = GpioA8;
using LedWhite = GpioB1;

/* Limbcontroller */
using led_ylw    = GpioB5;
using led_red    = GpioA15;

using ain_ub     = GpioC0;
using ain_vbat   = GpioC1;

using stat_vbat  = GpioA0;

using mot_pwr_en = GpioA7;
using com_pwr_en = GpioA6;


/* spinal cord communication */
namespace sc {
    using di  = GpioA9;
    using ro  = GpioA10;
    using nre = GpioA11;
    using de  = GpioA12;
}

/* motor cord communication */
namespace mc {
    using di  = GpioA2;
    using ro  = GpioA3;
    using nre = GpioA4;
    using de  = GpioA5;
}

/* external interface communication */
namespace ex {
    using di  = GpioC6;
    using ro  = GpioC7;
    using nre = GpioC8;
    using de  = GpioA13;
}

namespace i2c {
    using sda = GpioC9;
    using scl = GpioA8;
}


//using Button = xpcc::GpioInverted<GpioInputC13>;
//using LedD13 = D13;
//using Leds = xpcc::SoftwareGpioPort< LedD13 >;


namespace stlink
{
using Rx = GpioInputA3;
using Tx = GpioOutputA2;
using Uart = Usart2;
}


inline void
initialize()
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();

	//qb:stlink::Tx::connect(stlink::Uart::Tx);
	//qb:stlink::Rx::connect(stlink::Uart::Rx, Gpio::InputType::PullUp);
	stlink::Uart::initialize<systemClock, 115200>(12);

//qb:	Button::setInput();
//qb:	Button::setInputTrigger(Gpio::InputTrigger::RisingEdge);
//qb:	Button::enableExternalInterrupt();
//	Button::enableExternalInterruptVector(12);
}

}

#endif	// SUPREME_LIMBCTRL_F411RE_HPP
