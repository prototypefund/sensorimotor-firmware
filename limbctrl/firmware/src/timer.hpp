/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCTRL_TIMER_HPP
#define SUPREME_LIMBCTRL_TIMER_HPP

#include <xpcc/architecture/platform.hpp>

template <typename TimerType, unsigned Period>
static void
init_timer()
{
	TimerType::enable();
	TimerType::setMode(TimerType::Mode::UpCounter);
	TimerType::template setPeriod<Board::systemClock>(Period);
	TimerType::enableInterruptVector(true, 10);
	TimerType::enableInterrupt(TimerType::Interrupt::Update);
	TimerType::applyAndReset();
}

template <typename TimerType>
static void
reset_and_start_timer()
{
	TimerType::applyAndReset();
	TimerType::start();
}

template <typename TimerType>
static void
setperiod_and_restart_timer(unsigned Period)
{
	TimerType::template setPeriod<Board::systemClock>(Period);
	TimerType::applyAndReset();
	TimerType::start();
}

// template <typename TimerType>
// static bool
// is_timed_out()
// {
// 	/* TODO check if the return of setperiod == getvalue for timeout*/
// 	// or pull timer status register
// 	/* check for update event and do the ackknowledge here*/
//
// 	if ((TIM1->SR & 0x0001) != 0) {
//
// 	    TIM1->SR &= ~(1<<0);
//
// }


#endif /* SUPREME_LIMBCTRL_TIMER_HPP */
