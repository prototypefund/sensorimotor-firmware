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

#endif /* SUPREME_LIMBCTRL_TIMER_HPP */
