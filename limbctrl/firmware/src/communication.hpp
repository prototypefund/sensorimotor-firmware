#ifndef SUPREME_LIMBCTRL_COMMUNICATION_HPP
#define SUPREME_LIMBCTRL_COMMUNICATION_HPP

#include <xpcc/architecture/platform.hpp>

void motorcord_send_mode();
void motorcord_receive_mode();

void spinalcord_send_mode();
void spinalcord_receive_mode();

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

#endif /* SUPREME_LIMBCTRL_COMMUNICATION_HPP */
