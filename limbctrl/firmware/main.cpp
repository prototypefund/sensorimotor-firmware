#include <xpcc/architecture/platform.hpp>

using namespace Board;

int
main()
{
	Board::initialize();
	LedBlue::setOutput();
	LedWhite::setOutput();

	// Use the logging streams to print some messages.
	// Change XPCC_LOG_LEVEL above to enable or disable these messages
//	XPCC_LOG_DEBUG   << "debug"   << xpcc::endl;
//	XPCC_LOG_INFO    << "info"    << xpcc::endl;
//	XPCC_LOG_WARNING << "warning" << xpcc::endl;
//	XPCC_LOG_ERROR   << "error"   << xpcc::endl;

//	uint32_t counter(0);
	LedBlue::toggle();
	while (1)
	{
		LedBlue::toggle();
		LedWhite::toggle();
		xpcc::delayMilliseconds(100);

//		XPCC_LOG_INFO << "loop: " << counter++ << xpcc::endl;
	}

	return 0;
}
