//#include <system/communication.hpp>
#include <xpcc/architecture/platform.hpp>
#include "./catch_1.10.0.hpp"


namespace supreme {
namespace local_tests {


/*
template <typename T>
bool verify_checksum(T const& data) {
	uint8_t sum = 0;
	for (auto const& d : data)
		sum += d;
	return sum == 0;
}
*/

TEST_CASE( "fofo", "[communication]")
{
    REQUIRE( true );
}


}} /* namespace supreme::local_tests */

