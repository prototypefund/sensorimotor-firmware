/* read_sensor.hpp */
#include <xpcc/architecture/platform.hpp>

namespace supreme {

template <unsigned ChannelNumber>
class read_sensor {
    typedef xpcc::avr::SystemClock clock;
    uint16_t value = 0;

public:
    read_sensor() {
        /* initialize ADC */
        Adc::initialize<clock, 115000>();
        Adc::setReference(Adc::Reference::InternalVcc);
        value = Adc::readChannel(ChannelNumber);
        //Adc::setChannel(ChannelNumber);
        //Adc::startConversion();
    }

    void step() {
        /*Adc::setChannel(ChannelNumber);
        if (Adc::isConversionFinished()) {
            value = Adc::getValue();
            Adc::startConversion(); // restart the conversion
        }*/
        value = Adc::readChannel(ChannelNumber);
    }

    uint16_t get_value() const { return value; }
};

} /* namespace supreme */
