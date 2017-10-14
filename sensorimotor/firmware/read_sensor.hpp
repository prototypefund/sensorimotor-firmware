/* read_sensor.hpp */
#include <xpcc/architecture/platform.hpp>

namespace supreme {

class read_sensor {
    typedef xpcc::avr::SystemClock clock;
    uint16_t value = 0;

public:
    read_sensor() {
        /* initialize ADC */
        Adc::initialize<clock, 115000>();
        Adc::setReference(Adc::Reference::InternalVcc);
        value = Adc::readChannel(1);
        Adc::setChannel(1);
        Adc::startConversion();
    }

    void step() {
        if (Adc::isConversionFinished()){
            value = Adc::getValue();
            // restart the conversion
            Adc::setChannel(1);
            Adc::startConversion();
        }
    }

    uint16_t get_value() const { return value; }
};

} /* namespace supreme */
