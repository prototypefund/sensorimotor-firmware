#ifndef SUPREME_RS485CTRL_HPP
#define SUPREME_RS485CTRL_HPP

namespace supreme {

/*TODO implement using sendbuffer */

class rs485_controller : public communication_interface {
public:
	void read_msg(void) {
		// poll port and push to queue
	}
	
    bool send_msg(void) {return true;}
    bool wait_us(unsigned /*usec*/) const { return true; }
    void sleep_s(unsigned /*sec*/) const {}


    void enqueue_sync_bytes() {}
    void enqueue_byte(uint8_t /*byte*/) {}
    void enqueue_checksum() {}

    /* queue-like interface */
    bool empty(void) const  { return false; }
    uint8_t front() const {return 0;}
    void pop(void) {}
    std::size_t size() const  { return 0; }

    bool is_checksum_ok(void) const  {return true;}
    void reset_checksum(void)  {}

    uint8_t  get_byte(void)  {return 0;}
    uint16_t get_word(void)  {return 0;}
};

} /* namespace supreme */

#endif /* SUPREME_RS485CTRL_HPP */
