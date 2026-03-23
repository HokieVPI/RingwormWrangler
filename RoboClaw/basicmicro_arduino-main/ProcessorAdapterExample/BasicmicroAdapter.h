#ifndef BASICMICRO_ADAPTER_H
#define BASICMICRO_ADAPTER_H

#include <stdint.h>
#include <string.h>

// Forward declaration of Basicmicro class
class Basicmicro;

/**
 * @brief Minimal implementation of Arduino's Stream class
 * 
 * This class provides the interface expected by the Basicmicro library.
 * Any microcontroller can implement these methods to use the library.
 */
class Stream {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual void flush() = 0;
    virtual ~Stream() {}
};

/**
 * @brief Adapter class for hardware serial interfaces
 * 
 * Implement this class for your specific microcontroller by
 * providing implementations for the communication methods.
 */
class HardwareSerial : public Stream {
public:
    /**
     * @brief Initialize the serial port
     * 
     * @param baud_rate Desired baud rate
     */
    virtual void begin(long baud_rate) = 0;
    
    /**
     * @brief Check if data is available to read
     * 
     * @return Number of bytes available to read
     */
    virtual int available() override = 0;
    
    /**
     * @brief Read a byte from the serial port
     * 
     * @return The byte read, or -1 if no data is available
     */
    virtual int read() override = 0;
    
    /**
     * @brief Look at the next byte without removing it from the buffer
     * 
     * @return The next byte, or -1 if no data is available
     */
    virtual int peek() override = 0;
    
    /**
     * @brief Write a byte to the serial port
     * 
     * @param byte The byte to write
     * @return Number of bytes written (1 on success)
     */
    virtual size_t write(uint8_t byte) override = 0;
    
    /**
     * @brief Wait for transmission of outgoing data to complete
     */
    virtual void flush() override = 0;
};

/**
 * @brief Adapter class to simplify using Basicmicro with non-Arduino microcontrollers
 * 
 * This class wraps the Basicmicro library and provides a simpler interface.
 */
class BasicmicroAdapter {
private:
    HardwareSerial* serial;
    uint8_t address;
    
public:
    Basicmicro* controller;

    /**
     * @brief Constructor
     * 
     * @param serial Pointer to your hardware serial implementation
     * @param address Controller address (default 0x80)
     * @param timeout Communication timeout in microseconds (default 10000)
     */
	BasicmicroAdapter(HardwareSerial* serial, uint8_t address = 0x80, uint32_t timeout = 10000)
	{
		this->serial = serial;
		this->address = address;
		this->controller = new Basicmicro(serial, timeout);
	};
    
    /**
     * @brief Destructor
     */
    ~BasicmicroAdapter()
	{
		delete controller;
	};
    
};

#endif // BASICMICRO_ADAPTER_H
