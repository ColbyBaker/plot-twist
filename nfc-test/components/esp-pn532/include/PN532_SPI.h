
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__


#include "PN532Interface.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

class PN532_SPI : public PN532Interface {
public:
    PN532_SPI(gpio_num_t cs_pin, spi_host_device_t spi_host);
    
    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);

    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);
    
private:
    // SPIClass* _spi;
    gpio_num_t chip_select_pin;
    spi_host_device_t spi_host_device;
    uint8_t command;
    spi_device_handle_t spi_handle;
    
    bool isReady();
    // void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int8_t readAckFrame();
    void log_binary(uint8_t value);
    // inline void write(uint8_t data) {
    //     _spi->transfer(data);
    // };

    // inline uint8_t read() {
    //     return _spi->transfer(0);
    // }; 
};

#endif