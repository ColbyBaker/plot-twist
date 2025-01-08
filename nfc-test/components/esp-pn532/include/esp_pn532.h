#pragma once

#include <PN532_SPI.h>
#include <PN532.h>

class ESP_PN532 final {
    public:
        ESP_PN532(gpio_num_t cs_pin, spi_host_device_t spi_host);
        void setup();
        void read_card();
        int read_uid(uint8_t* uid_buffer);
        void format_card();

    private:
        PN532_SPI pn532_spi;               // I2C interface
        PN532 nfc;                         // NFC object
        void log_hex(uint8_t value);
};