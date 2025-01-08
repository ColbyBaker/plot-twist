
#include "PN532_SPI.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "PN532_SPI"
#define STATUS_READ     2
#define DATA_WRITE      1
#define DATA_READ       3


PN532_SPI::PN532_SPI(gpio_num_t cs_pin, spi_host_device_t spi_host) : chip_select_pin(cs_pin), spi_host_device(spi_host)
{
    ESP_LOGI(TAG, "GPIO number: %d", static_cast<int>(chip_select_pin));
    gpio_config_t io_conf;

    // Configure GPIO18 as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << chip_select_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_set_level(chip_select_pin, 1);  // Set CS pin HIGH (inactive)

    command = 0;
}

void PN532_SPI::begin()
{

spi_device_interface_config_t devcfg = {
    .command_bits = 0,           // Number of bits in command phase
    .address_bits = 0,           // Number of bits in address phase
    .dummy_bits = 0,             // Number of dummy bits
    .mode = 0,                   // SPI mode 0
    .duty_cycle_pos = 128,       // Default duty cycle (50%)
    .cs_ena_pretrans = 0,        // Chip select enable before transmission
    .cs_ena_posttrans = 0,       // Chip select enable after transmission
    .clock_speed_hz = 2 * 1000 * 1000,   // Clock speed in Hz (1 MHz)
    .input_delay_ns = 0,         // No input delay
    // .spics_io_num = chip_select_pin,  // GPIO pin for chip select
    .spics_io_num = -1,
    .flags = SPI_DEVICE_BIT_LSBFIRST,
    .queue_size = 1,             // Queue size for transactions
    .pre_cb = NULL,              // Pre-transaction callback
    .post_cb = NULL              // Post-transaction callback
};

esp_err_t ret = spi_bus_add_device(spi_host_device, &devcfg, &spi_handle);
assert(ret == ESP_OK);

}

void PN532_SPI::wakeup()
{
    gpio_set_level(chip_select_pin, 0);
    vTaskDelay(1);
    gpio_set_level(chip_select_pin, 1);
    vTaskDelay(1);
}



int8_t PN532_SPI::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    gpio_set_level(chip_select_pin, 0);
    vTaskDelay(1);

    command = header[0];
    uint8_t* combinedBuffer = (uint8_t*)malloc(hlen + blen + 9);
    if (!combinedBuffer) {
        ESP_LOGE(TAG, "Memory allocation failure in write command");
    }
    combinedBuffer[0] = DATA_WRITE;
    combinedBuffer[1] = PN532_PREAMBLE;
    combinedBuffer[2] = PN532_STARTCODE1;
    combinedBuffer[3] = PN532_STARTCODE2;
    uint8_t length = hlen + blen + 1;
    combinedBuffer[4] = length;
    uint8_t length_checksum = ~length + 1;
    combinedBuffer[5] = length_checksum;
    combinedBuffer[6] = PN532_HOSTTOPN532;

    uint8_t sum = PN532_HOSTTOPN532;
    memcpy(combinedBuffer + 7, header, hlen);
    memcpy(combinedBuffer + hlen + 7, body, blen);
    uint8_t lengthForSum = hlen + blen;
    for (size_t i = 7; i < lengthForSum + 7; i++) {
        sum += combinedBuffer[i];
    }
    uint8_t checksum = ~sum + 1;
    combinedBuffer[7 + blen + hlen] = checksum;
    combinedBuffer[7 + blen + hlen + 1] = PN532_POSTAMBLE;

    size_t transaction_length = (hlen + blen + 9) * 8;
    spi_transaction_t main_transaction = {
        .length = transaction_length,
        .tx_buffer = combinedBuffer,
    };

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &main_transaction);
    assert(ret == ESP_OK);
    free(combinedBuffer);

    gpio_set_level(chip_select_pin, 1);
    vTaskDelay(1);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    while (!isReady()) {
        vTaskDelay(1);
        timeout--;
        if (0 == timeout) {
            ESP_LOGE(TAG, "Time out when waiting for ACK");
            return -2;
        }
    }

    if (readAckFrame()) {
        ESP_LOGE(TAG, "Invalid ACK");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t PN532_SPI::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    //eventually will need to refactor for half-duplex transactions. Transaction length may be affected.
    uint16_t time = 0;
    while (!isReady()) {
        vTaskDelay(1);
        time++;
        if (timeout > 0 && time > timeout) {
            return PN532_TIMEOUT;
        }
    }

    gpio_set_level(chip_select_pin, 0);
    vTaskDelay(1);

    int16_t result = 0;
    size_t transaction_length = 6;
    uint8_t txBuffer_1[6] = {DATA_READ};
    uint8_t rxBuffer_1[6];
    memset(rxBuffer_1, 0, 6);

    spi_transaction_t transaction_1 = {};
    transaction_1.length = transaction_length * 8; // In bits
    transaction_1.tx_buffer = txBuffer_1;
    transaction_1.rx_buffer = rxBuffer_1;
    // transaction_1.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &transaction_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed");
        return PN532_INVALID_FRAME;
    }

    //after first transaction

    // ESP_LOGW(TAG, "LENGTH FROM DEVICE %d", rxBuffer_1[4]);

    if (rxBuffer_1[1] != 0x00 || rxBuffer_1[2] != 0x00 || rxBuffer_1[3] != 0xFF) {
        result = PN532_INVALID_FRAME;
        ESP_LOGE(TAG, "PN532_INVALID_FRAME_1");
        return result;
    }

    // Length and checksum validation
    uint8_t length = rxBuffer_1[4];
    if (rxBuffer_1[5] != (uint8_t)(~length + 1)) {
        result = PN532_INVALID_FRAME;
        ESP_LOGE(TAG, "PN532_INVALID_FRAME_2");
        return result;
    }

    size_t transaction_2_length = (size_t)rxBuffer_1[4];
    transaction_2_length += 2; //postamble and checksum
    uint8_t* txBuffer_2 = (uint8_t *)malloc(transaction_2_length);
    uint8_t* rxBuffer_2 = (uint8_t *)malloc(transaction_2_length);
    memset(txBuffer_2, 0, transaction_2_length);
    memset(rxBuffer_2, 0, transaction_2_length);   
    //start second transaction
    spi_transaction_t transaction_2 = {};
    transaction_2.length = transaction_2_length * 8; // In bits
    transaction_2.tx_buffer = txBuffer_2;
    transaction_2.rx_buffer = rxBuffer_2;

    ret = spi_device_polling_transmit(spi_handle, &transaction_2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed");
        spi_device_release_bus(spi_handle);
        return PN532_INVALID_FRAME;
    }
    spi_device_release_bus(spi_handle);


    // Validate command
    uint8_t cmd = command + 1; // Response command
    if (rxBuffer_2[0] != PN532_PN532TOHOST || rxBuffer_2[1] != cmd) {
        result = PN532_INVALID_FRAME;
        ESP_LOGE(TAG, "PN532_INVALID_FRAME_3");
        return result;
    }

    // ESP_LOGI(TAG, "read: 0x%02X", cmd);

    // Calculate length of data and validate buffer size
    length -= 2; // Exclude TFI and command
    if (length > len) {
        ESP_LOGE(TAG, "Not enough space to store the response...might not be true");
        result = PN532_NO_SPACE;
        return result;
    }

    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length; i++) {
        buf[i] = rxBuffer_2[i + 2];
        sum += buf[i];
        ESP_LOGD(TAG, "Data[%d]: 0x%02X", i, buf[i]);
    }


    free(txBuffer_2);
    free(rxBuffer_2);

    gpio_set_level(chip_select_pin, 1);
    vTaskDelay(1);
    return result;
}

bool PN532_SPI::isReady()
{

    gpio_set_level(chip_select_pin, 0);
    vTaskDelay(1);

    uint8_t status;
    uint8_t txBuffer[2] = {STATUS_READ, 0x00};  // STATUS_READ as the first byte
    uint8_t rxBuffer[2] = {0};                 // Receive buffer initialized to 0

    spi_transaction_t status_check = {
        .length = 16,
        .tx_buffer = &txBuffer,
        .rx_buffer = &rxBuffer,
    };
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &status_check);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI transmission failed: %s", esp_err_to_name(ret));
        return 0; // Treat as "not ready" on error
    }
    // Extract and return the least significant bit (LSB) of the received byte
    // log_binary(rxBuffer[1]);
    status = rxBuffer[1] & 1;
    // log_binary(status);

    gpio_set_level(chip_select_pin, 1);
    vTaskDelay(1);

    return status;
}

int8_t PN532_SPI::readAckFrame()
{
    gpio_set_level(chip_select_pin, 0);
    vTaskDelay(1);

    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
    uint8_t ackBuf[sizeof(PN532_ACK)];

    size_t transaction_length = (sizeof(PN532_ACK) + 1) * 8;
    // Prepare SPI buffers for sending and receiving
    uint8_t txBuffer[7] = {DATA_READ};    // Command to read data and an empty array afterwards
    uint8_t rxBuffer[7];                // To store the received data

    // Create the SPI transaction
    spi_transaction_t read_command = {
        .length = transaction_length,                  // 1 byte per transfer
        .tx_buffer = &txBuffer,       // Pointer to transmit buffer
        .rx_buffer = &rxBuffer        // Pointer to receive buffer
    };

    // Start the SPI transaction
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &read_command);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "SPI transmission failed: %s", esp_err_to_name(ret));
        return -1; // Return error if transmission failed
    }

    if (memcmp(&rxBuffer[1], PN532_ACK, sizeof(PN532_ACK)) == 0) {
        gpio_set_level(chip_select_pin, 1);
        vTaskDelay(1);
        return 0; // ACK received correctly
    } else {
        ESP_LOGE(TAG, "Invalid ACK received");
        return -1; // ACK mismatch
    }
}

void PN532_SPI::log_binary(uint8_t value) {
    char bin_str[9];  // 8 bits + 1 for null terminator
    for (int i = 7; i >= 0; i--) {
        bin_str[7 - i] = (value & (1 << i)) ? '1' : '0';
    }
    bin_str[8] = '\0';  // Null-terminate the string
    ESP_LOGI("TAG", "Binary: %s", bin_str);  // Log the binary string
}

// static void spi_pre_transfer_callback(spi_transaction_t *trans) {
//     // Assuming chip_select_pin is used for CS
//     gpio_set_level(chip_select_pin, 0);
// }

// static void spi_post_transfer_callback(spi_transaction_t *trans) {
//     // Assuming chip_select_pin is used for CS
//     gpio_set_level(chip_select_pin, 1);
// }