#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_pn532.h"

#define TAG "main"
#define number_of_nfc_readers 4

#ifdef __cplusplus
}
#endif

ESP_PN532 pn532_0(GPIO_NUM_22, SPI2_HOST);
ESP_PN532 pn532_1(GPIO_NUM_5, SPI2_HOST);
ESP_PN532 pn532_2(GPIO_NUM_2, SPI2_HOST);
ESP_PN532 pn532_3(GPIO_NUM_4, SPI2_HOST);

static SemaphoreHandle_t button_semaphore = NULL;
TaskHandle_t button_task_handle = NULL;

//three nested arrays, two to store responses from nfc reader chips and the last to provide a state for the application.
// The state is a result of comparison between the most recent and second most recent scane of nfc tags. This allows for a nfc card to
// miss detection or have a misread for one cycle of checks.
uint8_t** second_last_scan_tags[number_of_nfc_readers][7];
uint8_t** last_scan_tags[number_of_nfc_readers][7];
uint8_t** tag_state[number_of_nfc_readers][7];


// Private function declarations

void log_hex(uint8_t value) {
  printf("0x%02X ", value);
}


void initialize_pn532_spi(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_NUM_23,  // GPIO pin for MOSI
        .miso_io_num = GPIO_NUM_19,  // GPIO pin for MISO
        .sclk_io_num = GPIO_NUM_15,  // GPIO pin for SCLK
        .quadwp_io_num = -1,         // No Quad SPI WP pin
        .quadhd_io_num = -1,         // No Quad SPI HD pin
        .data4_io_num = -1,          // Default for non-quad
        .data5_io_num = -1,          // Default for non-quad
        .data6_io_num = -1,          // Default for non-quad
        .data7_io_num = -1,          // Default for non-quad
        .max_transfer_sz = 0,        // Use default maximum transfer size
        .flags = 0,                  // Default flags
        .intr_flags = 0              // Default interrupt flags
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(button_semaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // This might be needed to force a context switch if a higher priority task is now ready
    }
}

void task_to_control(void *pvParameters) {
    for(;;) {
        // Wait for the semaphore to be available before proceeding
        if(xSemaphoreTake(button_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Semaphore taken, executing task");
            uint8_t receive_buffer[] = {0, 0, 0, 0, 0, 0, 0};
            ESP_LOGI(TAG, "Scanning cards");
            pn532_0.read_uid(receive_buffer);
            for (int x = 0; x < 7; x++) {
                log_hex(receive_buffer[x]);
            }
            ESP_LOGI(TAG, "tag: 0");
            pn532_1.read_uid(receive_buffer);
            for (int x = 0; x < 7; x++) {
                log_hex(receive_buffer[x]);
            }
            ESP_LOGI(TAG, "tag: 1");
            pn532_2.read_uid(receive_buffer);
            for (int x = 0; x < 7; x++) {
                log_hex(receive_buffer[x]);
            }
            ESP_LOGI(TAG, "tag: 2");
            pn532_3.read_uid(receive_buffer);
            for (int x = 0; x < 7; x++) {
                log_hex(receive_buffer[x]);
            }
            ESP_LOGI(TAG, "tag: 3");
        } else {
            ESP_LOGE(TAG, "Failed to take semaphore");
        }
    }
}

int initialize_nfc_readers(void) {

    // for (int i = 0; i < number_of_nfc_readers; i++) {
    //     second_last_scan_tags[i] = (uint8_t*)calloc(7, sizeof(uint8_t));
    //     last_scan_tags[i] = (uint8_t*)calloc(7, sizeof(uint8_t));
    //     tag_state[i] = (uint8_t*)calloc(7, sizeof(uint8_t));
        
    //     if (second_last_scan_tags[i] == NULL || last_scan_tags[i] == NULL || tag_state[i] == NULL) {
    //         // Handle memory allocation error
    //         ESP_LOGE(TAG, "Failure for memory allocation of nfc tag arrays");
    //     }
    // }   

    button_semaphore = xSemaphoreCreateBinary();
    if(button_semaphore == NULL) {
        ESP_LOGE(TAG, "button_semaphore creation failed");
        return 0;
    }

    xTaskCreate(
                    task_to_control, /* Function that implements the task. */
                    "Button Task", /* Text name for the task. */
                    2048,            /* Stack size in words, not bytes. */
                    NULL,            /* Parameter passed into the task. */
                    tskIDLE_PRIORITY + 3,/* Priority at which the task is created. */
                    &button_task_handle );


    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Trigger on rising edge
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_17);
    
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(0);
    
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_NUM_17, gpio_isr_handler, NULL);





    initialize_pn532_spi();
    ESP_LOGI(TAG, "About to check firmware version for 0");
    pn532_0.setup();

    ESP_LOGI(TAG, "About to check firmware version for 1");
    pn532_1.setup();
    // pn532_1.read_uid();

    ESP_LOGI(TAG, "About to check firmware version for 2");
    pn532_2.setup();
    // pn532_2.read_uid();

    ESP_LOGI(TAG, "About to check firmware version for 3");
    pn532_3.setup();
    // pn532_3.read_uid();

    //todo add error handling
    return 1;
}

int update_tag_state(void) {
    // for (int i = 0; i < number_of_nfc_readers; i++) {
    //     memcpy(second_last_scan_tags[i], last_scan_tags[i], sizeof(uint8_t) * 7);
    // }
    // todo add error handling
    return 1;
}


extern "C" void app_main(void)
{
    initialize_nfc_readers();
    while(1){
        vTaskDelay(10);
    }
}