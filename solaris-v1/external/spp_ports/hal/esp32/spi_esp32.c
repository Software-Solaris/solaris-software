#include "spi.h" 
#include "types.h"
#include "esp_log.h" 
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



static const char *TAG = "SPP_HAL_SPI";

retval_t SPP_SPI_Transmit(void* handler, void* data_to_send, void* data_to_recieve, spp_uint8_t length) {
    spi_device_handle_t p_handler = (spi_device_handle_t) handler;
    esp_err_t trans_result = ESP_OK;

    if (length <= 2) {
        // Only one transmission
        spi_transaction_t trans_desc = {0};
        trans_desc.length    = 8 * length;
        trans_desc.tx_buffer = data_to_send;
        trans_desc.rx_buffer = data_to_recieve;

        trans_result = spi_device_transmit(p_handler, &trans_desc);

    } else {
        spp_uint8_t* p_tx = (spp_uint8_t*) data_to_send;
        spp_uint8_t* p_rx = (spp_uint8_t*) data_to_recieve;
        
        int num_ops = length / 2;
        spi_transaction_t transactions[num_ops];

        for (int i = 0; i < num_ops; i++) {
            spi_transaction_t* p_transaction = &transactions[i];
            *p_transaction = (spi_transaction_t){0}; //We put all the struct to zero
            p_transaction->length    = 16; 
            p_transaction->tx_buffer = &p_tx[i * 2];
            p_transaction->rx_buffer = &p_rx[i * 2];

            trans_result = spi_device_queue_trans(p_handler, p_transaction, portMAX_DELAY);
            if (trans_result != ESP_OK) break;
        }

        for (int i = 0; i < num_ops && trans_result == ESP_OK; i++) {
            spi_transaction_t* ret_t;
            trans_result = spi_device_get_trans_result(p_handler, &ret_t, portMAX_DELAY);
            if (trans_result != ESP_OK) break;
        }
    }

    if (trans_result != ESP_OK) {
        return SPP_ERROR;
    }
    return SPP_OK;
}


