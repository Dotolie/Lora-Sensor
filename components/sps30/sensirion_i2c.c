/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"
#include "sps30.h"

static const char TAG[] = "sps30";

/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx) {
    // IMPLEMENT or leave empty if all sensors are located on one single bus
    return 0;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
esp_err_t sensirion_i2c_init(sps30_config_t *param_ptr_config) {

	esp_err_t f_retval = ESP_OK;

    if (param_ptr_config->manage_i2c_driver == true) {
        i2c_config_t i2c_conf =
                    { 0 };
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.scl_io_num = param_ptr_config->i2c_scl_gpio_num;
        i2c_conf.sda_io_num = param_ptr_config->i2c_sda_gpio_num;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE; // @important
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE; // @important
        i2c_conf.master.clk_speed = SPS30_I2C_MASTER_FREQ_HZ;
        f_retval = i2c_param_config(param_ptr_config->i2c_port_num, &i2c_conf);
        if (f_retval != ESP_OK) {
            ESP_LOGE(TAG, "%s(). ABORT. i2c_param_config() | err %i (%s)", __FUNCTION__, f_retval,
                    esp_err_to_name(f_retval));
            // GOTO
            goto cleanup;
        }
        f_retval = i2c_set_timeout(param_ptr_config->i2c_port_num, SPS30_I2C_SLAVE_TIMEOUT_MAXVAL); // @important clock-strethcing
        if (f_retval != ESP_OK) {
            ESP_LOGE(TAG, "%s(). ABORT. i2c_set_timeout() | err %i (%s)", __FUNCTION__, f_retval,
                    esp_err_to_name(f_retval));
            // GOTO
            goto cleanup;
        }

        f_retval = i2c_driver_install(param_ptr_config->i2c_port_num, I2C_MODE_MASTER, SPS30_I2C_MASTER_RX_BUF_DISABLE,
        	SPS30_I2C_MASTER_TX_BUF_DISABLE, SPS30_I2C_MASTER_INTR_FLAG_NONE);
        if (f_retval != ESP_OK) {
            ESP_LOGE(TAG, "%s(). ABORT. i2c_driver_install() | err %i (%s)", __FUNCTION__, f_retval,
                    esp_err_to_name(f_retval));
            // GOTO
            goto cleanup;
        }
    }
	
cleanup:

	return f_retval;
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
esp_err_t sensirion_i2c_release(sps30_config_t *param_ptr_config) {
    // IMPLEMENT or leave empty if no resources need to be freed
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    esp_err_t f_retval = ESP_OK;


    /*
     * I2C Driver
     */
    if (param_ptr_config->manage_i2c_driver == true) {
        f_retval = i2c_driver_delete(param_ptr_config->i2c_port_num);
        if (f_retval != ESP_OK) {
            ESP_LOGE(TAG, "%s(). ABORT. i2c_driver_delete() | err %i (%s)", __FUNCTION__, f_retval,
                    esp_err_to_name(f_retval));
            // GOTO
            goto cleanup;
        }
    }

    // LABEL
    cleanup: ;

    return f_retval;
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
esp_err_t sensirion_i2c_read(sps30_config_t *param_ptr_config, uint8_t *param_ptr_buf, uint16_t param_buf_len) {
    // IMPLEMENT
    ESP_LOGD(TAG, "%s()", __FUNCTION__);
    ESP_LOGD(TAG, "  param_buf_len: %u", param_buf_len);

    esp_err_t f_retval = ESP_OK;
    i2c_cmd_handle_t handle;

    handle = i2c_cmd_link_create();
    f_retval = i2c_master_start(handle);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_start() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_write_byte(handle, (param_ptr_config->i2c_slave_addr << 1) | I2C_MASTER_READ, true);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_write_byte() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        i2c_cmd_link_delete(handle);
        // GOTO
        goto cleanup;
    }
    // @doc i2c_master_read() param4=I2C_MASTER_LAST_NACK: do ACK for all reads except do NACK for the last read
    f_retval = i2c_master_read(handle, param_ptr_buf, param_buf_len, I2C_MASTER_LAST_NACK);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_read() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        i2c_cmd_link_delete(handle);
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_stop(handle);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_stop() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        i2c_cmd_link_delete(handle);
        // GOTO
        goto cleanup;
    }

    f_retval = i2c_master_cmd_begin(param_ptr_config->i2c_port_num, handle, param_ptr_config->i2c_max_ticks_to_wait);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_cmd_begin() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // Dump received data
    for (uint32_t j = 0; j < param_buf_len; j++) {
        ESP_LOGD(TAG, "  param_ptr_buf[%u]: 0x%02X %3u", j, param_ptr_buf[j], param_ptr_buf[j]);
    }

    // No delay required after I2C Read (device spec)

    // LABEL
    cleanup: ;

    i2c_cmd_link_delete(handle);

    return f_retval;    
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
esp_err_t sensirion_i2c_write(sps30_config_t *param_ptr_config, const uint8_t *param_ptr_buf,
                           uint16_t param_buf_len) {
    // IMPLEMENT
    ESP_LOGD(TAG, "%s()", __FUNCTION__);
    ESP_LOGD(TAG, "  param_buf_len: %u", param_buf_len);

    esp_err_t f_retval = ESP_OK;

    // Dump data to be transmitted
    for (uint32_t j = 0; j < param_buf_len; j++) {
        ESP_LOGD(TAG, "  param_ptr_buf[%u]: 0x%02X %3u", j, param_ptr_buf[j], param_ptr_buf[j]);
    }

    i2c_cmd_handle_t handle;
    handle = i2c_cmd_link_create();
    f_retval = i2c_master_start(handle);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Send request i2c_master_start() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_write_byte(handle, (param_ptr_config->i2c_slave_addr << 1) | I2C_MASTER_WRITE, true);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Send request i2c_master_write_byte() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_write(handle, param_ptr_buf, param_buf_len, true);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Send request i2c_master_write() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_stop(handle);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_stop() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }
    f_retval = i2c_master_cmd_begin(param_ptr_config->i2c_port_num, handle, param_ptr_config->i2c_max_ticks_to_wait);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). ABORT. Receive response i2c_master_cmd_begin() err %i (%s)", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // @important Delay after each I2C Write
    ets_delay_us( 1000 * SPS30_DELAY_MS_AFTER_I2C_WRITE);

    // LABEL
    cleanup: ;

    i2c_cmd_link_delete(handle);

    return f_retval;

}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    // IMPLEMENT
    vTaskDelay( (useconds/1000) / portTICK_PERIOD_MS);
}

