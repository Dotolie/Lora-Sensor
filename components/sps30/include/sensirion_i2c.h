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

#ifndef SENSIRION_I2C_H
#define SENSIRION_I2C_H

#include "sensirion_arch_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */




#define SPS30_MAX_SERIAL_LEN 32
#define SPS30_MEASUREMENT_DURATION_SEC 60 /* 1s measurement intervals */
#define SPS30_RESET_DELAY_USEC 50000 /* 50ms delay after resetting the sensor */

/**
 * I2C Settings
 *
 * - Support I2C slaves that use clock-stretching
 *     @doc APB_CLK 80Mhz 80 MHz = 12.5 nanosec per tick
 *            default:   32000  =>  0.4 millisec
 *            test:     128000      1.6 millisec
 *            maxval:  1048575     13.1 millisec
 *
 * @doc Maximal I2C speed is 100 kHz.
 * @doc SCD30 does not support I2C repeated starts!
 * @doc The I2C master must support clock stretching.
 *      Clock stretching period in write- and read frames is 12 ms, however,
 *      due to internal calibration processes a maximal clock stretching of 150 ms may occur once per day!
 */
#define SPS30_I2C_ADDRESS_DEFAULT           (0x69)       /*!< Cannot be changed... */
#define SPS30_I2C_MASTER_NUM_DEFAULT        (I2C_NUM_1)  /*!< The default I2C_NUM_0 can be changed. */
#define SPS30_I2C_MASTER_FREQ_HZ            (100 * 1000) /*!< Default 100 * 1000 [ESP32 Max 1 MHz.] Use 10Khz for long wires (>25cm). I2C master clock freq: Normal (100 KHz), FastMode (400 Khz), FastModePlus (1 Mhz). */
#define SPS30_I2C_MASTER_RX_BUF_DISABLE     (0)          /*!< I2C master does not need RX buffer. This param is for I2C slaves. */
#define SPS30_I2C_MASTER_TX_BUF_DISABLE     (0)          /*!< I2C master does not need TX buffer. This param is for I2C slaves. */
#define SPS30_I2C_MASTER_INTR_FLAG_NONE     (0)
#define SPS30_I2C_MAX_TICKS_TO_WAIT_DEFAULT (1000 / portTICK_PERIOD_MS) /*!< 1 sec */
#define SPS30_I2C_SLAVE_TIMEOUT_MAXVAL      (1048575)    /*!< I2C clock-stretching. See docs above. */
#define SPS30_I2C_SDA_GPIO_NUM				(18)
#define SPS30_I2C_SCL_GPIO_NUM				(19)

#define SPS30_MEASUREMENT_INTERVAL_DEFAULT  (1) 
#define SPS30_DELAY_MS_AFTER_I2C_WRITE 		(30) 


typedef struct {
        bool manage_i2c_driver;
        i2c_port_t i2c_port_num;
        uint8_t i2c_slave_addr;
        gpio_num_t i2c_scl_gpio_num;
        gpio_num_t i2c_sda_gpio_num;
        int i2c_max_ticks_to_wait;

        uint16_t measurement_interval;

} sps30_config_t;

#define SPS30_CONFIG_DEFAULT()	{\
    .manage_i2c_driver = true, \
    .i2c_port_num = SPS30_I2C_MASTER_NUM_DEFAULT, \
    .i2c_slave_addr = SPS30_I2C_ADDRESS_DEFAULT, \
    .i2c_scl_gpio_num = SPS30_I2C_SCL_GPIO_NUM, \
    .i2c_sda_gpio_num = SPS30_I2C_SDA_GPIO_NUM, \
    .i2c_max_ticks_to_wait = SPS30_I2C_MAX_TICKS_TO_WAIT_DEFAULT, \
    .measurement_interval = SPS30_MEASUREMENT_INTERVAL_DEFAULT \
};

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
int16_t sensirion_i2c_select_bus(uint8_t bus_idx);

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
esp_err_t sensirion_i2c_init(sps30_config_t *param_ptr_config);

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
esp_err_t sensirion_i2c_release(sps30_config_t *param_ptr_config);

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
esp_err_t sensirion_i2c_read(sps30_config_t *param_ptr_config, uint8_t *data, uint16_t count);

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
esp_err_t sensirion_i2c_write(sps30_config_t *param_ptr_config, const uint8_t *data,
                           uint16_t count);

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution approximately, but no less than, the given time.
 *
 * When using hardware i2c:
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * When using software i2c:
 * The precision needed depends on the desired i2c frequency, i.e. should be
 * exact to about half a clock cycle (defined in
 * `SENSIRION_I2C_CLOCK_PERIOD_USEC` in `sensirion_arch_config.h`).
 *
 * Example with 400kHz requires a precision of 1 / (2 * 400kHz) == 1.25usec.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SENSIRION_I2C_H */

