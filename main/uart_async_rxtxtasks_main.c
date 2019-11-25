/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "mjd.h"
#include "mjd_scd30.h"
#include "sps30.h"



static const int RX_BUF_SIZE = 1024;
static const uint32_t NBR_OF_MEASUREMENT_RUNS = 100000; // 1 5 10 100 1000 10000 100000
static const char TAG[] = "myapp";

sps30_config_t sps30_config = SPS30_CONFIG_DEFAULT();


#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define MY_SCD30_I2C_SCL_GPIO_NUM 	22
#define MY_SCD30_I2C_SDA_GPIO_NUM	21




mjd_scd30_data_t scd30_data = { 0 };
struct sps30_measurement m;

void init() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task()
{
    static const char *TX_TASK_TAG = "TX_TASK";
	char cTxBuf[128];

	sendData(TX_TASK_TAG, "$Rpar=\r\n");
    vTaskDelay(RTOS_DELAY_1SEC*200);

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
		sprintf( cTxBuf, "$Tbupc=170:%.0f:%.0f:%.0f:%.0f:%.0f:%.0f:%.0f:%d\r\n", 
				scd30_data.co2_ppm *10,
				scd30_data.relative_humidity *10,
                scd30_data.temperature_celsius*10,
                 m.mc_1p0*100,
                 m.mc_2p5*100,
                 m.mc_4p0*100,
                 m.mc_10p0*100,
                 3900
                 );

        sendData(TX_TASK_TAG, cTxBuf);
		printf("%s", cTxBuf);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

static void rx_task()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

/*
 * TASK
 */
void sps30_task(void *pvParameter) {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    esp_err_t f_retval;
	int16_t ret;

    /*
     * SPS30 INIT
     */

	/* Initialize I2C bus */
	sensirion_i2c_init(&sps30_config);

	/* Busy loop for initialization, because the main loop does not work without
	 * a sensor.
	 */
	while (sps30_probe() != 0) {
		ESP_LOGI(TAG,"SPS sensor probing failed\n");
		vTaskDelay(RTOS_DELAY_1SEC); /* wait 1s */
	}
	ESP_LOGI(TAG,"SPS sensor probing successful\n");

	ret = sps30_start_measurement();
	if (ret < 0)
		ESP_LOGI(TAG,"error starting measurement\n");
	ESP_LOGI(TAG,"measurements started\n");

	while (1) {
		vTaskDelay(RTOS_DELAY_1SEC*SPS30_MEASUREMENT_DURATION_SEC); /* wait 1s */
		ret = sps30_read_measurement(&m);
		if (ret < 0) {
			ESP_LOGI(TAG,"error reading measurement\n");

		} else {
#if 0
			ESP_LOGI(TAG,"measured values:\n"
				   "\t%0.2f pm1.0\n"
				   "\t%0.2f pm2.5\n"
				   "\t%0.2f pm4.0\n"
				   "\t%0.2f pm10.0\n"
				   "\t%0.2f nc0.5\n"
				   "\t%0.2f nc1.0\n"
				   "\t%0.2f nc2.5\n"
				   "\t%0.2f nc4.5\n"
				   "\t%0.2f nc10.0\n"
				   "\t%0.2f typical particle size\n\n",
				   m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
				   m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);
#endif
		ESP_LOGI(TAG,"pm1.0  pm2.5  pm4.0  pm10.0  nc0.5   nc1.0   nc2.5   nc4.5  nc10.0  tp\n"
					 "                   %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f  %0.2f\n",
			   m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
			   m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);

		}
	}





	vTaskDelete(NULL);
}


/*
 * TASK
 */
void scd30_task(void *pvParameter) {
    ESP_LOGD(TAG, "%s()", __FUNCTION__);

    esp_err_t f_retval;

    /*
     * SCD30 INIT
     */
    ESP_LOGI(TAG, "do mjd_scd30_init()");

    // @important Do not use ={} or ={0}
    mjd_scd30_config_t scd30_config = MJD_SCD30_CONFIG_DEFAULT();

//    scd30_config.i2c_slave_addr = MY_SCD30_I2C_SLAVE_ADDRESS;
//    scd30_config.i2c_port_num = MY_SCD30_I2C_MASTER_PORT_NUM;
    scd30_config.i2c_scl_gpio_num = MY_SCD30_I2C_SCL_GPIO_NUM;
    scd30_config.i2c_sda_gpio_num = MY_SCD30_I2C_SDA_GPIO_NUM;

    f_retval = mjd_scd30_init(&scd30_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). mjd_scd30_init() err %i %s", __FUNCTION__, f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    /*
     * LOG DEVICE PARAMS (read registers)
     */
    f_retval = mjd_scd30_log_device_parameters(&scd30_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). mjd_scd30_log_device_parameters() err %i %s", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    /*
     * Start Continuous Measurement @ 0mBar
     * @doc Setting the argument to zero will deactivate the ambient pressure compensation.
     *
     * @examples param MJD_SCD30_AMBIENT_PRESSURE_DISABLED | 1020 (the average barometric pressure in Antwerp is 1020mBar).
     */
    ESP_LOGI(TAG, "  mjd_scd30_cmd_trigger_continuous_measurement()...");
    f_retval = mjd_scd30_cmd_trigger_continuous_measurement(&scd30_config, 1020);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). mjd_scd30_cmd_trigger_continuous_measurement() err %i %s", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    // DEVTEMP
    /////mjd_rtos_wait_forever();

    /*
     * LOOP
     */

    double sum_co2 = 0;
    double min_co2 = FLT_MAX;
    double max_co2 = FLT_MIN;
    double sum_rh = 0;
    double min_rh = FLT_MAX;
    double max_rh = FLT_MIN;
    double sum_tc = 0;
    double min_tc = FLT_MAX;
    double max_tc = FLT_MIN;
    double sum_tf = 0;
    double min_tf = FLT_MAX;
    double max_tf = FLT_MIN;
    double sum_dpc = 0;
    double min_dpc = FLT_MAX;
    double max_dpc = FLT_MIN;
    double sum_dpf = 0;
    double min_dpf = FLT_MAX;
    double max_dpf = FLT_MIN;

    uint32_t nbr_of_valid_runs = 0;
    uint32_t nbr_of_errors = 0;

    mjd_log_time();

    ESP_LOGI(TAG, "LOOP: NBR_OF_MEASUREMENT_RUNS %u", NBR_OF_MEASUREMENT_RUNS);

    for (uint32_t j = 1; 1; ++j) {
        ESP_LOGI(TAG, "  ***SCD30 MEAS#%u of %u***...", j, NBR_OF_MEASUREMENT_RUNS);

        // GET "Data Ready Status" until := YES or timeout
        uint32_t max_nbr_of_status_checks;
        max_nbr_of_status_checks = scd30_config.measurement_interval*3; // @example 30 = 30 seconds in combination with vTaskDelay(RTOS_DELAY_1SEC)
        mjd_scd30_data_ready_status_t data_ready_status = MJD_SCD30_DATA_READY_STATUS_NO;
        while ((data_ready_status != MJD_SCD30_DATA_READY_STATUS_YES) && (max_nbr_of_status_checks > 0)) {
            f_retval = mjd_scd30_cmd_get_data_ready_status(&scd30_config, &data_ready_status);
            if (f_retval != ESP_OK) {
                ++nbr_of_errors;
                ESP_LOGE(TAG, "%s(). ABORT. mjd_scd30_cmd_get_data_ready_status() failed | err %i (%s)", __FUNCTION__,
                        f_retval,
                        esp_err_to_name(f_retval));
                // CONTINUE (not BREAK!)
                continue;
            }
            ESP_LOGD(TAG, "    ... data_ready_status: %u ...", data_ready_status);
            --max_nbr_of_status_checks;
            vTaskDelay(RTOS_DELAY_1SEC); // @important Do not change because it correlates to max_nbr_of_status_checks!
        }
        ESP_LOGI(TAG, "    ...data_ready_status: %u", data_ready_status);

        // Check no timeout
        if (data_ready_status != MJD_SCD30_DATA_READY_STATUS_YES) {
            f_retval = ESP_ERR_TIMEOUT;
            ++nbr_of_errors;
            ESP_LOGE(TAG, "%s(). ABORT. Timeout checking mjd_scd30_cmd_get_data_ready_status() too many times | err %i (%s)",
                    __FUNCTION__,
                    f_retval,
                    esp_err_to_name(f_retval));
            // CONTINUE (not BREAK!)
            continue;
        }


        ESP_LOGI(TAG, "  mjd_scd30_cmd_read_measurement...");
        f_retval = mjd_scd30_cmd_read_measurement(&scd30_config, &scd30_data);
        if (f_retval != ESP_OK) {
            ++nbr_of_errors;
            ESP_LOGE(TAG, "%s(). Cannot read measurement | err %i (%s)", __FUNCTION__, f_retval,
                    esp_err_to_name(f_retval));
            // CONTINUE (not BREAK!)
            continue;
        }

        ESP_LOGD(TAG, "    Dump raw data");
        for (uint32_t k = 0; k < ARRAY_SIZE(scd30_data.raw_data); k++) {
            ESP_LOGD(TAG, "      scd30_data->raw_data[%u]: 0x%04X %4u", k, scd30_data.raw_data[k], scd30_data.raw_data[k]);
        }

        ESP_LOGI(TAG, "    CO2: %6.1f | Temp C: %6.1f | Temp F: %6.1f | RelHum: %6.1f | DewPnt C: %6.1f | DewPnt F: %6.1f",
                scd30_data.co2_ppm,
                scd30_data.temperature_celsius, scd30_data.temperature_fahrenheit,
                scd30_data.relative_humidity,
                scd30_data.dew_point_celsius, scd30_data.dew_point_fahrenheit);
        ESP_LOGI(TAG, "    EU IDA Air Quality Category: %u - %s (%s)", scd30_data.eu_ida_category,
                scd30_data.eu_ida_category_code,
                scd30_data.eu_ida_category_desc);


        // STATS
        sum_co2 += scd30_data.co2_ppm;
        if (scd30_data.co2_ppm < min_co2) {
            min_co2 = scd30_data.co2_ppm;
        }
        if (scd30_data.co2_ppm > max_co2) {
            max_co2 = scd30_data.co2_ppm;
        }

        sum_tc += scd30_data.temperature_celsius;
        if (scd30_data.temperature_celsius < min_tc) {
            min_tc = scd30_data.temperature_celsius;
        }
        if (scd30_data.temperature_celsius > max_tc) {
            max_tc = scd30_data.temperature_celsius;
        }

        sum_tf += scd30_data.temperature_fahrenheit;
        if (scd30_data.temperature_fahrenheit < min_tf) {
            min_tf = scd30_data.temperature_fahrenheit;
        }
        if (scd30_data.temperature_fahrenheit > max_tf) {
            max_tf = scd30_data.temperature_fahrenheit;
        }

        sum_rh += scd30_data.relative_humidity;
        if (scd30_data.relative_humidity < min_rh) {
            min_rh = scd30_data.relative_humidity;
        }
        if (scd30_data.relative_humidity > max_rh) {
            max_rh = scd30_data.relative_humidity;
        }

        sum_dpc += scd30_data.dew_point_celsius;
        if (scd30_data.dew_point_celsius < min_dpc) {
            min_dpc = scd30_data.dew_point_celsius;
        }
        if (scd30_data.dew_point_celsius > max_dpc) {
            max_dpc = scd30_data.dew_point_celsius;
        }

        sum_dpf += scd30_data.dew_point_fahrenheit;
        if (scd30_data.dew_point_fahrenheit < min_dpf) {
            min_dpf = scd30_data.dew_point_fahrenheit;
        }
        if (scd30_data.dew_point_fahrenheit > max_dpf) {
            max_dpf = scd30_data.dew_point_fahrenheit;
        }

        nbr_of_valid_runs++; // For calculating correct averages.


        // @optional A visual delay between reading loop items, is sometimes easier when debugging
        /////vTaskDelay(RTOS_DELAY_1SEC);

    }

    ESP_LOGI(TAG, "REPORT:");
    ESP_LOGI(TAG, "  NBR_OF_MEASUREMENT_RUNS: %u", NBR_OF_MEASUREMENT_RUNS);
    ESP_LOGI(TAG, "  nbr_of_valid_runs:       %u", nbr_of_valid_runs);
    ESP_LOGI(TAG, "  nbr_of_errors:           %u", nbr_of_errors);
    ESP_LOGI(TAG, "    METRIC                        avg        min        max");
    ESP_LOGI(TAG, "    ------                 ---------- ---------- ----------");
    ESP_LOGI(TAG, "    CO2 ppm                %10.3f %10.3f %10.3f", sum_co2 / nbr_of_valid_runs, min_co2, max_co2);
    ESP_LOGI(TAG, "    Temperature Celsius    %10.3f %10.3f %10.3f", sum_tc / nbr_of_valid_runs, min_tc, max_tc);
    ESP_LOGI(TAG, "    Temperature Fahrenheit %10.3f %10.3f %10.3f", sum_tf / nbr_of_valid_runs, min_tf, max_tf);
    ESP_LOGI(TAG, "    Relative Humidity      %10.3f %10.3f %10.3f", sum_rh / nbr_of_valid_runs, min_rh, max_rh);
    ESP_LOGI(TAG, "    Dew Point Celsius      %10.3f %10.3f %10.3f", sum_dpc / nbr_of_valid_runs, min_dpc, max_dpc);
    ESP_LOGI(TAG, "    Dew Point Fahrenheit   %10.3f %10.3f %10.3f", sum_dpf / nbr_of_valid_runs, min_dpf, max_dpf);

    /*
     * STOP CONTINUOUS MEASUREMENT
     */
    ESP_LOGI(TAG, "  mjd_scd30_cmd_stop_continuous_measurement()...");
    f_retval = mjd_scd30_cmd_stop_continuous_measurement(&scd30_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). mjd_scd30_cmd_stop_continuous_measurement() err %i %s", __FUNCTION__, f_retval,
                esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    /*
     * DEVICE DE-INIT
     */
    ESP_LOGI(TAG, "  mjd_scd30_deinit()...");
    f_retval = mjd_scd30_deinit(&scd30_config);
    if (f_retval != ESP_OK) {
        ESP_LOGE(TAG, "%s(). mjd_scd30_deinit() | err %i (%s)", __FUNCTION__, f_retval, esp_err_to_name(f_retval));
        // GOTO
        goto cleanup;
    }

    cleanup: ;

    mjd_log_time();

    /********************************************************************************
     * Task Delete
     * @doc Passing NULL will end the current task
     *
     */
    vTaskDelete(NULL);
}


void app_main()
{

	BaseType_t xReturned;

    /* SOC init */
    ESP_LOGI(TAG, "@doc exec nvs_flash_init() - mandatory for Wifi to work later on");
    nvs_flash_init();

    /********************************************************************************
     * MY STANDARD Init
     *
     */
    mjd_log_chip_info();
    mjd_log_memory_statistics();
//    mjd_set_timezone_utc();
//    mjd_log_time();
    ESP_LOGI(TAG,
            "@tip You can also change the log level to DEBUG for more detailed logging and to get insights in what the component is actually doing.");
    ESP_LOGI(TAG, "@doc Wait 2 seconds after power-on (start logic analyzer, let peripherals become active, ...)");
    vTaskDelay(RTOS_DELAY_2SEC);

    /*
     * Sensor Task
     */

    xReturned = xTaskCreatePinnedToCore(&scd30_task, "scd30_task (name)", 8192, NULL,
    RTOS_TASK_PRIORITY_NORMAL,
    NULL,
    APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK SCD30 Task has been created, and is running right now");
    }
#if 1
    xReturned = xTaskCreatePinnedToCore(&sps30_task, "sps30_task (name)", 8192, NULL,
    RTOS_TASK_PRIORITY_NORMAL,
    NULL,
    APP_CPU_NUM);
    if (xReturned == pdPASS) {
        ESP_LOGI(TAG, "OK SPS30 Task has been created, and is running right now");
    }
#endif
    init();
	
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
