#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "nvs_flash.h"

static const char *TAG = "BME280 - DEMO";

#define I2C_MASTER_SCL_IO           GPIO_NUM_22  // SCL pin
#define I2C_MASTER_SDA_IO           GPIO_NUM_21  // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0 // I2C bus number
#define I2C_MASTER_FREQ_HZ          1000000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0      /*!< I2C master doesn't need buffer */


/* The variable used to assign the standby time*/
u8  v_stand_by_time_u8 = 0;
/* The variable used to read uncompensated temperature*/
s32 v_data_uncomp_temp_s32 = 0;
/* The variable used to read uncompensated pressure*/
s32 v_data_uncomp_pres_s32 = 0;
/* The variable used to read uncompensated humidity*/
s32 v_data_uncomp_hum_s32 = 0;
/* The variable used to read compensated temperature*/
s32 v_comp_temp_s32[2] = {0, 0};
/* The variable used to read compensated pressure*/
u32 v_comp_press_u32[2] = {0, 0};
/* The variable used to read compensated humidity*/
u32 v_comp_humidity_u32[2] = {0, 0};

/* result of communication results*/
s32 com_rslt = ERROR;

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define FAIL -1

// Initialize I2C communication parameters
void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1)
	{
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C delay function
void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

static void start_bme280() {

    static struct bme280_t bme280_device= {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,
		.delay_msec = BME280_delay_msek
    };
    static struct bme280_t *bme280 = &bme280_device;    

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	// Initialize BME280 sensor and set internal parameters
	com_rslt = bme280_init(bme280);

	// Set oversampling for humidity, pressure, and temperature
	com_rslt += bme280_set_oversamp_pressure(bme280, BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(bme280, BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(bme280, BME280_OVERSAMP_1X);
	if (com_rslt != ESP_OK) { ESP_LOGE(TAG, "Failed to set oversampling. Error code %d", com_rslt); return; }

	com_rslt += bme280_set_standby_durn(bme280, BME280_STANDBY_TIME_1_MS);
	ESP_LOGI(TAG, "Standby duration: %u ms", v_stand_by_time_u8);
	com_rslt += bme280_set_filter(bme280, BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(bme280, BME280_NORMAL_MODE);


    // if (com_rslt != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set oversampling. Error code %d", com_rslt);
    //     return;
    // }
    // ESP_LOGI(TAG, "Oversampling set to 4X for all");

    // // Set standby duration
    // com_rslt = bme280_set_standby_durn(bme280, BME280_STANDBY_TIME_1_MS);
    // if (com_rslt != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set standby duration. Error code %d", com_rslt);
    //     return;
    // }
    // ESP_LOGI(TAG, "Standby duration set to 1ms");

    // com_rslt = bme280_get_standby_durn(bme280, &v_stand_by_time_u8);
    // if (com_rslt != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to get standby duration. Error code %d", com_rslt);
    //     return;
    // }
    ESP_LOGI(TAG, "Standby duration: %u ms", v_stand_by_time_u8);

    // ESP_LOGI(TAG, "BME280 setup completed successfully");

    u32 v_pressure_u32 = 0;
    s32 v_temperature_s32 = 0;
    u32 v_humidity_u32 = 0;

	s32 v_pressure_uncomp_u32 = 0;
    s32 v_temperature_uncomp_s32 = 0;
    s32 v_humidity_uncomp_u32 = 0;

	double temp = 0;
	char temperature[12];
	double press = 0; // Pa -> hPa
	char pressure[10];
	double hum = 0;
	char humidity[10];

    while(1) {
        // Read sensor data (temperature, pressure, and humidity)
        ESP_ERROR_CHECK(bme280_read_uncomp_pressure_temperature_humidity(bme280, &v_pressure_uncomp_u32, &v_temperature_uncomp_s32, &v_humidity_uncomp_u32));
        ESP_LOGI(TAG, "UNCOMPENSED READ\nPressure: %u, Temperature: %d, Humidity: %u",v_pressure_uncomp_u32, v_temperature_uncomp_s32, v_humidity_uncomp_u32);

		// Read sensor data (temperature, pressure, and humidity)
        ESP_ERROR_CHECK(bme280_read_pressure_temperature_humidity(bme280, &v_pressure_u32, &v_temperature_s32, &v_humidity_u32));
        ESP_LOGI(TAG, "COMPENSED READ\nPressure: %u Pa, Temperature: %d C, Humidity: %u %%RH", v_pressure_u32/256, v_temperature_s32/100, v_humidity_u32/1024);

		temp = bme280_compensate_temperature_double(bme280, v_temperature_uncomp_s32);
		sprintf(temperature, "%.2f degC", temp);

		double press = bme280_compensate_pressure_double(bme280, v_pressure_uncomp_u32) / 100; // Pa -> hPa
		sprintf(pressure, "%.2f hPa", press);

		double hum = bme280_compensate_humidity_double(bme280, v_humidity_uncomp_u32);
		sprintf(humidity, "%.2f %%", hum);
		
        ESP_LOGI(TAG, "DOUBLE COMP\nTemperature %s Pressure %.2f hPa Humidity %.2f %%RH",temperature, press, hum);


        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // u8 ret = 0;
    // // Initialize I2C
    // ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
    //     return;
    // }
    
    // // Start BME280 sensor
    // start_bme280();




    // Initialize memory
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Initialize I2C parameters
	i2c_master_init();

    start_bme280();
}
