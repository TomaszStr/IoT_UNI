#include "bme280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// // #define I2C_MASTER_SCL_IO    22  // SCL pin
// // #define I2C_MASTER_SDA_IO    21  // SDA pin
// #define I2C_MASTER_NUM       I2C_NUM_0 // I2C bus number
// // #define I2C_MASTER_FREQ_HZ   100000 // I2C frequency (100kHz)

static const char *TAG = "BME280_ESP"; 

// s8 bus_write(u8 dev_id, u8 reg_addr, u8 *data, u8 len) {
//     s8 ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//     // Start sequence to write register address and data
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true); // Device write address
//     i2c_master_write_byte(cmd, reg_addr, true);  // Register address to write to

//     // Write data
//     i2c_master_write(cmd, data, len, true);
//     i2c_master_stop(cmd);

//     // Execute the write operation
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);

//     if (ret != ESP_OK) {
//         ESP_LOGE("I2C", "Error writing data to device (bus_write).");
//         return ESP_FAIL;
//     }

//     return ESP_OK;
// }

// s8 bus_read(u8 dev_id, u8 reg_addr, u8 *data, u8 len) {
//     s8 ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//     // Start sequence to write register address
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true); // Device write address
//     i2c_master_write_byte(cmd, reg_addr, true); // Register address to read from
//     i2c_master_stop(cmd);

//     // Execute write operation (to set register address)
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);

//     if (ret != ESP_OK) {
//         ESP_LOGE("I2C", "Error sending register address to device (bus_read).");
//         return ESP_FAIL;
//     }

//     // Start sequence to read data from device
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true); // Device read address
//     i2c_master_read(cmd, data, len, I2C_MASTER_ACK);  // Read data from the device
//     i2c_master_stop(cmd);

//     // Execute read operation
//     ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);

//     if (ret != ESP_OK) {
//         ESP_LOGE("I2C", "Error reading data from device (bus_read).");
//         return ESP_FAIL;
//     }

//     return ESP_OK;
// }


BME280_RETURN_FUNCTION_TYPE bme280_write_register(struct bme280_t *bme280, u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bme280 structure pointer as NULL*/
	if (bme280 == NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = bme280->BME280_BUS_WRITE_FUNC(
			bme280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_read_register(struct bme280_t *bme280, u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bme280 structure pointer as NULL*/
	if (bme280 == NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = bme280->BME280_BUS_READ_FUNC(
			bme280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_get_calib_param(struct bme280_t *bme280)
{
    esp_err_t err;
    uint8_t calib_data[26];  // Bufor na odczytane dane kalibracyjne
    uint8_t calib_reg_addr = 0x88;  // Adres pierwszego rejestru kalibracyjnego dla T1, P1, H1, H2

    // Odczytanie wszystkich 26 bajtów danych kalibracyjnych za jednym razem
    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, calib_reg_addr, calib_data, 26);
    if (err != 0)
        return -1;

    // temperature calibration data
    bme280->cal_param.dig_T1 = (u16)(calib_data[0] | (calib_data[1] << 8));
    bme280->cal_param.dig_T2 = (s16)(calib_data[2] | (calib_data[3] << 8));
    bme280->cal_param.dig_T3 = (s16)(calib_data[4] | (calib_data[5] << 8));
    
    // pressure calibration data
    bme280->cal_param.dig_P1 = (u16)(calib_data[6] | (calib_data[7] << 8));
    bme280->cal_param.dig_P2 = (s16)(calib_data[8] | (calib_data[9] << 8));
    bme280->cal_param.dig_P3 = (s16)(calib_data[10] | (calib_data[11] << 8));
    bme280->cal_param.dig_P4 = (s16)(calib_data[12] | (calib_data[13] << 8));
    bme280->cal_param.dig_P5 = (s16)(calib_data[14] | (calib_data[15] << 8));
    bme280->cal_param.dig_P6 = (s16)(calib_data[16] | (calib_data[17] << 8));
    bme280->cal_param.dig_P7 = (s16)(calib_data[18] | (calib_data[19] << 8));
    bme280->cal_param.dig_P8 = (s16)(calib_data[20] | (calib_data[21] << 8));
    bme280->cal_param.dig_P9 = (s16)(calib_data[22] | (calib_data[23] << 8));

    // humidity calibration data
    bme280->cal_param.dig_H1 = calib_data[25];

    // Read data from 0xE1 - 0xE6
    calib_reg_addr = 0xE1;

    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, calib_reg_addr, calib_data, 7);
    if (err != 0)
        return -1;

    bme280->cal_param.dig_H2 = (calib_data[0] | (calib_data[1] << 8));
    bme280->cal_param.dig_H3 = calib_data[2];
    // Odczytanie H4 i H5 jako połączone bajty
    // bme280->cal_param.dig_H4 = (calib_data[3] << 4) | (calib_data[4] & 0x0F);
    bme280->cal_param.dig_H4 = (s16)((((s16)((s8)calib_data[3])) << 4) | (((u8)0x0F) & calib_data[4]));
    bme280->cal_param.dig_H5 = (s16)((((s16)((s8)calib_data[5])) << 4) | (calib_data[4] >> 4));
    // bme280->cal_param.dig_H5 = (calib_data[5] << 4) | ((calib_data[29] >> 4) & 0x0F);
    bme280->cal_param.dig_H6 = calib_data[6];

    return 0;
}


BME280_RETURN_FUNCTION_TYPE bme280_init(struct bme280_t *bme280)
{
    // uint8_t chip_id = 0;               // Buffer for read chip ID
    // uint8_t reg_addr = BME280_REG_ID;  // Register address 0xD0 (Chip ID)
    // esp_err_t err;

    // if (bme280 == NULL) {
    //     ESP_LOGE(TAG, "BME280 structure pointer is NULL.");
    //     return -1;  // Return error if bme280 pointer is null
    // }

    // // Set device address (you can change this if using different I2C addresses)
    // bme280->dev_addr = BME280_I2C_ADDRESS1;
    // ESP_LOGI(TAG, "BME280 device address set to 0x%02X", bme280->dev_addr);

    // if (bme280->bus_read == NULL) {
    //     ESP_LOGE(TAG, "Bus read function pointer is not initialized.");
    //     return -1;  // Early return to avoid crash
    // }

    // // Step 1: Read chip ID
    // ESP_LOGI(TAG, "Reading chip ID from register 0x%02X", reg_addr);
    // err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, reg_addr, &chip_id, 1);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to read chip ID. Error: %d", err);
    //     return -1;  // Return error if chip ID read failed
    // }

    // // Step 2: Verify chip ID (should be 0x60 for BME280)
    // ESP_LOGI(TAG, "Chip ID read: 0x%02X", chip_id);
    // if (chip_id != 0x60) {
    //     ESP_LOGE(TAG, "Chip ID mismatch. Expected: 0x60, Found: 0x%02X", chip_id);
    //     return -1;  // Return error if chip ID does not match expected value
    // }

    // bme280->chip_id = chip_id;  // Store chip ID in the struct
    // ESP_LOGI(TAG, "Chip ID verification successful. Chip ID: 0x%02X", chip_id);

    // // Step 3: Read calibration parameters
    // ESP_LOGI(TAG, "Reading calibration parameters...");
    // err = bme280_get_calib_param(bme280);  // Read calibration data
    // if (err != 0) {
    //     ESP_LOGE(TAG, "Failed to read calibration parameters. Error: %d", err);
    //     return -1;
    // }

    // ESP_LOGI(TAG, "Calibration parameters read successfully.");
    // return 0;


    /* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = 0;
	u8 v_chip_id_read_count = 5;

	/* assign BME280 ptr */
	struct bme280_t *p_bme280 = bme280;

	while (v_chip_id_read_count > 0)
	{

		/* read Chip Id */
		com_rslt = p_bme280->BME280_BUS_READ_FUNC(p_bme280->dev_addr, BME280_REG_ID, &v_data_u8, 1);
		/* Check for the correct chip id */
		if (v_data_u8 == BME280_CHIP_ID){
            break;
        }
        ESP_LOGW(TAG, "Failed to read chip ID. Chip: %d Error: %d", v_data_u8, com_rslt);
		v_chip_id_read_count--;
		/* Delay added concerning the low speed of power up system to
		facilitate the proper reading of the chip ID */
		p_bme280->delay_msec(1);
	}
	/*assign chip ID to the global structure*/
	p_bme280->chip_id = v_data_u8;
	/*com_rslt status of chip ID read*/
	com_rslt = (v_chip_id_read_count == 0) ? BME280_CHIP_ID_READ_FAIL : BME280_CHIP_ID_READ_SUCCESS;

	if (com_rslt == BME280_CHIP_ID_READ_SUCCESS)
	{
		/* readout bme280 calibparam structure */
		com_rslt += bme280_get_calib_param(p_bme280);
	}
	return com_rslt;
}



BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_temperature(struct bme280_t *bme280, s32 *v_uncomp_temperature_s32)
{
    uint8_t reg_addr = BME280_REG_TEMP_MSB; // Starting address 0xFA
    uint8_t temp_data[3] = {0};             // Buffer for MSB, LSB, XLSB
    esp_err_t err;

    // Check if pointers are valid
    if (bme280 == NULL || v_uncomp_temperature_s32 == NULL) {
        return -1; // Error: Invalid pointers
    }

    // Read 3 bytes starting from 0xFA
    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, reg_addr, temp_data, 3);
    if (err != ESP_OK) {
        return -1; // Communication error
    }

    // Combine the 20-bit temperature value
    *v_uncomp_temperature_s32 = (s32)(((u32)temp_data[0] << 12) |
                                         ((u32)temp_data[1] << 4) |
                                         ((u32)temp_data[2] >> 4));

    return 0; // Success
}

s32 bme280_compensate_temperature_int32(struct bme280_t *bme280, s32 v_uncomp_temperature_s32)
{
    s32 var1, var2, temperature;

    // Step 1: Calculate t_fine as per datasheet
    var1 = ((((v_uncomp_temperature_s32 >> 3) - ((s32)bme280->cal_param.dig_T1 << 1))) * ((s32)bme280->cal_param.dig_T2)) >> 11;

    var2 = (((((v_uncomp_temperature_s32 >> 4) - ((s32)bme280->cal_param.dig_T1)) * ((v_uncomp_temperature_s32 >> 4) - ((s32)bme280->cal_param.dig_T1))) >> 12) * ((s32)bme280->cal_param.dig_T3)) >> 14;

    bme280->cal_param.t_fine = var1 + var2; // Global variable for further calculations

    // Step 2: Calculate actual temperature
    temperature = (bme280->cal_param.t_fine * 5 + 128) >> 8;

    return temperature; // Temperature in 0.01 degree Celsius
}

s16 bme280_compensate_temperature_int32_sixteen_bit_output(struct bme280_t *bme280, s32 v_uncomp_temperature_s32)
{
    s16 temperature = 0;

	bme280_compensate_temperature_int32(bme280, v_uncomp_temperature_s32);
	temperature  = (s16)((((bme280->cal_param.t_fine - 122880) * 25) + 128) >> 8);

	return temperature;
}



BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure(struct bme280_t *bme280, s32 *v_uncomp_pressure_s32)
{
    uint8_t reg_addr = BME280_REG_PRESS_MSB; // Adres MSB rejestru ciśnienia (0xF7)
    uint8_t data[3]; // Bufor na dane (MSB, LSB, xLSB)
    esp_err_t err;

    // Odczyt 3-bajtowych danych z rejestru
    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, reg_addr, data, 3);
    if (err != ESP_OK) {
        return -1;  // Błąd odczytu
    }

    // Połączenie odczytanych bajtów w jedną wartość 32-bitową
    *v_uncomp_pressure_s32 = (s32)(((u32)data[0] << 12) |
                                ((u32)data[1] << 4) |
                                ((u32)(data[2] >> 4)));

    return 0;  // Sukces
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
u32 bme280_compensate_pressure_int32(struct bme280_t *bme280, s32 v_uncomp_pressure_s32)
{
    s64 var1, var2;
    s64 pressure;

    var1 = ((s64)bme280->cal_param.t_fine) - 128000;
    var2 = var1 * var1 * (s64)bme280->cal_param.dig_P6;
    var2 = var2 + ((var1*(s64)bme280->cal_param.dig_P5)<<17);
    var2 = var2 + (((s64)bme280->cal_param.dig_P4)<<35);
    var1 = ((var1 * var1 * (s64)bme280->cal_param.dig_P3)>>8) + ((var1 * (s64)bme280->cal_param.dig_P2)<<12);
    var1 = (((((s64)1)<<47)+var1))*((s64)bme280->cal_param.dig_P1)>>33; 
    if (var1 == 0)  {   
        return 0; // avoid exception caused by division by zero  
    }
    pressure = 1048576-v_uncomp_pressure_s32;
    pressure = (((pressure<<31)-var2)*3125)/var1;
    var1 = (((s64)bme280->cal_param.dig_P9) * (pressure>>13) * (pressure>>13)) >> 25;
    var2 = (((s64)bme280->cal_param.dig_P8) * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + (((s64)bme280->cal_param.dig_P7)<<4);
    return (s64)pressure;
}



BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_humidity(struct bme280_t *bme280, s32 *v_uncomp_humidity_s32)
{
    u8 reg_addr = BME280_REG_HUM_MSB; // Adres MSB rejestru wilgotności (0xFD)
    u8 data[2]; // Bufor na dane (MSB, LSB)
    esp_err_t err;

    // Odczyt 2-bajtowych danych z rejestru
    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, reg_addr, data, 2);
    if (err != ESP_OK) {
        return -1;  // Błąd odczytu
    }

    // Połączenie odczytanych bajtów w jedną wartość 32-bitową
    *v_uncomp_humidity_s32 = (s32)((u32)data[0] << 8) | (u32)data[1];

    return 0;  // Sukces
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
u32 bme280_compensate_humidity_int32(struct bme280_t *bme280, s32 v_uncomp_humidity_s32)
{
    s32 var1;

    var1 = (bme280->cal_param.t_fine - ((s32)76800));
    var1 = (((((v_uncomp_humidity_s32 << 14) - (((s32)bme280->cal_param.dig_H4) << 20) - (((s32)bme280->cal_param.dig_H5) * 
        var1)) + ((s32)16384)) >> 15) * (((((((var1 * 
        ((s32)bme280->cal_param.dig_H6)) >> 10) * (((var1 * ((s32)bme280->cal_param.dig_H3)) >> 11) + 
        ((s32)32768))) >> 10) + ((s32)2097152)) * ((s32)bme280->cal_param.dig_H2) + 
        8192) >> 14));  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * 
        ((s32)bme280->cal_param.dig_H1)) >> 4));  var1 = (var1 < 0 ? 0 : var1);
        var1 = (var1 > 419430400 ? 419430400 : var1);
    return (u32)(var1>>12);

}

u16 bme280_compensate_humidity_int32_sixteen_bit_output(struct bme280_t *bme280, s32 v_uncomp_humidity_s32)
{
    u32 var1 = 0;
	u16 var2 = 0;

	var1 =  bme280_compensate_humidity_int32(bme280, v_uncomp_humidity_s32);
	var2 = (u16)(var1 >> 1);

    return var2; // Wilgotność w %rH (16-bitowy wynik)
}



BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure_temperature_humidity(
    struct bme280_t *bme280,
    s32 *v_uncomp_pressure_s32,
    s32 *v_uncomp_temperature_s32,
    s32 *v_uncomp_humidity_s32)
{
    uint8_t reg_addr = BME280_REG_PRESS_MSB; // Adres MSB rejestru ciśnienia (0xF7)
    uint8_t data[8]; // Bufor na dane (8 bajtów: 3 bajty ciśnienia, 3 bajty temperatury, 2 bajty wilgotności)
    esp_err_t err;

    // Odczyt 8-bajtowych danych z rejestru
    err = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, reg_addr, data, 8);
    if (err != ESP_OK) {
        return -1; // Błąd odczytu
    }

    // Połączenie odczytanych bajtów w jeden wynik dla każdego parametru
    *v_uncomp_pressure_s32 = ((s32)data[0] << 12) | ((s32)data[1] << 4) | (data[2] >> 4);  // Ciśnienie (3 bajty)
    *v_uncomp_temperature_s32 = ((s32)data[3] << 12) | ((s32)data[4] << 4) | (data[5] >> 4);  // Temperatura (3 bajty)
    *v_uncomp_humidity_s32 = ((s32)data[6] << 8) | data[7]; // Wilgotność (2 bajty)

    return 0; // Sukces
}

BME280_RETURN_FUNCTION_TYPE bme280_read_pressure_temperature_humidity(
    struct bme280_t *bme280,
    u32 *v_pressure_u32,
    s32 *v_temperature_s32,
    u32 *v_humidity_u32
)
{
    s32 uncomp_pressure;
    s32 uncomp_temperature;
    s32 uncomp_humidity;
    BME280_RETURN_FUNCTION_TYPE result;

    // Odczytanie surowych danych (uncompensated pressure, temperature, humidity)
    result = bme280_read_uncomp_pressure_temperature_humidity(bme280, &uncomp_pressure, &uncomp_temperature, &uncomp_humidity);
    if (result != 0) {
        return result; // Błąd w odczycie surowych danych
    }

    // Kompensacja temperatury
    *v_temperature_s32 = bme280_compensate_temperature_int32(bme280, uncomp_temperature);

    // Kompensacja ciśnienia
    *v_pressure_u32 = bme280_compensate_pressure_int32(bme280, uncomp_pressure);

    // Kompensacja wilgotności
    *v_humidity_u32 = bme280_compensate_humidity_int32(bme280, uncomp_humidity);

    return 0; // Sukces
}



BME280_RETURN_FUNCTION_TYPE bme280_get_power_mode(struct bme280_t *bme280, u8 *v_power_mode_u8) {
    /* used to return the communication result*/
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_mode_u8r = 0;
    
    /* Check if the bme280 structure pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    }

    /* Read the value of control measurement register (0xF4) */
    com_rslt = bme280->BME280_BUS_READ_FUNC(
        bme280->dev_addr, 
        BME280_CTRL_MEAS_REG,  // Address of CTRL_MEAS_REG
        &v_mode_u8r, 
        1);     // Read 1 byte

    /* Check if the read was successful */
    if (com_rslt == ERROR) {
        return com_rslt;
    }

    /* Extract the power mode bits (bit 0 and bit 1) */
    *v_power_mode_u8 = ((v_mode_u8r & (0x03)) >> (0));  // Bit mask for 2 least significant bits (power mode bits)

    return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_power_mode(struct bme280_t *bme280, u8 v_power_mode_u8) {
    /* used to return the communication result*/
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_mode_u8r = 0;
	u8 v_prev_pow_mode_u8 = 0;
	u8 v_pre_ctrl_hum_value_u8 = 0;
	u8 v_pre_config_value_u8 = 0;
	u8 v_data_u8 = 0;

    /* Check if the bme280 structure pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    }

    /* Ensure valid power mode */
    if (v_power_mode_u8 <= BME280_NORMAL_MODE) {
        v_mode_u8r = bme280->ctrl_meas_reg;
        v_mode_u8r =((v_mode_u8r & ~(0x03)) | ((v_power_mode_u8<<(0))&(0x03)));
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);
        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);
            /* write previous value of
            configuration register*/
            v_pre_config_value_u8 =
            bme280->config_reg;
            com_rslt = bme280_write_register(bme280, BME280_CONFIG_REG, &v_pre_config_value_u8, 1);
            /* write previous value of
            humidity oversampling*/
            v_pre_ctrl_hum_value_u8 =
            bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);
            /* write previous and updated value of
            control measurement register*/
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_mode_u8r, 1);
        } else {
            com_rslt = bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_mode_u8r, 1);
        }
        /* read the control measurement register value*/
        com_rslt = bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;
        /* read the control humidity register value*/
        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;
        /* read the config register value*/
        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    } else {
    com_rslt = E_BME280_OUT_OF_RANGE;
    }

    return com_rslt;
}



BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_temperature(struct bme280_t *bme280, u8 *v_value_u8) {
    /* used to return the communication result*/
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;

    /* Check if the bme280 structure pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    }

    /* Read the value of control measurement register (0xF4) */
    com_rslt = bme280->BME280_BUS_READ_FUNC(
        bme280->dev_addr, 
        BME280_CTRL_MEAS_REG,  // Address of CTRL_MEAS_REG
        &v_data_u8, 
        1);     // Read 1 byte

    if (com_rslt != ERROR) {
        /* Extract the oversampling temperature bits (bits 5 to 7) */
        *v_value_u8 = ((v_data_u8 & (0xE0)) >> (5));  // Mask the bits 5-7

        bme280->oversamp_temperature = *v_value_u8;
    }

    return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_temperature(struct bme280_t *bme280, u8 v_value_u8)
{
    /* used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;  /* Variable to store the current control measurement register value */
    u8 v_prev_pow_mode_u8 = 0;  /* Variable to store previous power mode */
    u8 v_pre_ctrl_hum_value_u8 = 0;  /* Variable to store previous control humidity register value */
    u8 v_pre_config_value_u8 = 0;  /* Variable to store previous config register value */

    /* Check if the structure pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the current value of the control measurement register */
        v_data_u8 = bme280->ctrl_meas_reg;

        v_data_u8 = ((v_data_u8 & ~(0xE0)) | ((v_value_u8<<(5))&(0xE0)));  // 0x1F = 00011111 in binary, clears the upper 3 bits

        /* Get the current power mode */
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);
        if (com_rslt != SUCCESS) {
            return com_rslt;
        }

        /* If not in sleep mode, perform a soft reset and update the registers */
        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);

            /* Write the previous value of the configuration register */
            v_pre_config_value_u8 = bme280->config_reg;
            com_rslt += bme280_write_register(bme280, BME280_CONFIG_REG, &v_pre_config_value_u8, 1);

            /* Write the previous value of the humidity oversampling */
            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);

            /* Write the updated control measurement register */
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        } else {
            /* If in sleep mode, write directly to the register */
            com_rslt = bme280->BME280_BUS_WRITE_FUNC(bme280->dev_addr, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        }

        /* Update the oversampling temperature in the structure */
        bme280->oversamp_temperature = v_value_u8;

        /* Read the control measurement register value */
        com_rslt = bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;

        /* Read the control humidity register value */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        /* Read the configuration register value */
        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    }

    return com_rslt;
}


BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_pressure(struct bme280_t *bme280, u8 *v_value_u8)
{
	/* used to return the communication result */
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = 0;

	/* Check if the bme280 pointer is NULL */
	if (bme280 == NULL) {
		return E_BME280_NULL_PTR;
	} else {
		/* Read the register that contains the oversampling setting for pressure */
		com_rslt = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, BME280_CTRL_MEAS_REG, &v_data_u8, 1);

		/* Extract the oversampling setting from the read byte */
		*v_value_u8 = ((v_data_u8 & (0x1C)) >> (2)); // Extract bits 2-4

		/* Save the oversampling value in the internal state */
		bme280->oversamp_pressure = *v_value_u8;
	}

	return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_pressure(struct bme280_t *bme280, u8 v_value_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;
    u8 v_prev_pow_mode_u8 = 0;
    u8 v_pre_ctrl_hum_value_u8 = 0;
    u8 v_pre_config_value_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Get the current measurement control register value */
        v_data_u8 = bme280->ctrl_meas_reg;

        /* Modify the oversampling bits for pressure (bits 2-4) */
        // Clear bits 2-4 (masking)
        // Set the new oversampling value (shifted into bits 2-4)
        v_data_u8 = ((v_data_u8 & ~(0x1C)) | ((v_value_u8<<(2))&(0x1C)));

        /* Get the current power mode */
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);

        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            /* If the sensor is not in sleep mode, perform a soft reset */
            com_rslt += bme280_set_soft_rst(bme280);
            /* Simple delay (using a basic busy-wait loop or appropriate delay function) */
            bme280->delay_msec(3);  // Use a 3ms delay

            /* Save the previous configuration register value */
            v_pre_config_value_u8 = bme280->config_reg;
            /* Write the previous configuration register value */
            com_rslt = bme280_write_register(bme280, BME280_CONFIG_REG, &v_pre_config_value_u8, 1);  // Assuming BME280_CONFIG_REG is the config register

            /* Save the previous humidity oversampling value */
            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            /* Write the previous humidity oversampling value */
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);  // Assuming BME280_CTRL_HUMIDITY_REG is the humidity control register

            /* Write the updated value of the measurement control register */
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        } else {
            /* If the sensor is in sleep mode, just write the new oversampling value */
            // Same register for control of measurements
            com_rslt = bme280->BME280_BUS_WRITE_FUNC(bme280->dev_addr, BME280_CTRL_MEAS_REG,  &v_data_u8, 1);
        }

        /* Update the oversampling value in the internal state */
        bme280->oversamp_pressure = v_value_u8;

        /* Read the control measurement register value */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;

        /* Read the control humidity register value */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        /* Read the control configuration register value */
        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    }

    return com_rslt;
}



BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_humidity(struct bme280_t *bme280, u8 *v_value_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the humidity control register (BME280_CTRL_HUMIDITY_REG) */
        com_rslt = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);  // Read 1 byte

        /* Check for communication errors */
        if (com_rslt != 0) {
            return com_rslt;
        }

        /* Extract the oversampling bits (bits 0-2) */
        *v_value_u8 = ((v_data_u8 & (0x07)) >> (0));  // Mask the first 3 bits

        /* Update the internal oversampling value */
        bme280->oversamp_humidity = *v_value_u8;
    }

    return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_humidity(struct bme280_t *bme280, u8 v_value_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;
    u8 pre_ctrl_meas_value = 0;
    u8 v_pre_config_value_u8 = 0;
    u8 v_prev_pow_mode_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Get the current humidity control register value */
        com_rslt = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);

        /* Modify the oversampling bits for humidity (bits 0-2) */
        // Clear bits 0-2
        // Set new oversampling value (0-7)
        v_data_u8 = ((v_data_u8 & ~(0x07)) | ((v_value_u8<<(0))&(0x07)));

        /* Check the current power mode */
        com_rslt += bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);

        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            /* If not in sleep mode, reset the sensor and configure */
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);  // 3ms delay for soft reset

            /* write previous value of
				configuration register*/
				v_pre_config_value_u8 = bme280->config_reg;
				com_rslt += bme280_write_register(bme280, BME280_CONFIG_REG, &v_pre_config_value_u8, 1);
				/* write the value of control humidity*/
				com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value = bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &pre_ctrl_meas_value, 1);
        }

        /* Update the internal oversampling humidity value */
        bme280->oversamp_humidity = v_value_u8;

        /* Read the control measurement, humidity, and configuration registers */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    }

    return com_rslt;
}




BME280_RETURN_FUNCTION_TYPE bme280_set_soft_rst(struct bme280_t *bme280)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = BME280_SOFT_RESET_CODE;  // Soft reset code (0xB6)

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Write the reset code to the reset register (0xE0) */
        com_rslt = bme280->BME280_BUS_WRITE_FUNC(bme280->dev_addr, 0xE0, &v_data_u8, 1);
    }

    return com_rslt;
}




BME280_RETURN_FUNCTION_TYPE bme280_get_filter(struct bme280_t *bme280, u8 *v_value_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the value from the filter register (BME280_CONFIG_REG) */
        com_rslt = bme280->BME280_BUS_READ_FUNC(bme280->dev_addr, BME280_CONFIG_REG, &v_data_u8, 1);

        /* Extract the filter setting (bits 3 and 4) */
        *v_value_u8 = ((v_data_u8 & (0x1C)) >> (2));  // Shift to get bits 3 and 4
    }

    return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_filter(struct bme280_t *bme280, u8 v_value_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;
    u8 pre_ctrl_meas_value = 0;
    u8 v_prev_pow_mode_u8 = 0;
    u8 v_pre_ctrl_hum_value_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Get the current configuration register value */
        v_data_u8 = bme280->config_reg;

        /* Set the filter setting */
        v_data_u8 = ((v_data_u8 & ~(0x1C)) | ((v_value_u8<<(2))&(0x1C)));  // Setting the filter bits (3 and 4)

        /* Check the current power mode */
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);
        if (v_prev_pow_mode_u8 != 0x01) {  // If not in sleep mode
            /* Perform soft reset if not in sleep mode */
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);  // A small delay for reset

            /* Write the updated configuration register value */
            com_rslt += bme280_write_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);

            /* Write the previous values of other registers */
            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG,&v_pre_ctrl_hum_value_u8, 1);

            pre_ctrl_meas_value = bme280->ctrl_meas_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &pre_ctrl_meas_value, 1);
        } else {
            /* If in sleep mode, directly write the updated filter setting */
            com_rslt = bme280->BME280_BUS_WRITE_FUNC(bme280->dev_addr, BME280_CONFIG_REG, &v_data_u8, 1);
        }

        /* Read and update register values */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    }

    return com_rslt;
}



BME280_RETURN_FUNCTION_TYPE bme280_get_standby_durn(struct bme280_t *bme280, u8 *v_standby_durn_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the current value of the configuration register */
        com_rslt = bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        
        /* Extract the bits 5 to 7 for standby duration */
        *v_standby_durn_u8 = ((v_data_u8 & (0xE0)) >> (5));
    }

    return com_rslt;
}

BME280_RETURN_FUNCTION_TYPE bme280_set_standby_durn(struct bme280_t *bme280, u8 v_standby_durn_u8)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;
    u8 pre_ctrl_meas_value = 0;
    u8 v_prev_pow_mode_u8 = 0;
    u8 v_pre_ctrl_hum_value_u8 = 0;

    /* Check if the bme280 pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the current value of the configuration register */
        v_data_u8 = bme280->config_reg;
        
        /* Set the standby duration bits (bits 5-7) */
        v_data_u8 = (v_data_u8 & ~0xE0) | ((v_standby_durn_u8 << 5) & 0xE0);

        /* Read the current power mode to handle sleep mode */
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);
        
        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            /* If the sensor is not in sleep mode, perform a soft reset and update registers */
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);  // 3 ms delay for reset

            /* Write updated configuration register value */
            com_rslt += bme280_write_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
            
            /* Write the previous values of humidity and measurement control registers */
            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);

            pre_ctrl_meas_value = bme280->ctrl_meas_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &pre_ctrl_meas_value, 1);
        } else {
            /* If the sensor is in sleep mode, directly update the configuration register */
            com_rslt += bme280->BME280_BUS_WRITE_FUNC(bme280->dev_addr, BME280_CONFIG_REG, &v_data_u8, 1);
        }

        /* Read and update the registers after modification */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;
    }

    return com_rslt;
}





BME280_RETURN_FUNCTION_TYPE bme280_get_forced_uncomp_pressure_temperature_humidity(
    struct bme280_t *bme280,
    s32 *v_uncom_pressure_s32,
    s32 *v_uncom_temperature_s32, 
    s32 *v_uncom_humidity_s32)
{
    /* Used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
    u8 v_data_u8 = 0;
    u8 v_waittime_u8 = 0;
    u8 v_prev_pow_mode_u8 = 0;
    u8 v_mode_u8r = 0;
    u8 pre_ctrl_config_value = 0;
    u8 v_pre_ctrl_hum_value_u8 = 0;

    /* Check if the bme280 structure pointer is NULL */
    if (bme280 == NULL) {
        return E_BME280_NULL_PTR;
    } else {
        /* Read the current value of ctrl_meas register */
        v_mode_u8r = bme280->ctrl_meas_reg;

        /* Set the forced mode by clearing bits [0:1] and setting them to 01 */
        /* Clear bits 0 and 1 (mask 11111100) */
        /* Set to forced mode (01) */
        v_mode_u8r = ((v_mode_u8r & ~(0x03)) | (((0x01)<<(0))&(0x03)));

        /* Get the current power mode */
        com_rslt = bme280_get_power_mode(bme280, &v_prev_pow_mode_u8);
        if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
            /* Perform a soft reset */
            com_rslt += bme280_set_soft_rst(bme280);
            bme280->delay_msec(3);

            /* Restore previous values of configuration registers */
            pre_ctrl_config_value = bme280->config_reg;
            com_rslt += bme280_write_register(bme280, BME280_CONFIG_REG, &pre_ctrl_config_value, 1);

            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);

            /* Write forced mode to ctrl_meas register */
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_mode_u8r, 1);
        } else {
            /* Write humidity oversampling first */
            v_pre_ctrl_hum_value_u8 = bme280->ctrl_hum_reg;
            com_rslt += bme280_write_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_pre_ctrl_hum_value_u8, 1);

            /* Write forced mode directly */
            com_rslt += bme280_write_register(bme280, BME280_CTRL_MEAS_REG, &v_mode_u8r, 1);
        }

        /* Compute the wait time and delay */
        bme280_compute_wait_time(bme280, &v_waittime_u8);
        bme280->delay_msec(v_waittime_u8);

        /* Read uncompensated pressure, temperature, and humidity */
        com_rslt += bme280_read_uncomp_pressure_temperature_humidity(bme280,  v_uncom_pressure_s32,  v_uncom_temperature_s32, v_uncom_humidity_s32);

        /* Read and store updated register values */
        com_rslt += bme280_read_register(bme280, BME280_CTRL_HUMIDITY_REG, &v_data_u8, 1);
        bme280->ctrl_hum_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CONFIG_REG, &v_data_u8, 1);
        bme280->config_reg = v_data_u8;

        com_rslt += bme280_read_register(bme280, BME280_CTRL_MEAS_REG, &v_data_u8, 1);
        bme280->ctrl_meas_reg = v_data_u8;
    }
    return com_rslt;
}






#ifdef BME280_ENABLE_FLOAT

double bme280_compensate_temperature_double(struct bme280_t *bme280, s32 v_uncom_temperature_s32)
{
    /* Check if the pointer to bme280 is NULL */
    if (bme280 == NULL) {
        return 0.0; // Return 0 for error handling
    }

    double v_x1_u32 = 0;
	double v_x2_u32 = 0;
	double temperature = 0;

	v_x1_u32  = (((double)v_uncom_temperature_s32) / 16384.0 -
	((double)bme280->cal_param.dig_T1) / 1024.0) *
	((double)bme280->cal_param.dig_T2);
	v_x2_u32  = ((((double)v_uncom_temperature_s32) / 131072.0 -
	((double)bme280->cal_param.dig_T1) / 8192.0) *
	(((double)v_uncom_temperature_s32) / 131072.0 -
	((double)bme280->cal_param.dig_T1) / 8192.0)) *
	((double)bme280->cal_param.dig_T3);
	bme280->cal_param.t_fine = (s32)(v_x1_u32 + v_x2_u32);
	temperature  = (v_x1_u32 + v_x2_u32) / 5120.0;


	return temperature;
}

double bme280_compensate_pressure_double(struct bme280_t *bme280, s32 v_uncom_pressure_s32)
{
    /* Check if the pointer to bme280 is NULL */
    if (bme280 == NULL) {
        return 0.0; // Return 0 for error handling
    }

    double v_x1_u32 = 0;
	double v_x2_u32 = 0;
	double pressure = 0;

	v_x1_u32 = ((double)bme280->cal_param.t_fine /
	2.0) - 64000.0;
	v_x2_u32 = v_x1_u32 * v_x1_u32 *
	((double)bme280->cal_param.dig_P6) / 32768.0;
	v_x2_u32 = v_x2_u32 + v_x1_u32 *
	((double)bme280->cal_param.dig_P5) * 2.0;
	v_x2_u32 = (v_x2_u32 / 4.0) +
	(((double)bme280->cal_param.dig_P4) * 65536.0);
	v_x1_u32 = (((double)bme280->cal_param.dig_P3) *
	v_x1_u32 * v_x1_u32 / 524288.0 +
	((double)bme280->cal_param.dig_P2) * v_x1_u32) / 524288.0;
	v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) *
	((double)bme280->cal_param.dig_P1);
	pressure = 1048576.0 - (double)v_uncom_pressure_s32;
	/* Avoid exception caused by division by zero */
	if ((v_x1_u32 > 0) || (v_x1_u32 < 0))
		pressure = (pressure - (v_x2_u32 / 4096.0)) * 6250.0 / v_x1_u32;
	else
		return 0;
	v_x1_u32 = ((double)bme280->cal_param.dig_P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32 = pressure * ((double)bme280->cal_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32 + v_x2_u32 +
	((double)bme280->cal_param.dig_P7)) / 16.0;

	return pressure;
}

double bme280_compensate_humidity_double(struct bme280_t *bme280, s32 v_uncom_humidity_s32)
{
    /* Check if the pointer to bme280 is NULL */
    if (bme280 == NULL) {
        return 0.0; // Return 0 for error handling
    }
    double var_h = 0;

    var_h = (((double)bme280->cal_param.t_fine) - 76800.0);
	if ((var_h > 0) || (var_h < 0))
		var_h = (v_uncom_humidity_s32 -
		(((double)bme280->cal_param.dig_H4) * 64.0 +
		((double)bme280->cal_param.dig_H5) / 16384.0 * var_h))*
		(((double)bme280->cal_param.dig_H2) / 65536.0 *
		(1.0 + ((double) bme280->cal_param.dig_H6)
		/ 67108864.0 * var_h * (1.0 + ((double)
		bme280->cal_param.dig_H3) / 67108864.0 * var_h)));
	else
		return 0;
	var_h = var_h * (1.0 - ((double)
	bme280->cal_param.dig_H1)*var_h / 524288.0);
	if (var_h > 100.0)
		var_h = 100.0;
	else if (var_h < 0.0)
		var_h = 0.0;
	return var_h;// Return the humidity in %rH
}
#endif

BME280_RETURN_FUNCTION_TYPE bme280_compute_wait_time(struct bme280_t *bme280, u8 *v_delaytime_u8)
{
    /* used to return the communication result */
    BME280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

    /* Ensure that the pointer to bme280 structure is valid */
    if (bme280 == NULL || v_delaytime_u8 == NULL) {
        return E_BME280_NULL_PTR;
    }
    
    /* Calculate the waiting time using oversampling and setup values */
    *v_delaytime_u8 = (20 +
                       37 *
                       (((1 << bme280->oversamp_temperature) >> 1)
                       + ((1 << bme280->oversamp_pressure) >> 1) +
                       ((1 << bme280->oversamp_humidity) >> 1))
                       + ((bme280->oversamp_pressure > 0) ? 10 : 0) +
                       ((bme280->oversamp_humidity > 0) ? 10 : 0) + 15) / 16;

    return com_rslt;  // Return success
}
