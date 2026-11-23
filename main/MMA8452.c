#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// i2c
#define SDA_GPIO 4
#define SCL_GPIO 5
#define MMA8452_ADDR 0x1C // SA0 -> GND (SA0 -> GND); SA0 -> VCC gives 0x1D 
#define MMA8452_SCL_SPEED_HZ 100000 // 100kHz -> slow mode, 400kHz -> fast mode 

// registers 
#define MMA8452_WHO_AM_I_REG 0x0D
#define MMA8452_WHO_AM_I_VAL 0x2A // expected WHO_AM_I value for MMA8452 
#define MMA8452_XYZ_DATA_CFG_REG 0x0E // full-scale range register 
#define MMA8452_FS_2G 0x00 // 0 -> +-2g 
#define MMA8452_CTRL_REG1 0x2A // data rate + active/standby 
#define MMA8452_CTRL_REG1_STANDBY 0x00 // ACTIVE = 0 -> standby, settings can be changed only here 
#define MMA8452_CTRL_REG1_ACTIVE 0x01 // bit0 ACTIVE = 1 -> start measuring 
#define MMA8452_DR_100HZ (0x03 << 3) // bits[5:3] DR = 011 -> 100Hz output data rate 
#define MMA8452_START_REG 0x01 // OUT_X_MSB, then X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB 

// data layout 
#define MMA8452_DATA_LEN 6 // 3 axes * 2 bytes 
#define MMA8452_DATA_SHIFT 4 // 12-bit value is left-justified in 16 bits 

// conversion 
#define MMA8452_LSB_SENSITIVITY 1024.0f // counts per g at +-2g, 12-bit 
#define MMA8452_DP_NUMBER 2

// delays 
#define MMA8452_DELAY_ACTIVE_MS 100 // wait after switching to active mode 
#define MMA8452_PERIOD_MS 250


// i2c transaction timeout
#define MMA8452_XFER_TIMEOUT_MS 100

static const char *TAG = "mma8452";

static esp_err_t mma8452_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;

	// (1) creating the bus 
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = SDA_GPIO,
		.scl_io_num = SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7, // default value 
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus;
	err = i2c_new_master_bus(&bus_cfg, &bus);
	if (err != ESP_OK) {
		return err;
	}


	// (2) adding device 
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = MMA8452_ADDR,
		.scl_speed_hz = MMA8452_SCL_SPEED_HZ,
	};
	err = i2c_master_bus_add_device(bus, &dev_cfg, dev);
	if (err != ESP_OK) {
		return err;
	}


	// (3) checking WHO_AM_I register 
	uint8_t who_am_i_reg = MMA8452_WHO_AM_I_REG;
	uint8_t who = 0;

	err = i2c_master_transmit_receive(*dev, &who_am_i_reg, 1, &who, 1, MMA8452_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}

	if (who == MMA8452_WHO_AM_I_VAL) {
		ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
	} else {
		ESP_LOGW(TAG, "unexpected WHO_AM_I: 0x%02X", who);
	}

	// (4) go to standby, settings are ignored while sensor is active 
	uint8_t standby[2] = {MMA8452_CTRL_REG1, MMA8452_CTRL_REG1_STANDBY};

	err = i2c_master_transmit(*dev, standby, 2, MMA8452_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}


	// (5) set accel range to +-2g explicitly 
	uint8_t range_cfg[2] = {MMA8452_XYZ_DATA_CFG_REG, MMA8452_FS_2G};
	err = i2c_master_transmit(*dev, range_cfg, 2, MMA8452_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}

	// (6) set data rate 100Hz and switch to active mode 
	uint8_t active[2] = {MMA8452_CTRL_REG1, MMA8452_DR_100HZ | MMA8452_CTRL_REG1_ACTIVE};
	err = i2c_master_transmit(*dev, active, 2, MMA8452_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	vTaskDelay(pdMS_TO_TICKS(MMA8452_DELAY_ACTIVE_MS)); // wait for first measurement 

	return ESP_OK;

}


static esp_err_t mma8452_read (i2c_master_dev_handle_t dev, float *gX, float *gY, float *gZ) {
	uint8_t start_reg = MMA8452_START_REG;
	uint8_t data[MMA8452_DATA_LEN];
	esp_err_t err;

	// (1) read values, register address auto-increments 0x01 -> 0x06 
	err = i2c_master_transmit_receive(dev, &start_reg, 1, data, MMA8452_DATA_LEN, MMA8452_XFER_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}

	// (2) combine bits: MSB << 8 | LSB, then shift right by 4 (12-bit, sign is kept)
	int16_t rawX = (int16_t) ((data[0] << 8) | data[1]) >> MMA8452_DATA_SHIFT;
	int16_t rawY = (int16_t) ((data[2] << 8) | data[3]) >> MMA8452_DATA_SHIFT;
	int16_t rawZ = (int16_t) ((data[4] << 8) | data[5]) >> MMA8452_DATA_SHIFT;

	// (3) convert to real values 
	*gX = rawX / MMA8452_LSB_SENSITIVITY;
	*gY = rawY / MMA8452_LSB_SENSITIVITY;
	*gZ = rawZ / MMA8452_LSB_SENSITIVITY;

	return ESP_OK;
}




void app_main(void) {
	i2c_master_dev_handle_t mma1;
	ESP_ERROR_CHECK(mma8452_init(&mma1));

	// get accel in loop 
	while (1) {
		float gX = 0, gY = 0, gZ = 0;

		esp_err_t err = mma8452_read(mma1, &gX, &gY, &gZ);
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "accel read failed: %s", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "gX: %+7.*f | gY: %+7.*f | gZ: %+7.*f",
				MMA8452_DP_NUMBER, gX,
				MMA8452_DP_NUMBER, gY,
				MMA8452_DP_NUMBER, gZ);
		}

		vTaskDelay(pdMS_TO_TICKS(MMA8452_PERIOD_MS));
	}


}