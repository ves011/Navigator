/*
 * vl530lx.c
 *
 *  Created on: Jan 13, 2025
 *      Author: viorel_serbu
 */

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/i2c_master.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "esp_netif.h"
#include "sys/stat.h"
#include "math.h"
#include "../main/project_specific.h"
#include "common_defines.h"
#include "external_defs.h"
#include "gpios.h"	
#include "vl53l0x.h"

static char *TAG = "VL";

static i2c_master_dev_handle_t vl_handle;
static QueueHandle_t vl_cmd_queue = NULL;

void register_vl()
	{
	/*
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MPU_DRDY_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_DRDY_PIN, mpu_drdy_isr, (void*) MPU_DRDY_PIN);
    */
    vl_cmd_queue = xQueueCreate(10, sizeof(msg_t));
    i2c_device_config_t dev_cfg = {
    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    	.scl_speed_hz = 100000,
		};


	if(i2c_master_probe(i2c_bus_handle_0, VL_I2C_ADDRESS, -1) == ESP_OK) //pdMS_TO_TICKS(10)) == ESP_OK)
		{
		dev_cfg.device_address = VL_I2C_ADDRESS;
		ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle_0, &dev_cfg, &vl_handle));
		ESP_LOGI(TAG, "VL@%x found", VL_I2C_ADDRESS);
		//vl_init(vl_handle);
		}
	else 
		{
		ESP_LOGI(TAG, "VL@%x not found", VL_I2C_ADDRESS);
		vl_handle = NULL;
		}
	}
