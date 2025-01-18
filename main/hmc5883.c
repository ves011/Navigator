/*
 * hmc5833.c
 *
 *  Created on: Nov 30, 2024
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
#include "project_specific.h"
#include "common_defines.h"
#include "external_defs.h"
#include "gpios.h"	
#include "hmc5883.h"

#define PARAM_FILE	"params.txt"

static const char *TAG = "hmc";
static char *command = "hmc";

static int hmc_dr, hmc_gain, hmc_omode, hmc_mmode, hmc_average;
static i2c_master_dev_handle_t hmc_handle;
static QueueHandle_t hmc_cmd_queue = NULL;
static void r_params();



struct
	{
    struct arg_str *op;
    struct arg_int *arg;
    struct arg_int *arg1;
    struct arg_end *end;
	} hmc_args;

static void IRAM_ATTR hmc_drdy_isr(void* arg)
	{
	msg_t msg;
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == HMC_DRDY_PIN)
    	{
		msg.source = 1;
		xQueueSendFromISR(hmc_cmd_queue, &msg, NULL);
		}
	}
static int hmc_init(i2c_master_dev_handle_t handle)
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret;
	// read ID regs
	wr_buf[0] = 10;
	ret = i2c_master_transmit(handle, wr_buf, 1, 50);
	if(ret == ESP_OK)
		{
		ret = i2c_master_receive(handle, rd_buf, 3, 50);
		if(ret == ESP_OK)
			ESP_LOGI(TAG, "addr 10: %02x, addr 11: %02x, addr 12: %02x", rd_buf[0], rd_buf[1], rd_buf[2]);
		else
 			ESP_LOGI(TAG, "error on reading: %d", ret);
		}
	else
 		ESP_LOGI(TAG, "error on writing: %d", ret);
	return ESP_OK;
	}
int do_hmc(int argc, char **argv)
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret, wr_size, rd_size, nrs, count;
	int16_t x, y, z;
	msg_t msg;
	if(strcmp(argv[0], command))
		return 0;
	int nerrors = arg_parse(argc, argv, (void **)&hmc_args);
	if (nerrors != 0)
		{
		arg_print_errors(stderr, hmc_args.end, argv[0]);
		return ESP_FAIL;
		}
	if(strcmp(hmc_args.op->sval[0], "read") == 0)
		{
		if(hmc_args.arg->count == 0)
			ESP_LOGI(TAG, "register to read not provided");
		else
 			{
			wr_buf[0] = hmc_args.arg->ival[0]; // register
			wr_size = 1;
			ret = i2c_master_transmit(hmc_handle, wr_buf, wr_size, 50);
			if(ret != ESP_OK)
				ESP_LOGI(TAG, "error on positioning to register: %d", ret);
			else
				{
				if(hmc_args.arg1->count) // read size
					{
					rd_size = hmc_args.arg1->ival[0]; // read size
					rd_buf[0] = hmc_args.arg1->ival[0]; // read size
					}
				else
					{
					rd_size = 1;
 					rd_buf[0] = 1;
 					}
				ret = i2c_master_receive(hmc_handle, rd_buf, rd_size, 50);
				if(ret != ESP_OK)
					ESP_LOGI(TAG, "error on read: %d", ret);
				else
 					{
//					for(ret = 0; ret < rd_size; ret++)
//						ESP_LOGI(TAG, "reg: %d = %02x", wr_buf[0] + ret, rd_buf[ret]);
					x = (rd_buf[0] << 8 & 0xff00) | rd_buf[1];
					y = (rd_buf[2] << 8 & 0xff00) | rd_buf[3];
					z = (rd_buf[4] << 8 & 0xff00) | rd_buf[5];
					ESP_LOGI(TAG, "x: %d, y : %d, z: %d", x, y, z);
					double atyx = atan((double)y / (double)x) * 180. /3.14;
					double atyz = atan((double)y / (double)z) * 180. /3.14;
					double atzx = atan((double)z / (double)x) * 180. /3.14;
					double tf = sqrt((double)x * (double)x + (double)y * (double)y + (double)z * (double)z);
					ESP_LOGI(TAG, "atyx: %.2lf, atyz : %.2lf, atzx: %.2lf / total field: %.2lf", atyx, atyz, atzx, tf);
					}
				}
			}
		}
	else if(strcmp(hmc_args.op->sval[0], "write") == 0)
		{
		if(hmc_args.arg->count == 0)
			ESP_LOGI(TAG, "register to write not provided");
		else
 			{
			wr_buf[0] = hmc_args.arg->ival[0]; // register
			wr_size = 1;
			if(hmc_args.arg1->count)
				{
				wr_buf[1] = hmc_args.arg1->ival[0]; // register
				wr_size = 2;
				}
			ret = i2c_master_transmit(hmc_handle, wr_buf, wr_size, 50);
			if(ret != ESP_OK)
				ESP_LOGI(TAG, "error on write: %d", ret);
			}
		}
	else if(strcmp(hmc_args.op->sval[0], "cont") == 0 || strcmp(hmc_args.op->sval[0], "single") == 0)
		{
		if(hmc_args.arg->count == 0)
			ESP_LOGI(TAG, "No of samples not provided");
		else
 			{
			nrs = hmc_args.arg->ival[0];
			wr_buf[0] = 0; 
			wr_buf[1] = ((hmc_average << 5) & 0x60) | ((hmc_dr << 2) & 0x1c) | hmc_mmode;
			ESP_LOGI(TAG, "A: %u", wr_buf[1]);
			ret = i2c_master_transmit(hmc_handle, wr_buf, 2, 50);
			wr_buf[0] = 1; 
			wr_buf[1] = ((hmc_gain << 5) & 0xe0);
			ESP_LOGI(TAG, "B: %u", wr_buf[1]);
			ret = i2c_master_transmit(hmc_handle, wr_buf, 2, 50);
			wr_buf[0] = 2;
			if(strcmp(hmc_args.op->sval[0], "cont") == 0)
				wr_buf[1] = 0x00;
			else
				wr_buf[1] = 0x01;
			ESP_LOGI(TAG, "M: %u", wr_buf[1]);
			ret = i2c_master_transmit(hmc_handle, wr_buf, 2, 50);
			xQueueReset(hmc_cmd_queue);
			count = 0;
			while(count < nrs)
				{
				if(xQueueReceive(hmc_cmd_queue, &msg, pdMS_TO_TICKS(1500)))
					{
					ret = i2c_master_receive(hmc_handle, rd_buf, 6, 50);
					x = (rd_buf[0] << 8 & 0xff00) | rd_buf[1];
					y = (rd_buf[2] << 8 & 0xff00) | rd_buf[3];
					z = (rd_buf[4] << 8 & 0xff00) | rd_buf[5];
					double atyx = atan((double)y / (double)x) * 180. /3.14;
					double atyz = atan((double)y / (double)z) * 180. /3.14;
					double atzx = atan((double)z / (double)x) * 180. /3.14;
					double tf = sqrt((double)x * (double)x + (double)y * (double)y + (double)z * (double)z);
					//ESP_LOGI(TAG, "x: %d y : %d z: %d atyx: %.2lf atyz : %.2lf atzx: %.2lf / total field: %.2lf", 
					//	x, y, z, atyx, atyz, atzx, tf);
					printf("Raw:0,0,0,0,0,0,%6d %6d %6d\n", x, y, z);
					count++;
					if(strcmp(hmc_args.op->sval[0], "cont") == 0)
						{
						wr_buf[0] = 3;
						ret = i2c_master_transmit(hmc_handle, wr_buf, 1, 50);
						}
					else
						{
						wr_buf[0] = 2;
						wr_buf[1] = 1;
						ret = i2c_master_transmit(hmc_handle, wr_buf, 2, 50);
						}
					}
				else
					{
					ESP_LOGI(TAG, "timeout while receiving");
					count++;
					}
				}
			}
		}
	else if(strcmp(hmc_args.op->sval[0], "params") == 0)
		{
		ESP_LOGI(TAG, "hmc_dr: %d, hmc_gain: %d, hmc_omode: %d, hmc_mmode: %d, hmc_average: %d", 
			hmc_dr, hmc_gain, hmc_omode, hmc_mmode, hmc_average);
		}
	return ESP_OK;
	}
void register_hmc()
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << HMC_DRDY_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //gpio_install_isr_service(0);
    gpio_isr_handler_add(HMC_DRDY_PIN, hmc_drdy_isr, (void*) HMC_DRDY_PIN);
  
    hmc_cmd_queue = xQueueCreate(10, sizeof(msg_t));

	//scl = I2C_MASTER_SCL_IO;
	//sda = I2C_MASTER_SDA_IO;
	hmc_dr = 4;
	i2c_device_config_t dev_cfg = {
    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    	.scl_speed_hz = 100000,
		};
	r_params();

	if(i2c_master_probe(i2c_bus_handle_0, HMC_I2C_ADDRESS, -1) == ESP_OK) //pdMS_TO_TICKS(10)) == ESP_OK)
		{
		dev_cfg.device_address = HMC_I2C_ADDRESS;
		ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle_0, &dev_cfg, &hmc_handle));
		ESP_LOGI(TAG, "HMC@%x found", HMC_I2C_ADDRESS);
		hmc_init(hmc_handle);
		}
	else 
		{
		ESP_LOGI(TAG, "HMC@%x not found", HMC_I2C_ADDRESS);
		hmc_handle = NULL;
		}
	hmc_args.op = arg_str1(NULL, NULL, "<op>", "op: read | write");
	hmc_args.arg = arg_int0(NULL, NULL, "<#>", "#reg");
	hmc_args.arg1= arg_int0(NULL, NULL, "<#>", "#value2write | read size");
	hmc_args.end = arg_end(1);
	const esp_console_cmd_t hmc_cmd =
		{
		.command = command,
		.help = "hmc commands",
		.hint = NULL,
		.func = &do_hmc,
		.argtable = &hmc_args
		};
	ESP_ERROR_CHECK(esp_console_cmd_register(&hmc_cmd));
	}

static void r_params()
	{
	struct stat st;
	char buf[40];
	//default values
	hmc_dr = 4;
	hmc_gain = 1;
	hmc_omode = 0;
	hmc_mmode = 0;
	hmc_average = 0;
	if (stat(BASE_PATH"/"PARAM_FILE, &st) != 0)
		ESP_LOGI(TAG, "params file does not exist, taking default values");
	else
 		{
		FILE *f = fopen(BASE_PATH"/"PARAM_FILE, "r");
		if (f != NULL)
			{
			if(fgets(buf, 40, f))
				{
				sscanf(buf, "%d %d %d %d %d", &hmc_average, &hmc_dr, &hmc_mmode, &hmc_gain, &hmc_omode);
				}
			fclose(f);
			}
		}	
	}
