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
#include "esp_system.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "esp_netif.h"
#include "sys/stat.h"
#include "math.h"
#include "project_specific.h"
#include "common_defines.h"
#include "external_defs.h"
#include "kalman.h"
#include "gpios.h"	
#include "i2ccomm.h"
#include "utils.h"
#include "hmc5883.h"

#define PARAM_FILE	"hmc_params.txt"

static const char *TAG = "hmc";
static char *command = "hmc";

//dr - data rate, gain, mode, average
static int hmc_dr = 4 /* 15Hz */, hmc_gain = 0 /* +/-0.88G*/, hmc_mode = 0 /* continuous mode */, hmc_average = 2 /* 4 samples average*/;
static float bias_x = -459.910, bias_y = -315.867, bias_z = -382.403;
static float corr_mat[3][3] = {{ 0.2083, -0.000352, -0.000839},
								{-0.000352, 0.179401, 0.016923},
								{-0.000839, 0.016923, 0.275263}};
static i2c_master_dev_handle_t hmc_handle;
//command q and data q
static QueueHandle_t hmc_cmd_queue = NULL;
static void r_params();
static ks_filter_t kx, ky, kz;
static float err_m = 30., err_e = 3., p_noise = 0.5;

static float hmc_scale[8] = {0.73, 0.92, 1.22, 1.52, 2.27, 2.56, 3.03, 4.35};


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
		msg.source = HMC_DATA;
		msg.vts.ts = esp_timer_get_time();
		xQueueSendFromISR(hmc_cmd_queue, &msg, NULL);
		}
	}
static void hmc_message_handler(void *pvParameters)
	{
	msg_t msg;
	uint8_t disp_val = 0;
	uint8_t wr_buf[4], rd_buf[10];
	int16_t x, y, z;
	while(1)
		{
		if(xQueueReceive(hmc_cmd_queue, &msg, pdMS_TO_TICKS(1500)))
			{
			if(msg.source == HMC_CMD)
				{
				if(msg.val == HMC_CMD_DISP_VAL)
					{
					disp_val = msg.vts.m_val[0];
					}
				}
			else if(msg.source == HMC_DATA)
				{
				wr_buf[0] = HMC_OUTPUTX;
				master_transmit(hmc_handle, wr_buf, 1);
				master_receive(hmc_handle, rd_buf, 6);
				x = (rd_buf[0] << 8 & 0xff00) | rd_buf[1];
				z = (rd_buf[2] << 8 & 0xff00) | rd_buf[3];
				y = (rd_buf[4] << 8 & 0xff00) | rd_buf[5];
				float fx = (float)x * hmc_scale[hmc_gain];
				float fy = (float)y * hmc_scale[hmc_gain];
				float fz = (float)z * hmc_scale[hmc_gain];
				float tfx = ksf_update_est(fx, &kx);// - bias_x;
				float tfy = ksf_update_est(fy, &ky);// - bias_y;
				float tfz = ksf_update_est(fz, &kz);// - bias_z;
				float ffx = corr_mat[0][0] * (tfx - bias_x) + corr_mat[0][1] * (tfy - bias_y) + corr_mat[0][2] * (tfz - bias_z);
				float ffy = corr_mat[1][0] * (tfx - bias_x) + corr_mat[1][1] * (tfy - bias_y) + corr_mat[1][2] * (tfz - bias_z);
				float ffz = corr_mat[2][0] * (tfx - bias_x) + corr_mat[2][1] * (tfy - bias_y) + corr_mat[2][2] * (tfz - bias_z);
				float atyx = atan2(ffy , ffx) * 180. / 3.14159265358979;
				float atxy = atan2(ffx , ffy) * 180. / 3.14159265358979;
				//float atzx = atan2(ffz , ffx) * 180. / 3.14159265358979;
				float tf = sqrt(ffx * ffx + ffy * ffy + ffz * ffz);
				msg.source = SENSOR_DATA;
	 			msg.val = HMC_SENSOR_DATA;
	 			msg.ifvals.fval[0] = ffx; msg.ifvals.fval[1] = ffy; msg.ifvals.fval[2] = ffz;
	 			xQueueSend(dev_mon_queue, &msg, pdMS_TO_TICKS(10));
				if(disp_val)
					{
					// use with magento for hard/soft iron calibration
					my_printf("%10llu\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.2f\t%.2f\t%.5f", msg.vts.ts, tfx, tfy, tfz, ffx, ffy, ffz, atyx, atxy, tf);
					
					//ESP_LOGI(TAG, "%.3f %.3f %.3f %.3f", fx, fy, fz, tf);
					//ESP_LOGI(TAG, "%7.2f %7.2f", atyx, atxy);
					}
				}
			}
		}
	}
static int hmc_init(i2c_master_dev_handle_t handle)
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret;
	// read ID regs
	wr_buf[0] = HMC_ID;
	ret = i2c_master_transmit(handle, wr_buf, 1, 50);
	if(ret == ESP_OK)
		{
		ret = i2c_master_receive(handle, rd_buf, 3, 50);
		if(ret == ESP_OK)
			{
			ESP_LOGI(TAG, "addr 10: %02x, addr 11: %02x, addr 12: %02x", rd_buf[0], rd_buf[1], rd_buf[2]);
			wr_buf[0] = HMC_CONFIGA; 
			wr_buf[1] = ((hmc_dr << 2) & 0x1c) | hmc_mode;
			ESP_LOGI(TAG, "A: %u", wr_buf[1]);
			ret = master_transmit(hmc_handle, wr_buf, 2);
			wr_buf[0] = HMC_CONFIGB; 
			wr_buf[1] = ((hmc_gain << 5) & 0xe0);
			ESP_LOGI(TAG, "B: %u", wr_buf[1]);
			ret = master_transmit(hmc_handle, wr_buf, 2);
			wr_buf[0] = HMC_MODE;
			wr_buf[1] = hmc_mode;
			ESP_LOGI(TAG, "M: %u", wr_buf[1]);
			ret = master_transmit(hmc_handle, wr_buf, 2);
			}
		}
	return ret;
	}
int do_hmc(int argc, char **argv)
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret, wr_size, rd_size;
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
			msg.vts.m_val[0] = 1;
		else
			msg.vts.m_val[0] = hmc_args.arg->ival[0];
		msg.source = HMC_CMD;
		msg.val = HMC_CMD_DISP_VAL;
		xQueueSend(hmc_cmd_queue, &msg, pdMS_TO_TICKS(50));
		}
	else if(strcmp(hmc_args.op->sval[0], "params") == 0)
		{
		ESP_LOGI(TAG, "hmc_dr: %d, hmc_gain: %d, hmc_mode: %d, hmc_average: %d", 
			hmc_dr, hmc_gain, hmc_mode, hmc_average);
		ESP_LOGI(TAG, "Bias x: %.5f y: %.5f z: %.5f", bias_x, bias_y, bias_z);
		for(int i = 0; i < 3; i++)
			ESP_LOGI(TAG, "corr_mat [%d][0]: %.5f [%d][1]: %.5f [%d][2]: %.5f", i, corr_mat[i][0], i, corr_mat[i][1], i, corr_mat[i][2]);
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
	if(!hmc_cmd_queue)
		{
		ESP_LOGE(TAG, "Cannot create cmd queue for HMC");
		esp_restart();
		}


	i2c_device_config_t dev_cfg = {
    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    	.scl_speed_hz = 100000,
		};
	r_params();
	ksf_init(err_m, err_e, p_noise, &kx);
	ksf_init(err_m, err_e, p_noise, &ky);
	ksf_init(err_m, err_e, p_noise, &kz);
	if(i2c_master_probe(i2c_bus_handle_0, HMC_I2C_ADDRESS, -1) == ESP_OK) //pdMS_TO_TICKS(10)) == ESP_OK)
		{
		dev_cfg.device_address = HMC_I2C_ADDRESS;
		ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle_0, &dev_cfg, &hmc_handle));
		ESP_LOGI(TAG, "HMC@%x found", HMC_I2C_ADDRESS);
		if(hmc_init(hmc_handle) == ESP_OK)
			{
			if(xTaskCreate(hmc_message_handler, "HMC msg handler", 8192, NULL, 5, NULL) != pdPASS)
				{
				ESP_LOGE(TAG, "Cannot create hmg_message_handler task");
				esp_restart();
				}	
			}
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
	char buf[60];
	if (stat(BASE_PATH"/"PARAM_FILE, &st) != 0)
		ESP_LOGI(TAG, "params file does not exist, taking default values");
	else
 		{
		FILE *f = fopen(BASE_PATH"/"PARAM_FILE, "r");
		if (f != NULL)
			{
			if(fgets(buf, 40, f))
				{
				sscanf(buf, "%d %d %d %d", &hmc_average, &hmc_dr, &hmc_mode, &hmc_gain);
				if(fgets(buf, 50, f))
					{
					sscanf(buf, "%f %f %f", &bias_x, &bias_y, &bias_z);
					}
				for(int i = 0; i < 3; i++)
					{
					if(fgets(buf, 50, f))
						{
						sscanf(buf, "%f %f %f", &corr_mat[i][0], &corr_mat[i][1], &corr_mat[i][2]);
						}
					}
				}
			fclose(f);
			}
		}	
	}
