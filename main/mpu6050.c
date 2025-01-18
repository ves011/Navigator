/*
 * mpu6050.c
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
#include <sys/unistd.h>
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
#include "mqtt_ctrl.h"
#include "utils.h"
#include "i2ccomm.h"
#include "gpios.h"	
#include "mpu6050.h"

static struct
	{
    struct arg_str *op;
    struct arg_int *arg;
    struct arg_int *arg1;
    struct arg_int *arg2;
    struct arg_end *end;
	} mpu_args;

static char *TAG = "MPU";
static char *command = "mpu";

static i2c_master_dev_handle_t mpu_handle;
static QueueHandle_t mpu_cmd_queue = NULL;
static int mpu_state = MPU_NOT_INIT;
static int acc_fs, gyro_fs;
static float acc_res, gyro_res;

//means used to reduce the offsets
static int mac_x, mac_y, mac_z, mgy_x, mgy_y, mgy_z;

static float err_m = 2., err_e = 2., p_noise = 0.1;

static float err_m_ax = 2., err_e_ax = 2., p_noise_ax = 0.01;
static float err_m_ay = 2., err_e_ay = 2., p_noise_ay = 0.01;
static float err_m_az = 2., err_e_az = 2., p_noise_az = 0.01;
static float l_est_ax = 0, k_gain_ax = 0;
static float l_est_ay = 0, k_gain_ay = 0;
static float l_est_az = 0, k_gain_az = 0;

static void kf_init_ax(float m_error, float e_error, float pn);
static float update_est_ax(float m);
static void kf_init_ay(float m_error, float e_error, float pn);
static float update_est_ay(float m);
static void kf_init_az(float m_error, float e_error, float pn);
static float update_est_az(float m);

static void IRAM_ATTR mpu_drdy_isr(void* arg)
	{
	msg_t msg;
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == MPU_DRDY_PIN)
    	{
		msg.source = 1;
		xQueueSendFromISR(mpu_cmd_queue, &msg, NULL);
		}
	}
static int mpu_activate(bool enable)
	{
	uint8_t wr_buf[2];
	int ret = ESP_FAIL;
	if(enable)
		{
		if(mpu_state == MPU_NOT_INIT)
			{
			ESP_LOGI(TAG, "Init MPU before to activate it");
			}
		else if(mpu_state == MPU_INIT)
			{
			//INT_ENABLE: enable drdy interrupt
			wr_buf[0] = INT_ENABLE;
			wr_buf[1] = 1;
			if((ret = master_transmit(mpu_handle, wr_buf, 2)) == ESP_OK)
				{
				mpu_state = MPU_ACTIVE;
				ret = ESP_OK;
				}
			}
		}
	else
		{
		if(mpu_state == MPU_ACTIVE)
			{
			//INT_ENABLE: disable interrupt
			wr_buf[0] = INT_ENABLE;
			wr_buf[1] = 0;
			if((ret = master_transmit(mpu_handle, wr_buf, 2)) == ESP_OK)
				{
				mpu_state = MPU_INIT;
				ret = ESP_OK;
				}
			}
		else
			{
			ESP_LOGI(TAG, "MPU not in active state");
			}
		}
	return ret;
	}
static int mpu_self_test()
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret;
	//self test
 	uint8_t astest[3], gstest[3];
 	float aft[3], gft[3];
 	wr_buf[0] = GYRO_CONFIG;
 	wr_buf[1] = 0xe0; // set gyro fs to +-25dps and enable self test
 	wr_buf[2] = 0xf0; // set accel fs to +-8g and enable self test
 	if((ret = master_transmit(mpu_handle, wr_buf, 3)) != ESP_OK) goto error;
 	vTaskDelay(pdMS_TO_TICKS(250));
 	
 	wr_buf[0] = SELF_TEST_X;
 	if((ret = master_transmit(mpu_handle, wr_buf, 1)) != ESP_OK) goto error;
 	if((ret = master_receive(mpu_handle, rd_buf, 4)) != ESP_OK) goto error;
 	astest[0] = ((rd_buf[0] >> 3) & 0x1c) | ((rd_buf[3] >> 4) & 0x03);
 	astest[1] = ((rd_buf[1] >> 3) & 0x1c) | ((rd_buf[3] >> 2) & 0x03);
 	astest[2] = ((rd_buf[2] >> 3) & 0x1c) | (rd_buf[3] & 0x03);
	gstest[0] = rd_buf[0] & 0x1f; gstest[1] = rd_buf[1] & 0x1f; gstest[2] = rd_buf[2] & 0x1f;
	aft[0] = (astest[0] == 0)? 0. : 1392.64 * pow(2.70588, ((float)astest[0] - 1.) / 30.);
	aft[1] = (astest[1] == 0)? 0. : 1392.64 * pow(2.70588, ((float)astest[1] - 1.) / 30.);
	aft[2] = (astest[2] == 0)? 0. : 1392.64 * pow(2.70588, ((float)astest[2] - 1.) / 30.);
	gft[0] = (gstest[0] == 0)? 0. : 3275. * pow(1.046, (float)gstest[0] - 1.);
	gft[1] = (gstest[1] == 0)? 0. : -3275. * pow(1.046, (float)gstest[1] - 1.);
	gft[2] = (gstest[2] == 0)? 0. : 3275. * pow(1.046, (float)gstest[2] - 1.);
	
	// return to initial settings
 	wr_buf[0] = GYRO_CONFIG;
 	wr_buf[1] = gyro_fs << 3; 
 	wr_buf[2] = acc_fs << 3;
 	if((ret = master_transmit(mpu_handle, wr_buf, 3)) != ESP_OK) goto error;
 	
	//ESP_LOGI(TAG, "astest: %d, %d %d", astest[0], astest[1], astest[2]);
	//ESP_LOGI(TAG, "aft: %f, %f, %f", aft[0], aft[1], aft[2]);
	//ESP_LOGI(TAG, "gstest: %d, %d %d", gstest[0], gstest[1], gstest[2]);
	//ESP_LOGI(TAG, "gft: %f, %f, %f", gft[0], gft[1], gft[2]);
	ESP_LOGI(TAG, "MPU self test accel: %f, %f, %f ", 
		((float)astest[0] - aft[0]) / aft[0],
		((float)astest[1] - aft[1]) / aft[1],
		((float)astest[2] - aft[2]) / aft[2]);
	ESP_LOGI(TAG, "MPU self test gyro: %f, %f, %f ", 
		((float)gstest[0] - gft[0]) / gft[0],
		((float)gstest[1] - gft[1]) / gft[1],
		((float)gstest[2] - gft[2]) / gft[2]); 
error:;
	return ret;
	}
static int mpu_init()
	{
	uint8_t wr_buf[10], rd_buf[10];
	int ret;
	mpu_state = MPU_NOT_INIT;
	//reset mpu6050
	wr_buf[0] = PWR_MGMT_1;
 	wr_buf[1] = 0x80;
 	ret = master_transmit(mpu_handle, wr_buf, 2);
 	vTaskDelay(pdMS_TO_TICKS(20));
	// read ID regs
	wr_buf[0] = WHO_AM_I;
	ret = master_transmit(mpu_handle, wr_buf, 1);
	if(ret == ESP_OK)
		{
		ret = master_receive(mpu_handle, rd_buf, 1);
		if(ret == ESP_OK)
			{
			ESP_LOGI(TAG, "addr %02x: %02x ", WHO_AM_I, rd_buf[0]);
			if(rd_buf[0] == 0x68)
				{
				//PWR_MGMT_1: reset = 0, sleep = 0, cycle = 0, temp_dis = 0, clck_sel 1 --> 0x01
			 	wr_buf[0] = PWR_MGMT_1;
			 	wr_buf[1] = 0x01;
			 	if((ret = master_transmit(mpu_handle, wr_buf, 2)) != ESP_OK)
			 		goto error;
			 	//SMPLRT_DIV
			 	wr_buf[0] = SMPLRT_DIV;
			 	wr_buf[1] = 100;
			 	if((ret = master_transmit(mpu_handle, wr_buf, 2)) != ESP_OK)
			 		goto error;
			 	//CONFIG: fsync disabled, dlpf = 4 --> 0x04
			 	wr_buf[0] = CONFIG;
			 	wr_buf[1] = 0x06;
			 	//if((ret = master_transmit(mpu_handle, wr_buf, 2)) != ESP_OK)
			 	//	goto error;
			 	
			 	//GYRO_CONFIG: default scale
			 	//wr_buf[0] = GYRO_CONFIG;
			 	wr_buf[2] = gyro_fs << 3;
			 	//ACCEL_CONFIG: default scale
			 	wr_buf[3] = acc_fs << 3;
			 	if((ret = master_transmit(mpu_handle, wr_buf, 4)) != ESP_OK)
			 		goto error;
			 	//ACCEL_CONFIG: default scale
			 	//wr_buf[0] = ACCEL_CONFIG;
			 	//wr_buf[1] = DEFAULT_AFS << 3;
			 	//if((ret = master_transmit(mpu_handle, wr_buf, 2)) != ESP_OK)
			 	//	goto error;
			 	//INT_PIN_CFG: int_rd_clear = 1 (int stat bits cleared on any read): 0x10
			 	wr_buf[0] = INT_PIN_CFG;
			 	wr_buf[1] = 1 << 4;
			 	//INT_ENABLE
			 	wr_buf[2] = 0;
			 	if((ret = master_transmit(mpu_handle, wr_buf, 3)) != ESP_OK)
			 		goto error;
			 	//INT_ENABLE: disable all interrupts
			 	//wr_buf[0] = INT_ENABLE;
			 	//wr_buf[1] = 0;
			 	//if((ret = master_transmit(mpu_handle, wr_buf, 2)) != ESP_OK)
			 	//	goto error;
			 	mpu_state = MPU_INIT;
		 		}
			else
				{
		 		ESP_LOGI(TAG, "Bad MPU signature: %02x instead of 0x68 ", rd_buf[0]);
		 		return ret;
		 		}
			}
		}
error:;
	return ret;
	}
void mpu_message_handler(void *pvParameters)
	{
	msg_t msg;
	uint8_t wr_buf, rd_buf[20];
	int tac_x, tac_y, tac_z, tgy_x, tgy_y, tgy_z;
	int *rac_x, *rac_y, *rac_z, *rgy_x, *rgy_y, *rgy_z; 
	int rtemp, samples = 0;
	int nr_samples = 5;
	int cal_state = 0, calibrated = 0;
	while(1)
		{
		if(xQueueReceive(mpu_cmd_queue, &msg, pdMS_TO_TICKS(1500)))
			{
			//ESP_LOGI(TAG, "DRDY int");
			if(msg.source == 1)
				{
				wr_buf = ACCEL_XOUT_H;
	 			master_transmit(mpu_handle, (uint8_t *)&wr_buf, 1);
	 			master_receive(mpu_handle, (uint8_t *)rd_buf, 14);
	 			//ESP_LOGI(TAG, "raw data: %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x ", 
	 			//			rd_buf[0], rd_buf[1], rd_buf[2], rd_buf[3], rd_buf[4], rd_buf[5], rd_buf[8], rd_buf[9], rd_buf[10], rd_buf[11], rd_buf[12], rd_buf[13]);
	 			tac_x = TO_INT(rd_buf[0], rd_buf[1]);//int16_t)(((int16_t)rd_buf[0] << 8) | rd_buf[1]);
	 			tac_y = TO_INT(rd_buf[2], rd_buf[3]);//(int16_t)(((int16_t)rd_buf[2] << 8) | rd_buf[3]);
	 			tac_z = TO_INT(rd_buf[4], rd_buf[5]);//(int16_t)(((int16_t)rd_buf[4] << 8) | rd_buf[5]);
	 			
	 			rtemp = (int16_t)(((int16_t)rd_buf[6] << 8) | rd_buf[7]);
	 			
	 			tgy_x = (int16_t)(((int16_t)rd_buf[8] << 8) | (int)rd_buf[9]);
	 			tgy_y = (int16_t)(((int16_t)rd_buf[10] << 8) | (int)rd_buf[11]);
	 			tgy_z = (int16_t)(((int16_t)rd_buf[12] << 8) | (int)rd_buf[13]);
	 			//if(!calibrated)
	 				//ESP_LOGI(TAG, "raw accel: %6d %6d %6d %6d raw gyro : %6d %6d %6d ",	tac_x, tac_y, tac_z, rtemp, tgy_x, tgy_y, tgy_z);
	 			if(calibrated)
	 				{
	 				float ax, ay, az, gx, gy, gz;
	 				float eax, eay, eaz, egx, egy, egz;
	 				ax = (float)(tac_x - mac_x) / acc_res;
	 				ay = (float)(tac_y - mac_y) / acc_res;
	 				az = (float)(tac_z - mac_z) / acc_res;
	 				gx = (float)(tgy_x - mgy_x) / gyro_res;
	 				gy = (float)(tgy_y - mgy_y) / gyro_res;
	 				gz = (float)(tgy_z - mgy_z) / gyro_res;
	 				eax = update_est_ax(ax);
	 				eay = update_est_ax(ay);
	 				eaz = update_est_ax(az);
	 				egx = update_est_ax(gx);
	 				egy = update_est_ax(gy);
	 				egz = update_est_ax(gz); 
	 				//ESP_LOGI(TAG, "acc (dps): %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f gyro(m/sec2): %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f ", 
	 				//	ax, eax, ay, eay, az, eaz, gx, egx, gy, egy, gz, egz);
	 				my_printf("%8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f ", 
	 					ax, eax, ay, eay, az, eaz, gx, egx, gy, egy, gz, egz);
	 				}
	 			if(cal_state)
	 				{
					if(samples < 5)
						{
						samples++;
						continue;
						}
					if(samples < nr_samples)
						{
						mac_x += tac_x; mac_y += tac_y; mac_z += tac_z;
						mgy_x += tgy_x; mgy_y += tgy_y; mgy_z += tgy_z;
						rac_x[samples - 5] = tac_x; rac_y[samples - 5] = tac_y; rac_z[samples - 5] = tac_z;
						rgy_x[samples - 5] = tgy_x; rgy_y[samples - 5] = tgy_y; rgy_z[samples - 5] = tgy_z; 
						samples++;
						}
					else
						{
						mpu_activate(0);
						samples -= 5;
						cal_state = 0;
						mac_x /= samples; mac_y /= samples; mac_z /= samples;
						mgy_x /= samples; mgy_y /= samples; mgy_z /= samples;
						float sdac_x = 0, sdac_y = 0, sdac_z = 0, sdgy_x = 0, sdgy_y = 0, sdgy_z = 0;
						for(int i = 0; i < samples; i++)
							{
							sdac_x += (rac_x[i] - mac_x) * (rac_x[i] - mac_x);
							sdac_y += (rac_y[i] - mac_y) * (rac_y[i] - mac_y);
							sdac_z += (rac_z[i] - mac_z) * (rac_z[i] - mac_z);
							sdgy_x += (rgy_x[i] - mgy_x) * (rgy_x[i] - mgy_x);
							sdgy_y += (rgy_y[i] - mgy_y) * (rgy_y[i] - mgy_y);
							sdgy_z += (rgy_z[i] - mgy_z) * (rgy_z[i] - mgy_z); 
							}
						sdac_x = sqrt(sdac_x / (samples -1));
						sdac_y = sqrt(sdac_y / (samples -1));
						sdac_z = sqrt(sdac_z / (samples -1));
						sdgy_x = sqrt(sdgy_x / (samples -1));
						sdgy_y = sqrt(sdgy_y / (samples -1));
						sdgy_z = sqrt(sdgy_z / (samples -1));
						calibrated = 1;
						ESP_LOGI(TAG, "\noffset val nrs: %d\n\t%d, %d, %d, %d, %d, %d ", samples, mac_x, mac_y, mac_z, mgy_x, mgy_y, mgy_z);
						ESP_LOGI(TAG, "\t%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ", sdac_x, sdac_y, sdac_z, sdgy_x, sdgy_y, sdgy_z);
						free(rac_x); free(rac_y); free(rac_z);	free(rgy_x); free(rgy_y); free(rgy_z);
						}
					}
				}
			else if(msg.source == 2) // start calibration 
				{
				mpu_activate(0);
				nr_samples = msg.val;
				rac_x = malloc(nr_samples * sizeof(int));
				if(rac_x)
					{
					rac_y = malloc(nr_samples * sizeof(int));
					if(rac_y)
						{
						rac_z = malloc(nr_samples * sizeof(int));
						if(rac_z)
							{
							rgy_x = malloc(nr_samples * sizeof(int));
							if(rgy_x)
								{
								rgy_y = malloc(nr_samples * sizeof(int));
								if(rgy_y)
									{
									rgy_z = malloc(nr_samples * sizeof(int));
									if(rgy_z)
										{
										samples = 0;
										mac_x = 0, mac_y = 0, mac_z = 0, mgy_x = 0, mgy_y = 0, mgy_z = 0;
										cal_state = 1;
										calibrated = 0;
										mpu_activate(1);
										}
									else
										{
										free(rac_x); free(rac_y); free(rac_z);	free(rgy_x); free(rgy_y);
										}
									}
								else 
									{
									free(rac_x); free(rac_y); free(rac_z); free(rgy_x);
									}
								}
							else
								{
								free(rac_x); free(rac_y); free(rac_z);
								}
							}
						else
							{
							free(rac_x); free(rac_y);
							}
						}
					else
						{
						free(rac_x);
						}
					}
				}
			}
		}
	}
static void mpu_cal(int nrs)
	{
	msg_t msg;
	msg.source = 2;
	msg.val = nrs;
	xQueueSend(mpu_cmd_queue, &msg, pdMS_TO_TICKS(10));
	}
int do_mpu(int argc, char **argv)
	{
	int nrs;
	msg_t msg;
	if(strcmp(argv[0], command))
		return 0;
	int nerrors = arg_parse(argc, argv, (void **)&mpu_args);
	if (nerrors != 0)
		{
		arg_print_errors(stderr, mpu_args.end, argv[0]);
		return ESP_FAIL;
		}
	if(strcmp(mpu_args.op->sval[0], "init") == 0)
		mpu_init();
	else if(strcmp(mpu_args.op->sval[0], "activate") == 0)
		{
		if(mpu_args.arg->count)
			mpu_activate(mpu_args.arg->ival[0]);
		}
	else if(strcmp(mpu_args.op->sval[0], "selftest") == 0)
		mpu_self_test();
	else if(strcmp(mpu_args.op->sval[0], "cal") == 0)
		{
		if(mpu_args.arg->count)
			nrs = mpu_args.arg->ival[0];
		else 
			nrs = 100;
		msg.source = 2;
		msg.val = nrs;
		xQueueSend(mpu_cmd_queue, &msg, pdMS_TO_TICKS(10));
		}
	else if(strcmp(mpu_args.op->sval[0], "kf") == 0)
		{
		
		if(mpu_args.arg->count && mpu_args.arg1->count && mpu_args.arg2->count)
			{
			err_m = ((float)mpu_args.arg->ival[0])/1000.;
			err_e = ((float)mpu_args.arg1->ival[0])/1000.;
			p_noise = ((float)mpu_args.arg2->ival[0])/1000.;
			kf_init_ax(err_m, err_e, p_noise);
			kf_init_ay(err_m, err_e, p_noise);
			kf_init_az(err_m, err_e, p_noise);
			ESP_LOGI(TAG, "kf params: %f %f %f", err_m, err_e, p_noise);	
			}
		else 
			{
			ESP_LOGI(TAG, "not enough parameters. Actual values: %f %f %f", err_m, err_e, p_noise);
			}
		}
	return ESP_OK;
	}
void register_mpu()
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MPU_DRDY_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //gpio_install_isr_service(0);
    gpio_isr_handler_add(MPU_DRDY_PIN, mpu_drdy_isr, (void*) MPU_DRDY_PIN);
    mpu_cmd_queue = xQueueCreate(10, sizeof(msg_t));
	acc_fs = DEFAULT_AFS;
	gyro_fs = DEFAULT_GFS;
	if(gyro_fs == 0) gyro_res = 65536./500.;
	else if(gyro_fs == 1) gyro_res = 65536./1000.;
	else if(gyro_fs == 2) gyro_res = 65536./2000.;
	else if(gyro_fs == 3) gyro_res = 65536./4000.;
	
	if(acc_fs == 0) acc_res = 65536. / 4. / 9.807;
	else if(acc_fs == 1) acc_res = 65536. / 8. / 9.807;
	else if(acc_fs == 2) acc_res = 65536. / 16. / 9.807;
	else if(acc_fs == 3) acc_res = 65536. / 32. / 9.807;
	
	mac_x = 0, mac_y = 0, mac_z = 0, mgy_x = 0, mgy_y = 0, mgy_z = 0;
	
	//kf_init(me, ee, n);
	
    i2c_device_config_t dev_cfg = {
    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    	.scl_speed_hz = 100000,
		};


	if(i2c_master_probe(i2c_bus_handle_0, MPU_I2C_ADDRESS, -1) == ESP_OK) //pdMS_TO_TICKS(10)) == ESP_OK)
		{
		dev_cfg.device_address = MPU_I2C_ADDRESS;
		ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle_0, &dev_cfg, &mpu_handle));
		ESP_LOGI(TAG, "MPU@%x found", MPU_I2C_ADDRESS);
		mpu_init();
		mpu_self_test();
		}
	else 
		{
		ESP_LOGI(TAG, "MPU@%x not found", MPU_I2C_ADDRESS);
		mpu_handle = NULL;
		}
	
	mpu_args.op = arg_str1(NULL, NULL, "<op>", "op: activate | init | selftest");
	mpu_args.arg = arg_int0(NULL, NULL, "<#>", "#0 | #1");
	mpu_args.arg1 = arg_int0(NULL, NULL, "<#>", "#0 | #1");
	mpu_args.arg2 = arg_int0(NULL, NULL, "<#>", "#0 | #1");
	mpu_args.end = arg_end(1);
	const esp_console_cmd_t mpu_cmd =
		{
		.command = "mpu",
		.help = "mpu commands",
		.hint = NULL,
		.func = &do_mpu,
		.argtable = &mpu_args
		};
	ESP_ERROR_CHECK(esp_console_cmd_register(&mpu_cmd));
	if(xTaskCreate(mpu_message_handler, "MPU msg handler", 8192, NULL, 5, NULL) != pdPASS)
		{
		ESP_LOGE(TAG, "Cannot create mpu_message_handler task");
		esp_restart();
		}	
	mpu_cal(100);
	kf_init_ax(err_m, err_e, p_noise);
	kf_init_ay(err_m, err_e, p_noise);
	kf_init_az(err_m, err_e, p_noise);
	}
	
//Kalman filter
static void kf_init_ax(float m_error, float e_error, float pn)
	{
	err_m_ax = m_error;
	err_e_ax = e_error;
	p_noise_ax = pn;
	l_est_ax = 0, k_gain_ax = 0;	
	}
	
static float update_est_ax(float m)
	{
  	k_gain_ax = err_e_ax / (err_e_ax + err_m_ax);
  	float c_est = l_est_ax + k_gain_ax * (m - l_est_ax);
  	err_e_ax = (1.0f - k_gain_ax) * err_e_ax + fabsf(l_est_ax - c_est) * p_noise_ax;
  	l_est_ax = c_est;
  	return c_est;
	}

static void kf_init_ay(float m_error, float e_error, float pn)
	{
	err_m_ay = m_error;
	err_e_ay = e_error;
	p_noise_ay = pn;
	l_est_ay = 0, k_gain_ay = 0;	
	}
	
static float update_est_ay(float m)
	{
  	k_gain_ay = err_e_ay / (err_e_ay + err_m_ay);
  	float c_est = l_est_ay + k_gain_ay * (m - l_est_ay);
  	err_e_ay = (1.0f - k_gain_ay) * err_e_ay + fabsf(l_est_ay - c_est) * p_noise_ay;
  	l_est_ay = c_est;
  	return c_est;
	}
	
static void kf_init_az(float m_error, float e_error, float pn)
	{
	err_m_az = m_error;
	err_e_az = e_error;
	p_noise_az = pn;
	l_est_az = 0, k_gain_az = 0;	
	}
	
static float update_est_az(float m)
	{
  	k_gain_az = err_e_az / (err_e_az + err_m_az);
  	float c_est = l_est_az + k_gain_az * (m - l_est_az);
  	err_e_az = (1.0f - k_gain_az) * err_e_az + fabsf(l_est_az - c_est) * p_noise_az;
  	l_est_az = c_est;
  	return c_est;
	}
