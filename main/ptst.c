/*
 * ptst.c
 *
 *  Created on: Jan 4, 2025
 *      Author: viorel_serbu
 */


#include <string.h>
#include <stdio.h>
#include "hal/gpio_types.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "esp_netif.h"
#include "driver/gptimer.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "math.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "sys/stat.h"
#include "common_defines.h"
#include "utils.h"
#include "gpios.h"
#include "adc_op.h"
#include "ptst.h"

static struct
	{
    struct arg_str *op;
    struct arg_int *arg;
    struct arg_end *end;
	} ptst_args;
	
static const char *TAG = "PTST";
static char *command = "ptst";

static int pwm_f = 20; 	// GA25Y310 DC gear motor
						// the highest frequency for which the torque at low speed is acceptable 
						// for higher frequencies is very small
static int lpwm_dc = 50, rpwm_dc = 50;
static float lr_dc_ratio = 1.;

static uint32_t left_ctrl1 = 0, left_ctrl2 =0, right_ctrl1 = 0, right_ctrl2 = 0;
static int ledc_timer_res = LEDC_TIMER_11_BIT;

static QueueHandle_t pt_mon_queue = NULL;
static TaskHandle_t pt_mon_task_handle;

int ptst_save_params();
int ptst_read_params();

static void IRAM_ATTR moten_isr_handler(void* arg)
	{
	msg_t msg;
	msg.source = PT_ENABLED;
	xQueueSendFromISR(pt_mon_queue, &msg, NULL);
	}
	
static void config_ptst_gpio()
	{
	gpio_config_t io_conf;
	
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  ( 1ULL << ENLEFTM 		| 1ULL << ENRIGHTM 
    						| 1ULL << LEFTM_CTRL1 	| 1ULL << LEFTM_CTRL2 
    						| 1ULL << RIGHTM_CTRL1 	| 1ULL << RIGHTM_CTRL2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(LEFTM_CTRL1, left_ctrl1);
    gpio_set_level(LEFTM_CTRL2, left_ctrl2);
    gpio_set_level(RIGHTM_CTRL1, right_ctrl1);
    gpio_set_level(RIGHTM_CTRL2, right_ctrl2);
    gpio_set_level(ENLEFTM, 0);
    gpio_set_level(ENRIGHTM, 0);
    
    // single interrupt for ENLEFTM ored with ENRIGHTM
    io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =  ( 1ULL << MOTEN_INTR); 
    gpio_config(&io_conf);
    gpio_isr_handler_add(MOTEN_INTR, moten_isr_handler, NULL);
    
    ledc_timer_config_t ledc_timer = 
    	{
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = ledc_timer_res,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = pwm_f,  // Set output frequency at 1 kHz
        .clk_cfg          = SOC_MOD_CLK_XTAL
    	};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = 
    	{
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ENLEFTM,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    	};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = ENRIGHTM;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (lpwm_dc << LEDC_TIMER_10_BIT) / 100);
    //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (rpwm_dc << LEDC_TIMER_10_BIT) / 100);
    //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    /*
    esp_timer_handle_t pwm_timer;
	const esp_timer_create_args_t pwm_timer_args = 
		{
        .callback = &pwm_timer_callback,
        .name = "pwm timer"
    	};
    ESP_ERROR_CHECK(esp_timer_create(&pwm_timer_args, &pwm_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_timer, 1000)); // every 20 msec
    */
	}

int do_ptst(int argc, char **argv)
	{
	if(strcmp(argv[0], command))
		return 0;
	int nerrors = arg_parse(argc, argv, (void **)&ptst_args);
	if (nerrors != 0)
		{
		arg_print_errors(stderr, ptst_args.end, argv[0]);
		return ESP_FAIL;
		}
	if(strcmp(ptst_args.op->sval[0], "fr") == 0)
		{
		if(ptst_args.arg->count)
			{
			pwm_f = ptst_args.arg->ival[0];
			ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)pwm_f);
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "dcl") == 0)
		{
		if(ptst_args.arg->count)
			{
			lpwm_dc = ptst_args.arg->ival[0];
			if(lpwm_dc == 0)
				ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
			else if(lpwm_dc >= 100)
				ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);
			else
				{
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (lpwm_dc << ledc_timer_res) / 100);
	    		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
	    		}
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "dcr") == 0)
		{
		if(ptst_args.arg->count)
			{
			rpwm_dc = ptst_args.arg->ival[0];
			if(rpwm_dc == 0)
				ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
			else if(rpwm_dc >= 100)
				ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);
			else
				{
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (rpwm_dc << ledc_timer_res) / 100);
	    		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
	    		}
			}
		}
	
	else if(strcmp(ptst_args.op->sval[0], "f") == 0)
		{
		if(ptst_args.arg->count)
			{
			left_ctrl1 = 1;
			left_ctrl2 = 0;
			right_ctrl1 = 1;
			right_ctrl2 = 0;
			lpwm_dc = (ptst_args.arg->ival[0] << ledc_timer_res) / 100; 
			rpwm_dc = (int)((float)lpwm_dc * lr_dc_ratio);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, lpwm_dc);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rpwm_dc);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			gpio_set_level(LEFTM_CTRL1, left_ctrl1);
			gpio_set_level(LEFTM_CTRL2, left_ctrl2);
			gpio_set_level(RIGHTM_CTRL1, right_ctrl1);
			gpio_set_level(RIGHTM_CTRL2, right_ctrl2);
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "b") == 0)
		{
		if(ptst_args.arg->count)
			{
			left_ctrl1 = 0;
			left_ctrl2 = 1;
			right_ctrl1 = 0;
			right_ctrl2 = 1;
			lpwm_dc = (ptst_args.arg->ival[0] << ledc_timer_res) / 100; 
			rpwm_dc = (int)((float)lpwm_dc * lr_dc_ratio);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, lpwm_dc);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rpwm_dc);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			gpio_set_level(LEFTM_CTRL1, left_ctrl1);
			gpio_set_level(LEFTM_CTRL2, left_ctrl2);
			gpio_set_level(RIGHTM_CTRL1, right_ctrl1);
			gpio_set_level(RIGHTM_CTRL2, right_ctrl2);
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "l") == 0)
		{
		if(ptst_args.arg->count)
			{
			left_ctrl1 = 0;
			left_ctrl2 = 0;
			right_ctrl1 = 1;
			right_ctrl2 = 0;
			rpwm_dc = (ptst_args.arg->ival[0] << ledc_timer_res) / 100; 
			lpwm_dc = 0;
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, lpwm_dc);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rpwm_dc);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			gpio_set_level(LEFTM_CTRL1, left_ctrl1);
			gpio_set_level(LEFTM_CTRL2, left_ctrl2);
			gpio_set_level(RIGHTM_CTRL1, right_ctrl1);
			gpio_set_level(RIGHTM_CTRL2, right_ctrl2);
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "r") == 0)
		{
		if(ptst_args.arg->count)
			{
			left_ctrl1 = 1;
			left_ctrl2 = 0;
			right_ctrl1 = 0;
			right_ctrl2 = 0;
			lpwm_dc = (ptst_args.arg->ival[0] << ledc_timer_res) / 100; 
			rpwm_dc = 0;
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, lpwm_dc);
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rpwm_dc);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			gpio_set_level(LEFTM_CTRL1, left_ctrl1);
			gpio_set_level(LEFTM_CTRL2, left_ctrl2);
			gpio_set_level(RIGHTM_CTRL1, right_ctrl1);
			gpio_set_level(RIGHTM_CTRL2, right_ctrl2);
			}
		}
	else if(strcmp(ptst_args.op->sval[0], "lrdc") == 0)
		{
		if(ptst_args.arg->count)
			{
			lr_dc_ratio = (float)ptst_args.arg->ival[0] / 1000.;
			ptst_save_params();
			}
		else
			ESP_LOGI(TAG, "lr_dc_ratio: %f", lr_dc_ratio);
		}
	return ESP_OK;
	}
/*
ADC cycle takes nrs * 0.4msec
The ADC cycle must be smaller than smallest pulse width
smallest pulse with is 15% of pwm_f which is 20Hz:  7.5msec
need to allow some overhead for interrupts and processing
For a sample rate of 0.4msec ADC can get 8 samples in 3.2msec.
the rest is for overhead

*/
static void pt_mon_task(void *pvParameters)
	{
	uint64_t ts;
	int nrs = 8; //total number of ADC samples
	int igs = 4; // first samples to be ignored
	int adc_chn[3] = {SENSE_ML, SENSE_MR, SENSE_BAT};
	int *adc_data[3];
	int ml_data[100], mr_data[100], b_data[100];
	adc_data[0] = ml_data;
	adc_data[1] = mr_data;
	adc_data[2] = b_data;
	while(1)
		{
		msg_t msg;
		xQueueReceive(pt_mon_queue, &msg, portMAX_DELAY);
		if(msg.source == PT_ENABLED)
			{
			// get motor sense values
			//ESP_LOGI(TAG, "PT_ENABLED");
			ts = esp_timer_get_time();
			adc_get_data(adc_chn, 3, adc_data, nrs);
			int ml = 0, mr = 0, bat = 0;
			for(int i = igs; i < nrs; i++)
				{
				ml += ml_data[i];
				mr += mr_data[i];
				bat += b_data[i];
				}
			ml /= (nrs - igs); mr /= (nrs - igs); bat /= (nrs - igs);
			ts = esp_timer_get_time() - ts;
			//ESP_LOGI(TAG, "%llu %d %d %d", ts, ml, mr, bat);
			}
		}
	}
void register_ptst()
	{
	ptst_args.op = arg_str1(NULL, NULL, "<op>", "op: f | b | l | r | s | fr | dc");
	ptst_args.arg = arg_int1(NULL, NULL, "<speed> | <angle>", "channel to read");
	ptst_args.end = arg_end(1);
	const esp_console_cmd_t ptst_cmd =
		{
		.command = command,
		.help = "fw(f) | bw(b) | left(l) | right(r) speed | angle",
		.hint = NULL,
		.func = &do_ptst,
		.argtable = &ptst_args
		};
	ESP_ERROR_CHECK(esp_console_cmd_register(&ptst_cmd));
	pt_mon_queue = xQueueCreate(10, sizeof(msg_t));
	xTaskCreate(pt_mon_task, "pt monitor task", 8192, NULL, 5, &pt_mon_task_handle);
	if(!pt_mon_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start pt monitor task");
		esp_restart();
		}
	ptst_read_params();
	config_ptst_gpio();	
	}
int ptst_save_params()
	{
	int ret = ESP_FAIL;
	char buf[50];
	FILE *f = fopen(BASE_PATH"/"PTST_FILE, "w");
	if (f == NULL)
		ESP_LOGE(TAG, "Failed to create console %s", PTST_FILE);
	else
		{
		sprintf(buf, "%f\n", lr_dc_ratio);
		if(fputs(buf, f) >= 0)
			ret = ESP_OK;
		fclose(f);
		}
	return ret;
	}
int ptst_read_params()
	{
	struct stat st;
	char buf[50];
	if (stat(BASE_PATH"/"CONSOLE_FILE, &st) != 0)
		{
		// file does no exists
		ESP_LOGI(TAG, "%s not found. Return default parameters: lr_dc_ratio = %f", PTST_FILE, lr_dc_ratio);
		}
	else
		{
		FILE *f = fopen(BASE_PATH"/"PTST_FILE, "r");
		if (f != NULL)
			{
			if(fgets(buf, 48, f))
				{
				lr_dc_ratio = atof(buf);
				}
			else
 				{
				ESP_LOGI(TAG, "%s read error. Return default parameters: lr_dc_ratio = %f", PTST_FILE, lr_dc_ratio);	 
				}
			}
		else
			{
			ESP_LOGI(TAG, "%s cannot open for read. Return default parameters: lr_dc_ratio = %f", PTST_FILE, lr_dc_ratio);	 
			}
		}
	return ESP_OK;
	}