/*
 * navi_mon.c
 *
 *  Created on: Jan 19, 2025
 *      Author: viorel_serbu
 */


#include <string.h>
#include <stdio.h>
#include "hal/gpio_types.h"
#include "project_specific.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_spiffs.h"
#include "sys/stat.h"
#include "driver/i2c_master.h"
#include "math.h"
#include "common_defines.h"
#include "external_defs.h"
#include "gpios.h"
#include "adc_op.h"
#include "kalman.h"
#include "navi_mon.h"

static char *TAG = "NAVI MON";
static int nw_state = 0, remote_state = 0;
static int nw_led_state = 0, remote_led_state = 0;
static uint32_t int_tick;
static int bat_flashing_state = 0;
int bat_val;

// initial parameters for kalman battery filter
// convergence is ~5 samples
//static float err_m = 20., err_e = 20., p_noise = 1., l_est = 0., k_gain = 0.;
//---------------------------------------------

#define NW_LED_SET(on)		gpio_set_level(NW_CONNECT_ON, on); gpio_set_level(NW_CONNECT_OFF, 1 - on);
#define REMOTE_LED_SET(on)	gpio_set_level(REMOTE_CONNECT_ON, on); gpio_set_level(REMOTE_CONNECT_OFF, 1 - on);

static void mon_timer_callback(void* arg)
	{
	msg_t msg;
	int_tick++;
	if(int_tick % BAT_MON_INTERVAL == 0)
		{
		msg.source = MSG_BAT;
		xQueueSendFromISR(dev_mon_queue, &msg, NULL);
		}
	if(bat_flashing_state && int_tick &&(int_tick % bat_flashing_state ==0))
		{
		msg.source = MSG_LED_FLASH;
		xQueueSendFromISR(dev_mon_queue, &msg, NULL);
		}
	}
void navi_mon_task(void *pvParameters)
	{
	msg_t msg;
	ks_filter_t b_filter;
	ksf_init(20., 20., 1, &b_filter);
	esp_timer_create_args_t mon_timer_args = 
		{
    	.callback = &mon_timer_callback,
        .name = "mon_timer"
    	};
	dev_mon_queue = xQueueCreate(10, sizeof(msg_t));
	if(!dev_mon_queue)
		{
		ESP_LOGE(TAG, "Cannot create dev_mon_queue");
		esp_restart();
		}
	esp_timer_handle_t mon_timer;
	ESP_ERROR_CHECK(esp_timer_create(&mon_timer_args, &mon_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(mon_timer, 100000)); // 100 msce period
	while(1)
		{
		if(xQueueReceive(dev_mon_queue, &msg, portMAX_DELAY))
			{
			switch(msg.source)
				{
				case MSG_WIFI:
					nw_state = msg.val;
					NW_LED_SET(nw_state);
					break;
				case MSG_BAT:
					{
					int *s_data[1], s1_data[5];
					bat_val = 0;
					s_data[0] = s1_data;
					int chn[] = {BAT_ADC_CHANNEL};
					adc_get_data(chn, 1, (int **)&s_data, 5);
					for(int i = 0; i < 5; i++) bat_val += s1_data[i];
					bat_val /= 5;
					bat_val = ksf_update_est(bat_val, &b_filter);
					//kalman filter
					if(bat_val > BAT_OK)
						{
						bat_flashing_state = 0;
						NW_LED_SET(nw_state);
						REMOTE_LED_SET(remote_state);
						}
					else if(bat_val <= BAT_OK && bat_val > BAT_LOW)
						bat_flashing_state = BAT_FLASHING_MED;
					else
						{
						bat_flashing_state = BAT_FLASHING_LOW;
						//ESP_LOGI(TAG, "bat: %d %d", bv, bat_flashing_state);
						}
					break;
					}
				case MSG_LED_FLASH:
					//gpio_set_level(NW_CONNECT_ON, nw_state & nw_led_state);
					gpio_set_level(NW_CONNECT_OFF, nw_led_state);
					nw_led_state = 1 - nw_led_state;
					gpio_set_level(REMOTE_CONNECT_OFF, nw_led_state);
					break;
				default:
					break;
				}
			}
		}
	}