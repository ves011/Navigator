/*
 * process_mesages.c
 *
 *  Created on: Dec 17, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "bignum_mod.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "nmea_parser.h"
#include "common_defines.h"
#include "tcp_server.h"

//current gps location, last gps_location
extern gps_t c_gps_location, l_gps_location;

//remote device gps_location
extern gps_t r_gps_location;

static const char *TAG = "SPROCM";

void process_message(socket_message_t msg)
	{
	switch(msg.cmd_id)
		{
		case SEND_TO_SERIAL:
			printf("%s", msg.p.payload);
//			sprintf(buf, "%02x %02x %02x %02x %02x\n%02x %02x %02x %02x", 
//			msg.p.payload[0], msg.p.payload[1], msg.p.payload[2], msg.p.payload[3], msg.p.payload[4],
//			msg.p.payload[strlen(msg.p.payload) -4], msg.p.payload[strlen(msg.p.payload) - 3], 
//			msg.p.payload[strlen(msg.p.payload) -2], msg.p.payload[strlen(msg.p.payload) - 1]);
			//ESP_LOGI(TAG, "print count %d", count);
			break;
		case GPS_FIX:
			r_gps_location.altitude = msg.p.position.altitude;
			r_gps_location.latitude = msg.p.position.latitude;
			r_gps_location.longitude = msg.p.position.longitude;
			r_gps_location.fix = msg.p.position.fix;
			r_gps_location.fix_mode = msg.p.position.fix_mode;
#if 1			
			if(r_gps_location.fix_mode > GPS_MODE_INVALID)
				{
				ESP_LOGI(TAG, "Remote fix: %2d, %.10f, %.10f, %.2f", 
					r_gps_location.fix_mode, r_gps_location.latitude, r_gps_location.longitude, r_gps_location.altitude);
				}
#endif			
			if(c_gps_location.fix_mode > GPS_MODE_INVALID &&
				r_gps_location.fix_mode > GPS_MODE_INVALID)
				{
				
				}
			break;
		default:
			break;
		}
	}



