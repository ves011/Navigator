/*
 * project_specific.h
 *
 *  Created on: Dec 5, 2024
 *      Author: viorel_serbu
 */

#ifndef MAIN_PROJECT_SPECIFIC_H_
#define MAIN_PROJECT_SPECIFIC_H_

#define MDNS

#define ACTIVE_CONTROLLER			(NAVIGATOR)

#define TEST_BUILD (1)
#if(TEST_BUILD == 1)
	#define WITH_CONSOLE
	#define TEST1
	#define CTRL_DEV_ID					(99)
#else
	#define CTRL_DEV_ID					(1)
#endif

#define DEV_NAME			"navigator"

#define HOSTNAME			DEV_NAME
#define MDNSINSTANCE		DEV_NAME
#define MDNSINSTANCENAME	DEV_NAME
#define TCPCOMMPORT			10001


#define WIFI_STA_ON 					(1)
#define MQTT_PUBLISH					(0)

#define TCP_CLIENT_SERVER

typedef struct 
	{
	char payload[256];
	} nmea_msg_t;


#endif /* MAIN_PROJECT_SPECIFIC_H_ */
