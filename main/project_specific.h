/*
 * project_specific.h
 *
 *  Created on: Dec 5, 2024
 *      Author: viorel_serbu
 */

#ifndef MAIN_PROJECT_SPECIFIC_H_
#define MAIN_PROJECT_SPECIFIC_H_

#define MDNS
#define USE_I2C

#define ACTIVE_CONTROLLER			(NAVIGATOR)

#define TEST_BUILD (1)
#if(TEST_BUILD == 1)
	#define WITH_CONSOLE
	#define TEST1
	#define CTRL_DEV_ID					(99)
	#define LOG_SERVER_DEV				"proxy.gnet"
	#define LOG_PORT_DEV				5556
#else
	#define CTRL_DEV_ID					(1)
#endif

#define DEV_NAME			"navigator"

#define HOSTNAME			DEV_NAME
#define MDNSINSTANCE		DEV_NAME
#define MDNSINSTANCENAME	DEV_NAME
#define TCPCOMMPORT			10001

#define BAT_ADC_CHANNEL		3
#define ADC_CHN_0			0
#define ADC_CHN_1			1
#define ADC_CHN_2			3


#define WIFI_STA_ON 		(1)
#define MQTT_PUBLISH		(0)

#define TCP_CLIENT_SERVER

/*
Message definitions for device monitor queue
*/
#define MSG_WIFI			1	// wifi connect (.val = 1)/disconnect (.val = 0) event 
#define MSG_BAT				2	// battery level .val = ADC battery measurement * 1000
#define MSG_LED_FLASH		3	// nw state and remote state flashing
#define NW_STATE_CHANGE		4	// nw connected (.val = 1) / disconnected (.val = 0)
#define REMOTE_STATE_CHANGE	5	// remote connected (.val = 1) / disconnected (.val = 0)
#define INIT_COMPLETE		6	// init completed
#define SENSOR_DATA			7	// data form sensors
#define MPU_SENSOR_DATA		8	// data from MPU
#define HMC_SENSOR_DATA		9	// data from HMC
#define MON_SHOW_DATA		10  // 

typedef struct 
	{
	char payload[256];
	} nmea_msg_t;


#endif /* MAIN_PROJECT_SPECIFIC_H_ */
