/*
 * navi_mon.h
 *
 *  Created on: Jan 19, 2025
 *      Author: viorel_serbu
 */

#ifndef MAIN_NAVI_MON_H_
#define MAIN_NAVI_MON_H_

#define MON_TIMER_INTERVAL		100000 	//100 msec
#define BAT_MON_INTERVAL		10		// check battery status every BAT_MON_INTERVAL * MON_TIMER_INTERVAL 
#define BAT_FLASHING_MED		10
#define BAT_FLASHING_LOW		2			

// the resistor divider is 3/1.8k
// bat ok > 7.6V --> (7.6 / 4.8) * 1.8 =  2.85V
// bat medium 7 - 7.6V --> 2.625 - 2.85
// bat low < 7V	--> < 2.625
#define BAT_OK 		2850
#define BAT_LOW		2625
		

void navi_mon_task(void *pvParameters);



#endif /* MAIN_NAVI_MON_H_ */
