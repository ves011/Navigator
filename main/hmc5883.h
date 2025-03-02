/*
 * hmc5833.h
 *
 *  Created on: Nov 30, 2024
 *      Author: viorel_serbu
 */

#ifndef MAIN_HMC5883_H_
#define MAIN_HMC5883_H_


#define HMC_I2C_ADDRESS		0x1e
#define HMC_CONFIGA			0
#define HMC_CONFIGB			1
#define HMC_MODE			2
#define HMC_OUTPUTX			3
#define HMC_OUTPUTZ			5
#define HMC_OUTPUTY			7
#define HMC_STATUS			9
#define HMC_ID				10

#define HMC_DATA			1
#define HMC_CMD				0x80
#define HMC_CMD_DISP_VAL	0x81



void register_hmc();
int do_hmc(int argc, char **argv);


#endif /* MAIN_HMC5883_H_ */
