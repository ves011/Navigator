/*
 * hmc5833.h
 *
 *  Created on: Nov 30, 2024
 *      Author: viorel_serbu
 */

#ifndef MAIN_HMC5883_H_
#define MAIN_HMC5883_H_


#define HMC_I2C_ADDRESS		0x1e

void register_hmc();
int do_hmc(int argc, char **argv);


#endif /* MAIN_HMC5883_H_ */
