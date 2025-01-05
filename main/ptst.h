/*
 * ptst.h
 *
 *  Created on: Jan 4, 2025
 *      Author: viorel_serbu
 */

#ifndef MAIN_PTST_H_
#define MAIN_PTST_H_

#define ENLEFTM				(40)
#define ENRIGHTM			(35)
#define MOTEN_INTR			(41)
#define LEFTM_CTRL1			(39)
#define LEFTM_CTRL2			(38)
#define RIGHTM_CTRL1		(37)
#define RIGHTM_CTRL2		(36)

#define SENSE_ML			(0)			// IO1
#define SENSE_MR			(1)			// IO2
#define SENSE_BAT			(3)			// IO4

#define PT_ENABLED			100			//message source when enL or enR goues high
										// sent by moten interrupt 

void register_ptst();

#endif /* MAIN_PTST_H_ */
