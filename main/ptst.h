/*
 * ptst.h
 *
 *  Created on: Jan 4, 2025
 *      Author: viorel_serbu
 */

#ifndef MAIN_PTST_H_
#define MAIN_PTST_H_

//decoder GPIOs
#define DEC_LEFT_A			(6)
#define DEC_LEFT_B			(5)
#define DEC_RIGHT_A			(41)
#define DEC_RIGHT_B			(42)
//Motors control GPIOs
#define ENLEFTM				(40)
#define ENRIGHTM			(35)
//#define MOTEN_INTR			(41)
#define LEFTM_CTRL1			(38)
#define LEFTM_CTRL2			(39)
#define RIGHTM_CTRL1		(36)
#define RIGHTM_CTRL2		(37)

#define SENSE_ML			(0)			// IO1
#define SENSE_MR			(1)			// IO2
#define SENSE_BAT			(3)			// IO4

#define PT_ENABLED			100			//message source when enL or enR goues high
#define DEC_SRC				101			// decoder left / right B message interrupt
 
#define PTST_FILE					"ptst.txt"



void register_ptst();
int do_ptst(int argc, char **argv);

#endif /* MAIN_PTST_H_ */
