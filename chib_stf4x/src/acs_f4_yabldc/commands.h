/*
 * commands.h
 *
 *  Created on: Jul 27, 2013
 *      Author: kjell
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#define		CMD_GET_STATUS		    0x00
#define     CMD_ACK                 0x0E
#define     CMD_NACK                0x0F

#define		CMD_BLDC1_INIT		    0x80
#define		CMD_BLDC1_KILL		    0x81
#define     CMD_BLDC1_START         0x82
#define     CMD_BLDC1_STOP          0x83
#define     CMD_BLDC1_DIRECTION     0x84
#define		CMD_BLDC1_DUTYCYCLE	    0x85
#define		CMD_BLDC1_RPM           0x86

#define     CMD_LED1_ON             0xF0
#define     CMD_LED1_OFF            0xF1
#define     CMD_LED2_ON             0xF2
#define     CMD_LED2_OFF            0xF3
#define     CMD_LED3_ON             0xF4
#define     CMD_LED3_OFF            0xF5
#define     CMD_LED4_ON             0xF6
#define     CMD_LED4_OFF            0xF7



#endif /* COMMANDS_H_ */
