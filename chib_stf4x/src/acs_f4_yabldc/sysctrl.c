/*
 * sysctrl.c
 *
 *  Created on: Jul 22, 2013
 *      Author: kjell
 */

#include "ch.h"
#include "hal.h"

#include "sysctrl.h"
#include "commands.h"

//#include "usbdevice.h"

#include "bldc.h"

//static Mailbox* cmdTXMailbox;
//static Mailbox* cmdRXMailbox;

static THD_WORKING_AREA(waSysCtrlCmdParser, SYS_CTRL_STACK_SIZE);
static THD_FUNCTION(tSysCtrlCmdParser, arg){
//static msg_t tSysCtrlCmdParser(void *arg) {
  (void)arg;
  chRegSetThreadName("SysCtrlCmdParser");

/*
  msg_t msg;
  usbPacket* usbBufp;
  cmdPkg* cmdBufp;
  uint32_t dc;

  while (TRUE) {
    chMBFetch (cmdRXMailbox, &msg, TIME_INFINITE);

    usbBufp=(usbPacket*)msg;
    cmdBufp=(cmdPkg*)usbBufp->packet;

    switch (cmdBufp->cmd) {
    case CMD_BLDC1_INIT:
      bldcInit();
      break;
    case CMD_BLDC1_KILL:
      bldcKill();
      break;
    case CMD_BLDC1_START:
      bldcStart();
      break;
    case CMD_BLDC1_DIRECTION:
      break;
    case CMD_BLDC1_DUTYCYCLE:
      dc = (uint32_t)(((cmdBufp->payload[0])<<24) +
          ((cmdBufp->payload[1])<<16) +
          ((cmdBufp->payload[2])<<8) +
          cmdBufp->payload[3]);
      bldcSetDutyCycle (dc);
      break;
    case CMD_BLDC1_RPM:
      bldcSetRPM ((uint32_t)((cmdBufp->payload[0]<<8) + (cmdBufp->payload[1])));
      break;
    case CMD_LED1_ON:
      palSetPad(GPIOD, GPIOD_LED3);
      break;
    case CMD_LED1_OFF:
      palClearPad(GPIOD, GPIOD_LED3);
      break;
    case CMD_LED2_ON:
      palSetPad(GPIOD, GPIOD_LED4);
      break;
    case CMD_LED2_OFF:
      palClearPad(GPIOD, GPIOD_LED4);
      break;
    case CMD_LED3_ON:
      palSetPad(GPIOD, GPIOD_LED5);
      break;
    case CMD_LED3_OFF:
      palClearPad(GPIOD, GPIOD_LED5);
      break;
    case CMD_LED4_ON:
      palSetPad(GPIOD, GPIOD_LED6);
      break;
    case CMD_LED4_OFF:
      palClearPad(GPIOD, GPIOD_LED6);
      break;
    default:
      break;
    }

    usbFreeMailboxBuffer (usbBufp);

    usbBufp = usbAllocMailboxBuffer();
    cmdBufp=(cmdPkg*)usbBufp->packet;
    cmdBufp->cmd = CMD_ACK;
    cmdBufp->pkgSize = 2;
    usbBufp->size = 2;

    chMBPost (cmdTXMailbox, (msg_t)usbBufp, TIME_INFINITE);

  }
	//*/
	

  bldcStart();
			
//  return 0;
}


void startSysCtrl(void) {
  chThdCreateStatic(waSysCtrlCmdParser,
                    sizeof(waSysCtrlCmdParser),
                    NORMALPRIO,
                    tSysCtrlCmdParser,
                    NULL);


  /* Start system peripherals such as communication modules and other
   * modules not directly related to the BLDC
   */

 // startUsbControl();
 // usbGetMailboxes (&cmdRXMailbox, &cmdTXMailbox);
}



