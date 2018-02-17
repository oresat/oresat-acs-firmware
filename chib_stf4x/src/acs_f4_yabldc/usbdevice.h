/*
 * usbcfg.h
 *
 *  Created on: Jul 10, 2013
 *      Author: kjell
 */

#ifndef USBCFG_H_
#define USBCFG_H_

#define USB_STACK_SIZE      1024

#define USB_PACKET_SIZE     64
#define USB_MEM_POOL_SIZE   10
#define USB_MAILBOX_SIZE    4

void startUsbControl(void);
void* usbAllocMailboxBuffer(void);
void usbFreeMailboxBuffer (void* buffer);
void usbGetMailboxes (Mailbox** RXMailbox, Mailbox** TXMailbox);

typedef struct {
  uint8_t   size;
  uint8_t   packet[USB_PACKET_SIZE];
} usbPacket;

#endif /* USBCFG_H_ */
