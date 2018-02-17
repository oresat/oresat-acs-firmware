/*
 * usbcfg.c
 *
 *  Created on: Jul 10, 2013
 *      Author: kjell
 */

#include "ch.h"
#include "hal.h"

#include "usb.h"
#include "usbdevice.h"
#include "usbdescriptor.h"
#include "sysctrl.h"

#define IN_MULT 4

USBDriver *  	usbp = &USBD1;

static Mailbox usbTXMailbox;
static Mailbox usbRXMailbox;

static usbPacket usbMemPoolBuffer[USB_MEM_POOL_SIZE];
static MemoryPool usbMemPool;

static uint8_t* usbTXMailboxBuffer[USB_MAILBOX_SIZE];
static uint8_t* usbRXMailboxBuffer[USB_MAILBOX_SIZE];

EVENTSOURCE_DECL(esUsbConfigured);
EVENTSOURCE_DECL(esUsbReset);

EVENTSOURCE_DECL(esUsbTxComplete);
EVENTSOURCE_DECL(esUsbRxComplete);

/*
 * data Transmitted Callback
 * Does nothing but signal the TX thread that the current transmit is completed
 */
void dataTransmitted(USBDriver *usbp, usbep_t ep){
  (void) usbp;
  (void) ep;

  //Signals that the current TX is finished
  chSysLockFromIsr();
  chEvtBroadcastFlagsI(&esUsbTxComplete, (flagsmask_t)0);
  chSysUnlockFromIsr();
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
                                            USB_EP_MODE_TYPE_BULK,    //Type and mode of the endpoint
                                            NULL,                     //Setup packet notification callback (NULL for non-control EPs)
                                            dataTransmitted,          //IN endpoint notification callback
                                            NULL,                     //OUT endpoint notification callback
                                            USB_PACKETSIZE,            //IN endpoint maximum packet size
                                            0x0000,                   //OUT endpoint maximum packet size
                                            &ep1instate,              //USBEndpointState associated to the IN endpoint
                                            NULL,                     //USBEndpointState associated to the OUTendpoint
                                            IN_MULT,
                                            NULL                      //Pointer to a buffer for setup packets (NULL for non-control EPs)
};



/*
 * data Received Callback
 * Does nothing but signal the RX thread that the current receive is completed
 */
void dataReceived(USBDriver *usbp, usbep_t ep){
  (void) usbp;
  (void) ep;

  //Signals that the current RX is finished
  chSysLockFromIsr();
  chEvtBroadcastFlagsI(&esUsbRxComplete, (flagsmask_t)0);
  chSysUnlockFromIsr();
}


/**
 * @brief   OUT EP2 state.
 */
USBOutEndpointState ep2outstate;

/**
 * @brief   EP2 initialization structure (OUT only).
 */
static const USBEndpointConfig ep2config = {
                                            USB_EP_MODE_TYPE_BULK,
                                            NULL,
                                            NULL,
                                            dataReceived,
                                            0x0000,
                                            USB_PACKETSIZE,
                                            NULL,
                                            &ep2outstate,
                                            1,
                                            NULL
};


/*
 * Handles the USB driver global events.
 * These events are triggered by the USB driver.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  (void) usbp;
  switch (event) {
  case USB_EVENT_RESET:
    /* Signals when a USB reset event has occurred.
     * Typically this will happen at startup and when the cable
     * is connected or disconnected.
     * After a reset, the usb system will not be operational
     * until it is configured.
     */
    chSysLockFromIsr();
    chEvtBroadcastFlagsI(&esUsbReset, (flagsmask_t)0);
    chSysUnlockFromIsr();
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:

    /* Enables the endpoints specified into the configuration.
     * Note, this callback is invoked from an ISR so I-Class functions
     * must be used.
     */
    chSysLockFromIsr();
    usbInitEndpointI(usbp, 1, &ep1config);
    usbInitEndpointI(usbp, 2, &ep2config);

    /* Signals that the configuration is complete, and the USB system
     * is ready to be used.
     */
    chEvtBroadcastFlagsI(&esUsbConfigured, (flagsmask_t)0);
    chSysUnlockFromIsr();

    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Returns the USB descriptors defined in usbdescriptor.h
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;

  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4){
      return &vcom_strings[dindex];
    }
    else{
      return &vcom_strings[4];
    }
  }
  return NULL;
}

/*
 * Requests hook callback.
 * This hook allows to be notified of standard requests or to
 * handle non standard requests.
 * This implementation does nothing and passes all requests to
 * the upper layers
 */
bool_t requestsHook(USBDriver *usbp) {
  (void) usbp;
  return FALSE;
}

/*
 * USBconfig
 */
const USBConfig   	config =   {
                  	            usb_event,
                  	            get_descriptor,
                  	            requestsHook,
                  	            NULL
};


/* TX management thread.
 * This thread will fetch USB packages passed through the Out Mailbox
 * and pass it on to the USB driver for transmission.
 */
static WORKING_AREA(waUsbTx, USB_STACK_SIZE);
static msg_t tUsbTx(void *arg) {
  (void)arg;
  chRegSetThreadName("usbTx");


  msg_t msg;
  usbPacket *usbBufp;

  enum {UsbTxComleteID = 0, UsbResetID = 1, UsbConfiguredID = 2};

  EventListener elUsbTxComplete;
  EventListener elUsbReset;
  EventListener elUsbConfigured;

  eventmask_t   activeEvents;


  chEvtRegister(&esUsbTxComplete, &elUsbTxComplete, UsbTxComleteID);
  chEvtRegister(&esUsbReset, &elUsbReset, UsbResetID);
  chEvtRegister(&esUsbConfigured, &elUsbConfigured, UsbConfiguredID);

  // Wait for the USB system to be configured. and clear all other event flags.
  chEvtWaitOne(EVENT_MASK(UsbConfiguredID));
  chEvtGetAndClearEvents(EVENT_MASK(UsbTxComleteID) | EVENT_MASK(UsbResetID));

  while (TRUE) {
    chMBFetch (&usbTXMailbox, &msg, TIME_INFINITE);

    // Check if USB has been reconfigured while waiting for message from sysctrl
    activeEvents = chEvtGetAndClearEvents(EVENT_MASK(UsbConfiguredID));
    if (activeEvents == EVENT_MASK(UsbConfiguredID)) {
      // If so, clear the reset event since it is no longer relevant.
      activeEvents = chEvtGetAndClearEvents(EVENT_MASK(UsbResetID));
    }

    // Typecast Mailbox message to command package pointer for readability
    usbBufp = (usbPacket*)msg;

    // Prepare transmit and start the transmission. This operation will return immediately
    usbPrepareTransmit(usbp, EP_IN, usbBufp->packet, (size_t)usbBufp->size);
    chSysLock();
    usbStartTransmitI(usbp, EP_IN);
    chSysUnlock();

    //Check for events from the USB system.
    activeEvents = chEvtWaitAny(EVENT_MASK(UsbTxComleteID) | EVENT_MASK(UsbResetID));

    if (activeEvents == EVENT_MASK(UsbResetID)) {
      chEvtWaitOne(EVENT_MASK(UsbConfiguredID));
      // Clear any events that has occurred while the usb was not configured.
      chEvtGetAndClearEvents(EVENT_MASK(UsbTxComleteID) | EVENT_MASK(UsbResetID));
    }
    usbFreeMailboxBuffer (usbBufp);
  }
  return 0;
}

/* RX management thread.
 * This thread will initiate listening on the USB.
 * When a package is received, it will be passed
 * through IN mailbox to the sysctrl
 */
static WORKING_AREA(waUsbRx, USB_STACK_SIZE);
static msg_t tUsbRx(void *arg) {
  (void)arg;
  chRegSetThreadName("usbRx");

  enum {UsbRxComleteID = 0, UsbResetID = 1, UsbConfiguredID = 2};

  usbPacket *usbBufp;

  EventListener elUsbRxComplete;
  EventListener elUsbReset;
  EventListener elUsbConfigured;

  eventmask_t   activeEvents;


  chEvtRegister(&esUsbRxComplete, &elUsbRxComplete, UsbRxComleteID);
  chEvtRegister(&esUsbReset, &elUsbReset, UsbResetID);
  chEvtRegister(&esUsbConfigured, &elUsbConfigured, UsbConfiguredID);

  // Wait for the USB system to be configured.
  chEvtWaitOne(EVENT_MASK(UsbConfiguredID));
  chEvtGetAndClearEvents(EVENT_MASK(UsbRxComleteID) | EVENT_MASK(UsbResetID));

  while (TRUE) {

    // Allocate buffer space for reception of package in the sysctrl mempool
    usbBufp = usbAllocMailboxBuffer();

    // Prepare receive operation and initiate the usb system to listen
    usbPrepareReceive(usbp, EP_OUT, usbBufp->packet, 64);
    chSysLock();
    usbStartReceiveI(usbp, EP_OUT);
    chSysUnlock();

    // Wait for events from the USB system
    activeEvents = chEvtWaitAny(EVENT_MASK(UsbRxComleteID) | EVENT_MASK(UsbResetID));

    if (activeEvents == EVENT_MASK(UsbResetID)) {
      // If the system was reset, clean up and wait for new configure.
      usbFreeMailboxBuffer (usbBufp);
      chEvtWaitOne(EVENT_MASK(UsbConfiguredID));
      chEvtGetAndClearEvents(EVENT_MASK(UsbRxComleteID) | EVENT_MASK(UsbResetID));
    }
    else {
      // Post pagckage to sysctrl if receive was successful
      usbBufp->size = ep2outstate.rxcnt;
      chMBPost (&usbRXMailbox, (msg_t)usbBufp, TIME_INFINITE);
    }
  }

  return 0;
}

/*
 * Start all usb related threads and initiate the USB subsytem.
 */
void startUsbControl(void) {
  chMBInit (&usbTXMailbox, (msg_t *)usbTXMailboxBuffer, USB_MAILBOX_SIZE);
  chMBInit (&usbRXMailbox, (msg_t *)usbRXMailboxBuffer, USB_MAILBOX_SIZE);

  chPoolInit (&usbMemPool, USB_PACKET_SIZE, NULL);
  chPoolLoadArray(&usbMemPool, &usbMemPoolBuffer, USB_MEM_POOL_SIZE);

  chThdCreateStatic(waUsbTx, sizeof(waUsbTx), NORMALPRIO, tUsbTx, NULL);
  chThdCreateStatic(waUsbRx, sizeof(waUsbRx), NORMALPRIO, tUsbRx, NULL);

  //Start and Connect USB
  usbStart(usbp, &config);
  usbConnectBus(usbp);
}

void* usbAllocMailboxBuffer(void){
  return chPoolAlloc(&usbMemPool);
}

void usbFreeMailboxBuffer (void* buffer) {
  chPoolFree (&usbMemPool, buffer);
}

void usbGetMailboxes (Mailbox** RXMailbox, Mailbox** TXMailbox) {
  *RXMailbox = &usbRXMailbox;
  *TXMailbox = &usbTXMailbox;
}

