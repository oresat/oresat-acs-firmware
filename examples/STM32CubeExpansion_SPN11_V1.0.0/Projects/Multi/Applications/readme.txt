-----------
readme.txt
-----------
In this folder there is an application for Motor Control tested on the NUCLEO-F401RE board.

  - MotorControl: 	In this application, a low voltage three-phase brushless motor is driven 
			by the STSPIN230 device. The 6-step algorithm is based on the 1-shunt 
			current sensing mode and the sensorless algorithm for BEMF detection. 
			A STM32 Nucleo expansion board (X-NUCLEO-IHM11M1) is plugged onto the 
			STM32 Nucleo board. The speed profile is completely handled by the 
			microcontroller depending on the potentiometer setting. Motor activity 
			is commanded differently, depending on workspace configuration
  

Please, read the respective readme.txt file within the application folders for details 
and usage instructions.

IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is suggested to
update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version. It can be done by downloading the
latest version of the STM32 ST-LINK utility and then use the ST-LINK -> Firmware update feature.	

