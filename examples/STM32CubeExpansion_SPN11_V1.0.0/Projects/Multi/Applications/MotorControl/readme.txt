/**
  @page IHM11M1 Expansion Board for STM32 Nucleo Boards Sample Application
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    readme.txt 
  * @author  IPC Group 
  * @version V0
  * @date    20-July-2016
  * @brief   Description of the IHM11M1 MotorControl application.
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This sample application shows how to use STSPIN230 to drive a motor.

To test this application you need a IHM11M1 expansion board plugged onto a STM32 Nucleo board. 
Please make sure to select the right configuration of the boards.

Motor activity is commanded differently, depending on workspace configuration:

  -  	Configuration mode waits for the blue button event to start the motor 
	and the black button to reset the firmware
  - 	comm mode enables communication via an external PC terminal
  -  	demo mode starts and stop the motor automatically

A motion command can be stopped at any moment by pushing the user button, via the
external PC terminal, or automatically. The firmware can be reset by pushing the reset
button on the STM32 Nucleo development board.
The speed profile is completely handled by the microcontroller depending on the potentiometer setting. 

In file “MC_SixStep_Param.h” you can easily modify the driver parameters  (acceleration,
deceleration, minimum and maximum speed, starting Duty Cycle, etc.). 


@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with IHM11M1 expansion board
  - This example has been tested with NUCLEO-F401RE board
    and can be easily tailored to many other supported device and development board.

@par How to use it? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - To best fit the features of the engine modify MC_SixStep_Param.h values.
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.40).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.21a).
   Alternatively you can use the System Workbench for STM32 (this firmware
   has been successfully tested with Version 1.3).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
