/**
  @page  Description of the example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    readme.txt 
  * @author  System Lab Team
  * @version V1.0.0
  * @date    06-July-2015
  * @brief   Workspace description
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

This directory provides a reference workspace project that can be used to build any firmware application for
STM32F302R8 devices using STM32CubeF3 HAL and running on STM32F302R8-Nucleo board from STMicroelectronics. 
The workspace contains the middleware layer with Motor Control library to run the motor connected on X-Nucleo board with 6-step control algorithm and enables the potentiometer to
regulate the motor speed. The 6-step algorithm is based on 1shunt current sensing mode and sensorless algorithm for bEmf detection.

The workspace is provided for STM32F3xx-Nucleo (STM32F302R8 devices) in four different configurations, normal, demo, comm mode, boot mode.
The "normal" mode waits the blue button event to start the motor, the "demo" mode starts and stop the motor automatically, the "comm" mode 
enables the communication protocol with external PC terminal and the "boot" mode enables the FW for boot loader call.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@note The clock setting is configured to have the max product performance (max clock frequency) 
      so not optimized in term of power consumption.
      
@par User directory contents 

  - ../Src/main_F302.c            Main program
  - ../Src/system_stm32f3xx.c     STM32F3xx system clock configuration file
  - ../Src/stm32f3xx_it.c         Interrupt handlers 
  - ../Src/stm32f3xx_hal_msp.c    HAL MSP module
  - ../Inc/main.h                 Main program header file  
  - ../Inc/stm32f3xx_hal_conf.h   HAL Configuration file
  - ../Inc/stm32f3xx_it.h         Interrupt handlers header file
  - ../Inc/MC_SixStep_param.h     All motor parameters file

@par BSP directory contents 

  - /BSP/Component/l6230.c   		      			      main file for L6230 driver
  - /BSP/STM32F3xx-Nucleo/STM32F3xx-Nucleo.c   		      STM32F3xx nucleo board file
  - /BSP/X-NUCLEO-IHM07M1/X-NUCLEO-IHM07M1.c         	  X-NUCLEO-IHM07M1 nucleo board file
  
@par Middleware directory contents 

  - /MC_6STEP_LIB/6Step_Lib.c  		      			      FW 6-STEP library main file
  - /MC_6STEP_LIB/stm32F302_nucleo_ihm07m1.c		      FW 6-STEP library interface file
  - /UART_SERIAL_COM/UART_UI.c          		          UART communication file  
  
@par Hardware and Software environment  

  - This example runs on STM32F302R8 devices.
    
  - This example has been tested with STMicroelectronics STM32F302R8-Nucleo 
    boards and can be easily tailored to any other supported device 
    and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open the toolchain KEIL IDE
 - Select the configuration for workspace (normal, demo, comm or boot)
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
