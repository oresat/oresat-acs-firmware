/**
 ******************************************************************************
 * @file    MC_Common.h
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This header file is a common file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_COMMON_H
#define __MC_COMMON_H

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP    BSP
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup COMPONENTS    COMPONENTS
  * @brief  Components
  * @{ 
  */

/** @addtogroup L6230_Motor_Driver_handler     L6230_Motor_Driver_handler  
  * @brief  Handler for L6230 Motor driver
  * @{ 
  */  

  typedef struct
  {
    void (*EnableInput_CH1_E_CH2_E_CH3_D)(void);  /*!< Enable the channel 1,2 and Disable the channel 3 */
    void (*EnableInput_CH1_E_CH2_D_CH3_E)(void);  /*!< Enable the channel 1,3 and Disable the channel 2 */
    void (*EnableInput_CH1_D_CH2_E_CH3_E)(void);  /*!< Enable the channel 2,3 and Disable the channel 1 */
    void (*DisableInput_CH1_D_CH2_D_CH3_D)(void); /*!< Disable all channels */
    void (*Start_PWM_driving)(void);              /*!< Start PWM generation */
    void (*Stop_PWM_driving)(void);               /*!< Stop PWM generation */
    void (*HF_TIMx_SetDutyCycle_CH1)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH1 */    
    void (*HF_TIMx_SetDutyCycle_CH2)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH2 */
    void (*HF_TIMx_SetDutyCycle_CH3)(uint16_t);   /*!< High Frequency Timer - Change DutyCycle value for CH3 */
    void (*Current_Reference_Start)(void);        /*!< Start current reference generation for closed loop control */
    void (*Current_Reference_Stop)(void);         /*!< Stop current reference generation for closed loop control */
    void (*Current_Reference_Setvalue)(uint16_t); /*!< Set current reference value for closed loop control */    
  } L6230_MotorDriver_TypeDef;                          /*!< MC driver handler */

/**
  * @}  end L6230_Motor_Driver_handler 
  */

/**
  * @}  end COMPONENTS 
  */

/**
  * @}  end BSP 
  */

/**
  * @}  end DRIVERS
  */

#endif

