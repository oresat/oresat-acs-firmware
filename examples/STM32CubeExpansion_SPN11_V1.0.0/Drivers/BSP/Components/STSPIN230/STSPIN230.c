/**
 ******************************************************************************
 * @file    STSPIN230.c
 * @author  IPC
 * @version V1.0.0
 * @date    28-April-2016
 * @brief   This file provides a set of functions to manage STSPIN230 driver
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

#include "STSPIN230.h"


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

/** @addtogroup STSPIN230    STSPIN230
  * @brief  STSPIN230 driver section
  * @{ 
  */

/** @defgroup STSPIN230MotorDriver    STSPIN230MotorDriver
  *  @{
    * @brief API pointer for STSPIN230
  */
/**
  * @brief  It handles all API functions for STSPIN230 MC Driver    
  * @retval None
*/

STSPIN230_MotorDriver_TypeDef STSPIN230MotorDriver =
{
  EnableInput_CH1_E_CH2_E_CH3_D,
  EnableInput_CH1_E_CH2_D_CH3_E,
  EnableInput_CH1_D_CH2_E_CH3_E,  
  DisableInput_CH1_D_CH2_D_CH3_D,
  Start_PWM_driving,
  Stop_PWM_driving,
  HF_TIMx_SetDutyCycle_CH1,
  HF_TIMx_SetDutyCycle_CH2,
  HF_TIMx_SetDutyCycle_CH3,
  Current_Reference_Start,
  Current_Reference_Stop,
  Current_Reference_Setvalue,
};    
    
 /**
  * @} 
  */   

/** @defgroup EnableInput_CH1_E_CH2_E_CH3_D    EnableInput_CH1_E_CH2_E_CH3_D
  *  @{
  * @brief  Enable Input channel CH1 and CH2 for STSPIN230     
  * @retval None
  */

void EnableInput_CH1_E_CH2_E_CH3_D()
{  
  STSPIN230_EnableInput_CH1_E_CH2_E_CH3_D();
}

/**
  * @} 
  */


/** @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
  * @brief  Enable Input channel CH1 and CH3 for STSPIN230           
  * @retval None
*/

void EnableInput_CH1_E_CH2_D_CH3_E()
{
  STSPIN230_EnableInput_CH1_E_CH2_D_CH3_E();
}

/**
  * @} 
  */

/** @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
  *  @{
  * @brief  Enable Input channel CH2 and CH3 for STSPIN230           
  * @retval None
*/

void EnableInput_CH1_D_CH2_E_CH3_E()
{
  STSPIN230_EnableInput_CH1_D_CH2_E_CH3_E();
}

/**
  * @} 
  */


/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
  * @brief  Enable Input channel CH2 and CH3 for STSPIN230           
  * @retval None
*/

void DisableInput_CH1_D_CH2_D_CH3_D()
{
  STSPIN230_DisableInput_CH1_D_CH2_D_CH3_D();
}

/**
  * @} 
  */

/** @defgroup Start_PWM_driving    Start_PWM_driving
  *  @{
  * @brief  Enable PWM channels for STSPIN230           
  * @retval None
*/

void Start_PWM_driving()
{
  STSPIN230_Start_PWM_driving();
} 

/**
  * @} 
  */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
  *  @{
  * @brief  Disable PWM channels for STSPIN230           
  * @retval None
*/

void Stop_PWM_driving()
{
  STSPIN230_Stop_PWM_driving();
}  

/**
  * @}
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
  *  @{
  * @brief  Set the Duty Cycle value for CH1           
  * @retval None
*/

void HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  STSPIN230_HF_TIMx_SetDutyCycle_CH1(CCR_value);
}


/**
  * @} 
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
  *  @{
  * @brief  Set the Duty Cycle value for CH2           
  * @retval None
*/

void HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  STSPIN230_HF_TIMx_SetDutyCycle_CH2(CCR_value);
}

/**
  * @} 
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
  *  @{
  * @brief  Set the Duty Cycle value for CH3           
  * @retval None
*/

void HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  STSPIN230_HF_TIMx_SetDutyCycle_CH3(CCR_value);
}

/**
  * @} 
  */




/** @defgroup Current_Reference_Start    Current_Reference_Start
  *  @{
  * @brief  Enable the Current Reference generation  
  * @retval None
*/

void Current_Reference_Start()
{
  STSPIN230_Current_Reference_Start();
}

/**
  * @} 
  */


/** @defgroup Current_Reference_Stop    Current_Reference_Stop
  *  @{
  * @brief  Disable the Current Reference generation
  * @retval None
*/

void Current_Reference_Stop()
{
 STSPIN230_Current_Reference_Stop();
}

/**
  * @}  
  */


/** @defgroup Current_Reference_Setvalue    Current_Reference_Setvalue
  *  @{
  * @brief  Set the value for Current Reference
  * @retval None
*/

void Current_Reference_Setvalue(uint16_t Iref)
{
  STSPIN230_Current_Reference_Setvalue(Iref);  
}

/**
  * @}  
  */






/**
  * @}  end STSPIN230 
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

