/**
 ******************************************************************************
 * @file    l6230.c
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This file provides a set of functions to manage L6230 driver
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

#include "l6230.h"

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

/** @addtogroup L6230    L6230
  * @brief  L6230 driver section
  * @{ 
  */

/** @defgroup L6230MotorDriver    L6230MotorDriver
  *  @{
    * @brief API pointer for L6230
  */


L6230_MotorDriver_TypeDef L6230MotorDriver =
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
  * @brief  Enable Input channel CH1 and CH2 for L6230     
  * @retval None
  */

void EnableInput_CH1_E_CH2_E_CH3_D()
{
  L6230_ECH1CH2_DCH3_IO_Write();
}

/**
  * @} 
  */


/** @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
  * @brief  Enable Input channel CH1 and CH3 for L6230           
  * @retval None
*/

void EnableInput_CH1_E_CH2_D_CH3_E()
{
  L6230_ECH1CH3_DCH2_IO_Write();
}

/**
  * @} 
  */

/** @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
  *  @{
  * @brief  Enable Input channel CH2 and CH3 for L6230           
  * @retval None
*/

void EnableInput_CH1_D_CH2_E_CH3_E()
{
  L6230_ECH2CH3_DCH1_IO_Write();
}

/**
  * @} 
  */


/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
  * @brief  Enable Input channel CH2 and CH3 for L6230           
  * @retval None
*/

void DisableInput_CH1_D_CH2_D_CH3_D()
{
  L6230_DCH1CH2CH3_IO_Write();
}

/**
  * @} 
  */

/** @defgroup Start_PWM_driving    Start_PWM_driving
  *  @{
  * @brief  Enable PWM channels for L6230           
  * @retval None
*/

void Start_PWM_driving()
{
  L6230_Start_PWM_generation(); 
} 

/**
  * @} 
  */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
  *  @{
  * @brief  Disable PWM channels for L6230           
  * @retval None
*/

void Stop_PWM_driving()
{
  L6230_Stop_PWM_generation(); 
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
  L6230_HFTIM_DC_CH1(CCR_value);
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
  L6230_HFTIM_DC_CH2(CCR_value);
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
  L6230_HFTIM_DC_CH3(CCR_value);
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
  START_Ref_Generation();
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
  STOP_Ref_Generation();
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
  Set_Ref_Generation(Iref);
}

/**
  * @}  
  */

/**
  * @}  end L6230 
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

