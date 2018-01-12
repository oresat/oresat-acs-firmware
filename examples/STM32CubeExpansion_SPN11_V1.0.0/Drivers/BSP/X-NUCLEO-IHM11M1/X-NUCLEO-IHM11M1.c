/**
 ******************************************************************************
 * @file    X-NUCLEO-IHM11M1.c
 * @author  IPC
 * @version V1.0.0
 * @date    10/07/2016
 * @brief   This file provides the set of functions to manage the X-Nucleo expansion board
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

/** @addtogroup DRIVERS     DRIVERS 
  * @brief  Driver Layer
  * @{ 
  */

/** @addtogroup BSP     BSP 
  * @brief  BSP Layer
  * @{ 
  */

/** @addtogroup X-NUCLEO-IHM11M1    X-NUCLEO-IHM11M1
  * @brief  X-Nucleo expansion board
  * @{ 
  */


/* Includes ------------------------------------------------------------------*/

#include "X-NUCLEO-IHM11M1.h"
#include "6Step_Lib.h"

#ifdef STM32F030x8
#include "stm32f0xx_hal.h"
#include "stm32F030_nucleo_ihm11m1.h"
#endif
#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#include "stm32F401_nucleo_ihm11m1.h"
#endif
extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/ 

/** @defgroup STSPIN230_EnableInput_CH1_E_CH2_E_CH3_D    STSPIN230_EnableInput_CH1_E_CH2_E_CH3_D
  *  @{
    * @brief Enable Input channel for STSPIN230
 */

/**
  * @brief  Enable Input channel CH1 and CH2 for STSPIN230     
  * @retval None
*/

void STSPIN230_EnableInput_CH1_E_CH2_E_CH3_D()
{
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 ENABLE
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
  * @} 
  */  
  
 /** @defgroup STSPIN230_EnableInput_CH1_E_CH2_D_CH3_E    STSPIN230_EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
    * @brief Enable Input channel for STSPIN230
  */
/**
  * @brief  Enable Input channel CH1 and CH3 for STSPIN230           
  * @retval None
*/

void  STSPIN230_EnableInput_CH1_E_CH2_D_CH3_E()
{
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 ENABLE 
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;    
}

/**
  * @} 
  */ 

/** @defgroup STSPIN230_EnableInput_CH1_D_CH2_E_CH3_E    STSPIN230_EnableInput_CH1_D_CH2_E_CH3_E
  *  @{
    * @brief Enable Input channel for STSPIN230
  */
/**
  * @brief  Enable Input channel CH2 and CH3 for STSPIN230           
  * @retval None
*/

void STSPIN230_EnableInput_CH1_D_CH2_E_CH3_E()
{
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Start(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
  HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
  * @} 
  */

/** @defgroup STSPIN230_DisableInput_CH1_D_CH2_D_CH3_D    STSPIN230_DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
    * @brief Disable All Input channels for STSPIN230
  */
/**
  * @brief  Enable Input channel CH2 and CH3 for STSPIN230           
  * @retval None
*/

void STSPIN230_DisableInput_CH1_D_CH2_D_CH3_D()
{
    HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH1);           //TIM1_CH1 DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;
  
  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH2);           //TIM1_CH2  DISABLE 
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;

  HAL_TIM_PWM_Stop(&HF_TIMx,HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
  HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  
}

/**
  * @} 
  */

/** @defgroup STSPIN230_Start_PWM_driving    STSPIN230_Start_PWM_driving
  *  @{
    * @brief Enable the PWM generation on Input channels for STSPIN230
  */
/**
  * @brief  Enable PWM channels for STSPIN230           
  * @retval None
*/

void STSPIN230_Start_PWM_driving()
{
    HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 ENABLE   
    HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH1) ;  

  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 ENABLE   
    HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH2) ;  

  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
    HAL_TIMEx_PWMN_Start(&HF_TIMx,HF_TIMx_CH3) ;  
} 

/**
  * @} 
  */

/** @defgroup STSPIN230_Stop_PWM_driving    STSPIN230_Stop_PWM_driving
  *  @{
    * @brief Disable the PWM generation on Input channels for STSPIN230
  */
/**
  * @brief  Disable PWM channels for STSPIN230           
  * @retval None
*/

void STSPIN230_Stop_PWM_driving()
{
    HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 DISABLE   
    HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH1) ;  

  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 DISABLE   
    HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH2) ;  

  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
    HAL_TIMEx_PWMN_Stop(&HF_TIMx,HF_TIMx_CH3) ;  

}  

/**
  * @}
  */

/** @defgroup STSPIN230_HF_TIMx_SetDutyCycle_CH1    STSPIN230_HF_TIMx_SetDutyCycle_CH1
  *  @{
    * @brief Set the Duty Cycle value for CH1 for STSPIN230
*/
/**
  * @brief  Set the Duty Cycle value for CH1           
  * @retval None
*/

void STSPIN230_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
    HF_TIMx.Instance->HF_TIMx_CCR1 = CCR_value;  

}


/**
  * @} 
  */

/** @defgroup STSPIN230_HF_TIMx_SetDutyCycle_CH2    STSPIN230_HF_TIMx_SetDutyCycle_CH2
  *  @{
    * @brief Set the Duty Cycle value for CH2 for STSPIN230
*/
/**
  * @brief  Set the Duty Cycle value for CH2           
  * @retval None
*/

void STSPIN230_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
    HF_TIMx.Instance->HF_TIMx_CCR2 = CCR_value;  

}
/**
  * @} 
  */






/** @defgroup STSPIN230_HF_TIMx_SetDutyCycle_CH3    STSPIN230_HF_TIMx_SetDutyCycle_CH3
  *  @{
    * @brief Set the Duty Cycle value for CH3 for STSPIN230
*/
/**
  * @brief  Set the Duty Cycle value for CH3           
  * @retval None
*/


void STSPIN230_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
    HF_TIMx.Instance->HF_TIMx_CCR3 = CCR_value;  

}

/**
  * @} 
  */

/** @defgroup STSPIN230_Current_Reference_Start    STSPIN230_Current_Reference_Start
  *  @{
    * @brief Start PWM and set the STARTUP_DUTY_CYCLE for STSPIN230
*/
/**
  * @brief  Start PWM and set the STARTUP_DUTY_CYCLE  
  * @retval None
*/

void STSPIN230_Current_Reference_Start()
{
  Start_PWM_driving();
  SIXSTEP_parameters.pulse_value=STARTUP_DUTY_CYCLE;
}

/**
  * @} 
  */


/** @defgroup STSPIN230_Current_Reference_Stop    STSPIN230_Current_Reference_Stop
  *  @{
    * @brief Stop PWM for STSPIN230
*/
/**
  * @brief  Stop PWM
  * @retval None
*/

void STSPIN230_Current_Reference_Stop()
{
  Stop_PWM_driving();
  SIXSTEP_parameters.pulse_value=STARTUP_DUTY_CYCLE;

}
/**
  * @}  
  */


/** @defgroup STSPIN230_Current_Reference_Setvalue    STSPIN230_Current_Reference_Setvalue
  *  @{
    * @brief Set the value for Duty Cucle for STSPIN230
*/
/**
  * @brief  Set the value for Duty Cucle
  * @retval None
*/


void STSPIN230_Current_Reference_Setvalue(uint16_t Iref)
{
      SIXSTEP_parameters.pulse_value=Iref;

}

/**
  * @}  
  */  
  

/** @defgroup BSP_X_NUCLEO_FAULT_LED_ON    BSP_X_NUCLEO_FAULT_LED_ON
  *  @{
    * @brief Turns selected LED On.
  * @retval None
*/

void BSP_X_NUCLEO_FAULT_LED_ON()
{
}

/**
  * @}  
  */  
  
/** @defgroup BSP_X_NUCLEO_FAULT_LED_OFF    BSP_X_NUCLEO_FAULT_LED_OFF
  *  @{
    * @brief Turns selected LED Off.
  * @retval None
*/
void BSP_X_NUCLEO_FAULT_LED_OFF()
{
}
/**
  * @}  
  */
/**
  * @}  end X-NUCLEO-IHM11M1 
  */

/**
  * @}  end BSP
  */

/**
  * @}  end DRIVERS
  */
