/**
 ******************************************************************************
 * @file    X-NUCLEO-IHM07M1.c
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This file provides the set of functions to manage the X-Nucleo board
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

/** @addtogroup X-NUCLEO-IHM07M1    X-NUCLEO-IHM07M1
  * @brief  X-Nucleo board
  * @{ 
  */


/* Includes ------------------------------------------------------------------*/
#include "X-NUCLEO-IHM07M1.h"
#ifdef STM32F030x8
#include "stm32f0xx_hal.h"
#include "stm32F030_nucleo_ihm07m1.h"
#endif
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#include "stm32F103_nucleo_ihm07m1.h"
#endif
#ifdef STM32F302x8
#include "stm32f3xx_hal.h"
#include "stm32F302_nucleo_ihm07m1.h"
#endif
#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#include "stm32F401_nucleo_ihm07m1.h"
#endif
        

/** @defgroup L6230_ECH1CH2_DCH3_IO_Write    L6230_ECH1CH2_DCH3_IO_Write
  *  @{
    * @brief Enable Input channel CH1 and CH2 for L6230   
  * @retval None
*/

void L6230_ECH1CH2_DCH3_IO_Write()
{
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH1,GPIO_SET);      //EN1 ENABLE               
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH2,GPIO_SET);      //EN2 DISABLE
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH3,GPIO_RESET);    //EN3 ENABLE    
}
/**
  * @}  
  */

/** @defgroup L6230_ECH1CH3_DCH2_IO_Write    L6230_ECH1CH3_DCH2_IO_Write
  *  @{
    * @brief Enable Input channel CH1 and CH3 for L6230   
  * @retval None
*/

void L6230_ECH1CH3_DCH2_IO_Write()
{
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH1,GPIO_SET);    //EN1 ENABLE               
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH3,GPIO_SET);    //EN3 ENABLE    
}
/**
  * @}  
  */
/** @defgroup L6230_ECH2CH3_DCH1_IO_Write    L6230_ECH2CH3_DCH1_IO_Write
  *  @{
    * @brief Enable Input channel CH2 and CH3 for L6230   
  * @retval None
*/
void L6230_ECH2CH3_DCH1_IO_Write()
{
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  //EN1 DISABLE               
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH2,GPIO_SET);    //EN2 ENABLE
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH3,GPIO_SET);    //EN3 ENABLE   
}
/**
  * @}  
  */
/** @defgroup L6230_DCH1CH2CH3_IO_Write    L6230_DCH1CH2CH3_IO_Write
  *  @{
    * @brief Disable all channels for L6230   
  * @retval None
*/
void L6230_DCH1CH2CH3_IO_Write()
{
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH1,GPIO_RESET);  //EN1 DISABLE          
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH2,GPIO_RESET);  //EN2 DISABLE
  HAL_GPIO_WritePin(GPIO_PORT_1,GPIO_CH3,GPIO_RESET);  //EN3 DISABLE   
}
/**
  * @}  
  */
/** @defgroup L6230_Start_PWM_generation    L6230_Start_PWM_generation
  *  @{
    * @brief Enable the PWM generation on Input channels for L6230 
  * @retval None
*/

void L6230_Start_PWM_generation()
{
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 ENABLE   
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 ENABLE   
  HAL_TIM_PWM_Start(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 ENABLE  
}
/**
  * @}  
  */
/** @defgroup L6230_Stop_PWM_generation    L6230_Stop_PWM_generation
  *  @{
    * @brief Disable the PWM generation on Input channels for L6230 
  * @retval None
*/
void L6230_Stop_PWM_generation()
{
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH1);           //TIM1_CH1 DISABLE   
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH2);           //TIM1_CH2 DISABLE   
  HAL_TIM_PWM_Stop(&HF_TIMx, HF_TIMx_CH3);           //TIM1_CH3 DISABLE  
}
/**
  * @}  
  */
/** @defgroup L6230_HFTIM_DC_CH1    L6230_HFTIM_DC_CH1
  *  @{
    * @brief Set the Duty Cycle value for CH1  
  * @retval None
*/
void L6230_HFTIM_DC_CH1(uint16_t CCRx)
{
  HF_TIMx.Instance->HF_TIMx_CCR1 = CCRx;  
}
/**
  * @}  
  */
/** @defgroup L6230_HFTIM_DC_CH2    L6230_HFTIM_DC_CH2
  *  @{
    * @brief Set the Duty Cycle value for CH2
  * @retval None
*/
void  L6230_HFTIM_DC_CH2(uint16_t CCRx)
{
  HF_TIMx.Instance->HF_TIMx_CCR2 = CCRx;  
}
/**
  * @}  
  */
/** @defgroup L6230_HFTIM_DC_CH3    L6230_HFTIM_DC_CH3
  *  @{
    * @brief Set the Duty Cycle value for CH3  
  * @retval None
*/
void  L6230_HFTIM_DC_CH3(uint16_t CCRx)
{
  HF_TIMx.Instance->HF_TIMx_CCR3 = CCRx;  
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
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
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
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}
/**
  * @}  
  */
/**
  * @}  end X-NUCLEO-IHM07M1 
  */

/**
  * @}  end BSP
  */

/**
  * @}  end DRIVERS
  */
