/**
 ******************************************************************************
 * @file    stm32F103_nucleo_ihm07m1.h
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This file provides the interface between the MC-lib and STM Nucleo
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
#ifndef __STM32F103_NUCLEO_IHM07M1_H
#define __STM32F103_NUCLEO_IHM07M1_H

  #include "stm32f1xx_hal.h"
  #include "main_F103.h"
  #include "X-NUCLEO-IHM07M1.h"

  #define HF_TIMx               htim1
  #define LF_TIMx               htim4
  #define HALL_ENCODER_TIMx     htim2
  #define ADCx                  hadc1
  #define REFx                  htim3
  #define UART                  huart2

  #define gState                State     /* for UART */

  #define GPIO_PORT_1           GPIOC
  #define GPIO_CH1              GPIO_PIN_10
  #define GPIO_PORT_2           GPIOC
  #define GPIO_CH2              GPIO_PIN_11
  #define GPIO_PORT_3           GPIOC
  #define GPIO_CH3              GPIO_PIN_12
  #define GPIO_SET              GPIO_PIN_SET
  #define GPIO_RESET            GPIO_PIN_RESET

  #define ADC_CH_1              ADC_CHANNEL_11  /*CURRENT*/
  #define ADC_CH_2              ADC_CHANNEL_9   /*SPEED*/
  #define ADC_CH_3              ADC_CHANNEL_1   /*VBUS*/
  #define ADC_CH_4              ADC_CHANNEL_12  /*TEMP*/
  #define ADC_Bemf_CH1          ADC_CHANNEL_13  /*BEMF1*/
  #define ADC_Bemf_CH2          ADC_CHANNEL_8   /*BEMF2*/
  #define ADC_Bemf_CH3          ADC_CHANNEL_7   /*BEMF3*/

  #define ADC_CH_1_ST           ADC_SAMPLETIME_7CYCLES_5    /*CURRENT sampling time */
  #define ADC_CH_2_ST           ADC_SAMPLETIME_7CYCLES_5   /*SPEED sampling time*/
  #define ADC_CH_3_ST           ADC_SAMPLETIME_7CYCLES_5     /*VBUS sampling time*/
  #define ADC_CH_4_ST           ADC_SAMPLETIME_7CYCLES_5   /*TEMP sampling time*/
  #define ADC_Bemf_CH1_ST       ADC_SAMPLETIME_7CYCLES_5   /*BEMF1 sampling time*/
  #define ADC_Bemf_CH2_ST       ADC_SAMPLETIME_7CYCLES_5   /*BEMF2 sampling time*/
  #define ADC_Bemf_CH3_ST       ADC_SAMPLETIME_7CYCLES_5   /*BEMF3 sampling time*/

  #define HF_TIMx_CH1           TIM_CHANNEL_1
  #define HF_TIMx_CH2           TIM_CHANNEL_2
  #define HF_TIMx_CH3           TIM_CHANNEL_3
  #define HF_TIMx_CCR1          CCR1            /*Channel 1*/
  #define HF_TIMx_CCR2          CCR2            /*Channel 2*/
  #define HF_TIMx_CCR3          CCR3            /*Channel 3*/

  #define DAC_ENABLE            0               /*!< Enable (1) the DAC peripheral */  
  #define DACx                  htim3           /*!<  TIM peripheral for DAC output */
  #define DAC_CH                TIM_CHANNEL_2   /*!<  TIM channel for DAC output */
  #define CCRx                  CCR2            /*!<  TIM channel number for DAC output (CCRx) */

  #define GPIO_PORT_ZCR         GPIOC           /*!<  GPIO port name for zero crossing detection */
  #define GPIO_CH_ZCR           GPIO_PIN_7      /*!<  GPIO pin name for zero crossing detection */
  #define GPIO_PORT_COMM        GPIOC           /*!<  GPIO port name for 6Step commutation */
  #define GPIO_CH_COMM          GPIO_PIN_4      /*!<  GPIO pin name for 6Step commutation */

  #define STARTM_CMD             0     /*!<  Start Motor command received */
  #define STOPMT_CMD             1     /*!<  Stop Motor command received */
  #define SETSPD_CMD             2     /*!<  Set the new speed value command received */
  #define GETSPD_CMD             3     /*!<  Get Mechanical Motor Speed command received */
  #define INIREF_CMD             4     /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
  #define POLESP_CMD             5     /*!<  Set the Pole Pairs value command received */
  #define ACCELE_CMD             6     /*!<  Set the Accelleration for Start-up of the motor command received */
  #define DMGCTR_CMD             7     /*!<  Enable the DEMAG dynamic control command received */
  #define MAXDMG_CMD             8     /*!<  Set the BEMF Demagn MAX command received */
  #define MINDMG_CMD             9     /*!<  Set the BEMF Demagn MIN command received */
  #define KP_PRM_CMD             10    /*!<  Set the KP PI param command received */
  #define KI_PRM_CMD             11    /*!<  Set the KI PI param command received */
  #define POTENZ_CMD             12    /*!<  Enable Potentiometer command received */
  #define HELP_CMD               13    /*!<  Help command received */
  #define STATUS_CMD             14    /*!<  Get the Status of the system command received */
  #define DIRECT_CMD             15    /*!<  Get the motor direction */

  /** @addtogroup stm32F103_nucleo_ihm07m1    stm32F103_nucleo_ihm07m1
  * @brief  Interface file for STM32F103 and Library configuration
  * @{ 
  */

  /** @defgroup Exported_function_F103 Exported_function_F103
  * @{
  */
  /** 
    * @brief  API function for STM32 instruction
    */
  void MC_SixStep_ADC_Channel(uint32_t);
  void MC_SixStep_Nucleo_Init(void);
  void START_Ref_Generation(void);
  void STOP_Ref_Generation(void);
  void Set_Ref_Generation(uint16_t);
  void START_DAC(void);
  void STOP_DAC(void);
  void SET_DAC_value(uint16_t);  
  void Bemf_delay_calc(void);
  uint32_t Get_UART_Data(void);
  void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D(void);
  void MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E(void);
  void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E(void);
  void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D(void);
  void MC_SixStep_Start_PWM_driving(void);
  void MC_SixStep_Stop_PWM_driving(void);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t);
  void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t);
  void MC_SixStep_Current_Reference_Start(void);
  void MC_SixStep_Current_Reference_Stop(void);
  void MC_SixStep_Current_Reference_Setvalue(uint16_t);  
  /**
  * @} 
  */
  
  /**
  * @} 
  */
  
#endif
