/**
 ******************************************************************************
 * @file    stm32F103_nucleo_ihm07m1.c
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


  #include "stm32F103_nucleo_ihm07m1.h"
  #include "6Step_Lib.h"

  extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters; /*!< Main SixStep structure*/ 
  extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters; /*!< SixStep PI regulator structure*/ 
  extern L6230_MotorDriver_TypeDef L6230MotorDriver;
  extern void MC_ADCx_SixStep_Bemf(void);
  extern void MC_TIMx_SixStep_timebase(void);
  extern void MC_SysTick_SixStep_MediumFrequencyTask(void);  
  
/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup MC_6-STEP_LIB    MC_6-STEP_LIB
  * @brief  Motor Control driver
  * @{ 
  */

/** @addtogroup stm32F030_nucleo_ihm07m1    stm32F030_nucleo_ihm07m1
  * @brief  Interface file for STM32F030 and Motor Control Library configuration
  * @{ 
  */

/** @defgroup MC_SixStep_ADC_Channel    MC_SixStep_ADC_Channel
  *  @{
    * @brief Select the new ADC Channel 
*/
/**
  * @brief  Select the new ADC Channel 
  * @param  adc_ch 
  * @retval None
*/

  void MC_SixStep_ADC_Channel(uint32_t adc_ch)
  {     
//    __HAL_ADC_DISABLE(&hadc1);
//   /* Regular sequence configuration */
//   /* Set the channel selection register from the selected channel */
//   MODIFY_REG(hadc1.Instance->SQR3,ADC_SQR3_RK(ADC_SQR3_SQ1, 1),ADC_SQR3_RK(adc_ch,1));
//   __HAL_ADC_ENABLE(&hadc1); 
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = adc_ch;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
     
  }
/**
  * @} 
  */
/** @defgroup MC_SixStep_Nucleo_Init    MC_SixStep_Nucleo_Init
  *  @{
    * @brief Init the STM32 register
*/
/**
  * @brief  Init the STM32 register
  * @retval None
*/
  void MC_SixStep_Nucleo_Init()
  {
    TIM_ClearInputConfigTypeDef sClearInputConfig;
    ADC_ChannelConfTypeDef sConfig;
    
    /******************** ETR CONFIGURATION **************************P*********/
    sClearInputConfig.ClearInputState = 1;  
    sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
    sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
    sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
    sClearInputConfig.ClearInputFilter = 0;
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH1);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH2);
    HAL_TIM_ConfigOCrefClear(&HF_TIMx, &sClearInputConfig, HF_TIMx_CH3);
    /***************************************************************************/

    __HAL_AFIO_REMAP_SWJ_NONJTRST(); /* Remap JNRST pin */
  
    __HAL_FREEZE_TIM1_DBGMCU();  /* Stop TIM during Breakpoint*/
    
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); /* Enable the TIM Break interrupt */
    
    /******************** REGULAR CHANNELS CONFIGURATION *************************/
    sConfig.Channel = ADC_CH_1; /* Current feedabck */
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_CH_1_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CH_3; /* Bus voltage */
    sConfig.SamplingTime = ADC_CH_3_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CH_4; /* Temperature feedback */
    sConfig.SamplingTime = ADC_CH_4_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH1; /* BEMF feedback phase A */
    sConfig.SamplingTime = ADC_Bemf_CH1_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH2; /* BEMF feedback phase B */
    sConfig.SamplingTime = ADC_Bemf_CH2_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_Bemf_CH3; /* BEMF feedback phase C */
    sConfig.SamplingTime = ADC_Bemf_CH3_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CH_2; /* Potentiometer */
    sConfig.SamplingTime = ADC_CH_2_ST;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);    
    /***************************************************************************/
  }
/**
  * @} 
  */

/** @defgroup START_Ref_Generation    START_Ref_Generation
  *  @{
    * @brief Start the current generation
*/
/**
  * @brief  Start the current generation
  * @retval None
*/
  void START_Ref_Generation()
  {
     REFx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Start(&REFx, HF_TIMx_CH1);   
  } 

/**
  * @} 
  */

/** @defgroup STOP_Ref_Generation    STOP_Ref_Generation
  *  @{
    * @brief Stop the current generation
*/
/**
  * @brief  Stop the current generation
  * @retval None
*/

  void STOP_Ref_Generation()
  {
     REFx.Instance->CCR1 = 0;
     HAL_TIM_PWM_Stop(&REFx, HF_TIMx_CH1);    
  }
/**
  * @} 
  */

/** @defgroup Set_Ref_Generation    Set_Ref_Generation
  *  @{
    * @brief Set the current generation value
*/
/**
  * @brief  Set the current generation value
  * @param  Iref Current reference value 
  * @retval None
*/
  void Set_Ref_Generation(uint16_t Iref)
  {
      REFx.Instance->CCR1 = (uint32_t)(Iref * REFx.Instance->ARR)/4096;
  }
/**
  * @} 
  */
  
  
  /** @defgroup START_DAC    START_DAC
  *  @{
     @brief Start DAC for debug
*/
/**
  * @brief  Start DAC for debug
  * @retval None
*/ 
  void START_DAC()
  {
     DACx.Instance->CCRx = 0;
     HAL_TIM_PWM_Start(&DACx, DAC_CH);       
  } 
/**
  * @} 
  */  
/** @defgroup STOP_DAC    STOP_DAC
  *  @{
     @brief Stop DAC for debug
*/
/**
  * @brief  Stop DAC for debug
  * @retval None
*/ 
  void STOP_DAC()
  {
     DACx.Instance->CCRx = 0;
     HAL_TIM_PWM_Stop(&DACx, DAC_CH);     
  }
/**
  * @} 
  */    
/** @defgroup SET_DAC_value    SET_DAC_value
  *  @{
     @brief Set DAC value for debug
*/
/**
  * @brief  Set DAC value for debug
  * @param  dac_value: information to plot through DAC
  * @retval None
*/ 
  void SET_DAC_value(uint16_t dac_value)
  {
       DACx.Instance->CCRx = (uint32_t)(dac_value * DACx.Instance->ARR)/4096;    
  }
/**
  * @} 
  */

/** @defgroup HAL_ADC_ConvCpltCallback    HAL_ADC_ConvCpltCallback
  *  @{
    * @brief ADC callback 
*/
/**
  * @brief  ADC callback
  * @param  hadc
  * @retval None
*/

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
     MC_ADCx_SixStep_Bemf();
  }
/**
  * @} 
  */

/** @defgroup HAL_TIM_PeriodElapsedCallback    HAL_TIM_PeriodElapsedCallback
  *  @{
    * @brief htim callback 
*/
/**
  * @brief  htim callback
  * @param  htim
  * @retval None
*/
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    MC_TIMx_SixStep_timebase();
  }
/**
  * @} 
  */

/** @defgroup HAL_SYSTICK_Callback    HAL_SYSTICK_Callback
  *  @{
    * @brief Systick callback 
*/
/**
  * @brief  Systick callback 
  * @retval None
*/
  void HAL_SYSTICK_Callback()
  {
    MC_SysTick_SixStep_MediumFrequencyTask();
  }
/**
  * @} 
  */    

/** @defgroup HAL_GPIO_EXTI_Callback    HAL_GPIO_EXTI_Callback
  *  @{
    * @brief EXT callback 
*/
/**
  * @brief  EXT callback
  * @param  GPIO_Pin
  * @retval None
*/  
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
      MC_EXT_button_SixStep();
  }
/**
  * @} 
  */
  
/** @defgroup Get_UART_data   Get_UART_data
  *  @{
    * @brief Get the UART value from DR register
*/
/**
  * @brief  Get the UART value from DR register
  * @retval uint32_t
*/  
uint32_t Get_UART_Data()
{
  return (UART.Instance->DR);
}
/**
 * @}  
 */ 


/** @defgroup EnableInput_CH1_E_CH2_E_CH3_D    EnableInput_CH1_E_CH2_E_CH3_D
  *  @{
    * @brief Enable Input channel for L6230
  */
/**
  * @brief  Enable Input channel CH1 and CH2 for L6230     
  * @retval None
*/

void MC_SixStep_EnableInput_CH1_E_CH2_E_CH3_D()
{
   L6230MotorDriver.EnableInput_CH1_E_CH2_E_CH3_D();
}

/**
  * @} 
  */  
  
 /** @defgroup EnableInput_CH1_E_CH2_D_CH3_E    EnableInput_CH1_E_CH2_D_CH3_E
  *  @{
    * @brief Enable Input channel for L6230
  */
/**
  * @brief  Enable Input channel CH1 and CH3 for L6230           
  * @retval None
*/

void  MC_SixStep_EnableInput_CH1_E_CH2_D_CH3_E()
{
  L6230MotorDriver.EnableInput_CH1_E_CH2_D_CH3_E();
}

/**
  * @} 
  */ 

/** @defgroup EnableInput_CH1_D_CH2_E_CH3_E    EnableInput_CH1_D_CH2_E_CH3_E
  *  @{
    * @brief Enable Input channel for L6230
  */
/**
  * @brief  Enable Input channel CH2 and CH3 for L6230           
  * @retval None
*/

void MC_SixStep_EnableInput_CH1_D_CH2_E_CH3_E()
{
  L6230MotorDriver.EnableInput_CH1_D_CH2_E_CH3_E();
}

/**
  * @} 
  */

/** @defgroup DisableInput_CH1_D_CH2_D_CH3_D    DisableInput_CH1_D_CH2_D_CH3_D
  *  @{
    * @brief Disable All Input channels for L6230
  */
/**
  * @brief  Enable Input channel CH2 and CH3 for L6230           
  * @retval None
*/

void MC_SixStep_DisableInput_CH1_D_CH2_D_CH3_D()
{
  L6230MotorDriver.DisableInput_CH1_D_CH2_D_CH3_D();
}

/**
  * @} 
  */

/** @defgroup Start_PWM_driving    Start_PWM_driving
  *  @{
    * @brief Enable the PWM generation on Input channels
  */
/**
  * @brief  Enable PWM channels for L6230           
  * @retval None
*/

void MC_SixStep_Start_PWM_driving()
{
   L6230MotorDriver.Start_PWM_driving();
} 

/**
  * @} 
  */

/** @defgroup Stop_PWM_driving    Stop_PWM_driving
  *  @{
    * @brief Disable the PWM generation on Input channels
  */
/**
  * @brief  Disable PWM channels for L6230           
  * @retval None
*/

void MC_SixStep_Stop_PWM_driving()
{
  L6230MotorDriver.Stop_PWM_driving();
}  

/**
  * @}
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH1    HF_TIMx_SetDutyCycle_CH1
  *  @{
    * @brief Set the Duty Cycle value for CH1 
*/
/**
  * @brief  Set the Duty Cycle value for CH1           
  * @retval None
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH1(uint16_t CCR_value)
{ 
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH1(CCR_value);
}


/**
  * @} 
  */

/** @defgroup HF_TIMx_SetDutyCycle_CH2    HF_TIMx_SetDutyCycle_CH2
  *  @{
    * @brief Set the Duty Cycle value for CH2 
*/
/**
  * @brief  Set the Duty Cycle value for CH2           
  * @retval None
*/

void MC_SixStep_HF_TIMx_SetDutyCycle_CH2(uint16_t CCR_value)
{ 
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH2(CCR_value);
}
/**
  * @} 
  */






/** @defgroup HF_TIMx_SetDutyCycle_CH3    HF_TIMx_SetDutyCycle_CH3
  *  @{
    * @brief Set the Duty Cycle value for CH3 
*/
/**
  * @brief  Set the Duty Cycle value for CH3           
  * @retval None
*/


void MC_SixStep_HF_TIMx_SetDutyCycle_CH3(uint16_t CCR_value)
{ 
  L6230MotorDriver.HF_TIMx_SetDutyCycle_CH3(CCR_value);
}

/**
  * @} 
  */

/** @defgroup Current_Reference_Start    Current_Reference_Start
  *  @{
    * @brief Enable the Current Reference generation
*/
/**
  * @brief  Enable the Current Reference generation  
  * @retval None
*/

void MC_SixStep_Current_Reference_Start()
{
  L6230MotorDriver.Current_Reference_Start();
}

/**
  * @} 
  */


/** @defgroup Current_Reference_Stop    Current_Reference_Stop
  *  @{
    * @brief Disable the Current Reference generation
*/
/**
  * @brief  Disable the Current Reference generation
  * @retval None
*/

void MC_SixStep_Current_Reference_Stop()
{
  L6230MotorDriver.Current_Reference_Stop();
}

/**
  * @}  
  */


/** @defgroup Current_Reference_Setvalue    Current_Reference_Setvalue
  *  @{
    * @brief Set the value for Current Reference
*/
/**
  * @brief  Set the value for Current Reference
  * @retval None
*/


void MC_SixStep_Current_Reference_Setvalue(uint16_t Iref)
{
  L6230MotorDriver.Current_Reference_Setvalue(Iref);
}

/**
  * @}  
  */  



/** @defgroup Bemf_delay_calc    Bemf_delay_calc
  *  @{
    * @brief Bemf delay calculation
*/
/**
  * @brief  Bemf delay calculation
  * @retval None
*/  
void Bemf_delay_calc()
{
if(PI_parameters.Reference>=0)
 {  
 if(SIXSTEP_parameters.speed_fdbk_filtered<=12000 && SIXSTEP_parameters.speed_fdbk_filtered>10000)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;    
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=10000 && SIXSTEP_parameters.speed_fdbk_filtered>7800)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2; 
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=7800 && SIXSTEP_parameters.speed_fdbk_filtered>6400) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;    
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=6400 && SIXSTEP_parameters.speed_fdbk_filtered>5400)  
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=5400 && SIXSTEP_parameters.speed_fdbk_filtered>4650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4650 && SIXSTEP_parameters.speed_fdbk_filtered>4100) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=4100 && SIXSTEP_parameters.speed_fdbk_filtered>3650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;      
  }     
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=3650 && SIXSTEP_parameters.speed_fdbk_filtered>3300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=3300 && SIXSTEP_parameters.speed_fdbk_filtered>2600) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;    
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=2600 && SIXSTEP_parameters.speed_fdbk_filtered>1800) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1800 && SIXSTEP_parameters.speed_fdbk_filtered>1500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1500 && SIXSTEP_parameters.speed_fdbk_filtered>1300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }    
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1300 && SIXSTEP_parameters.speed_fdbk_filtered>1000) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered<=1000 && SIXSTEP_parameters.speed_fdbk_filtered>500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  }  
 }
 else 
 {
  if(SIXSTEP_parameters.speed_fdbk_filtered>=-12000 && SIXSTEP_parameters.speed_fdbk_filtered<-10000)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_1;    
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-10000 && SIXSTEP_parameters.speed_fdbk_filtered<-7800)
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_2; 
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-7800 && SIXSTEP_parameters.speed_fdbk_filtered<-6400) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_3;    
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-6400 && SIXSTEP_parameters.speed_fdbk_filtered<-5400)  
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_4;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-5400 && SIXSTEP_parameters.speed_fdbk_filtered<-4650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_5;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4650 && SIXSTEP_parameters.speed_fdbk_filtered<-4100) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_6;      
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-4100 && SIXSTEP_parameters.speed_fdbk_filtered<-3650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_7;      
  }     
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3650 && SIXSTEP_parameters.speed_fdbk_filtered<-3300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_8;      
  }
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-3300 && SIXSTEP_parameters.speed_fdbk_filtered<-2650) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_9;     
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-2600 && SIXSTEP_parameters.speed_fdbk_filtered<-1800) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_10;
  } 
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1800 && SIXSTEP_parameters.speed_fdbk_filtered<-1500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_11;
  }   
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1500 && SIXSTEP_parameters.speed_fdbk_filtered<-1300) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_12;
  }   
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1300 && SIXSTEP_parameters.speed_fdbk_filtered<-1000) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_13;
  }  
  else if(SIXSTEP_parameters.speed_fdbk_filtered>=-1000 && SIXSTEP_parameters.speed_fdbk_filtered<-500) 
  { 
    SIXSTEP_parameters.demagn_value = DEMAGN_VAL_14;
  } 
 }
}

/**
  * @}  
  */

/**
  * @} // end STM32F302_Interface
  */

/**
  * @}  end MC_6-STEP_LIB 
  */
    
/**
  * @} // end MIDDLEWARES
  */
