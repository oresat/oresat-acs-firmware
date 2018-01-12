/**
 ******************************************************************************
 * @file    STSPIN230.h
 * @author  IPC
 * @version V1.0.0
 * @date    04-April-2016
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN230_H
#define __STSPIN230_H

#include "stdint.h" 
#include "MC_Common.h" 

 
 
  extern void STSPIN230_EnableInput_CH1_E_CH2_E_CH3_D(void);
  extern void STSPIN230_EnableInput_CH1_E_CH2_D_CH3_E(void);
  extern void STSPIN230_EnableInput_CH1_D_CH2_E_CH3_E(void);
  extern void STSPIN230_DisableInput_CH1_D_CH2_D_CH3_D(void);
  extern void STSPIN230_Start_PWM_driving(void);
  extern void STSPIN230_Stop_PWM_driving(void);
  extern void STSPIN230_HF_TIMx_SetDutyCycle_CH1(uint16_t);
  extern void STSPIN230_HF_TIMx_SetDutyCycle_CH2(uint16_t);
  extern void STSPIN230_HF_TIMx_SetDutyCycle_CH3(uint16_t);
  extern void STSPIN230_Current_Reference_Start(void);
  extern void STSPIN230_Current_Reference_Stop(void);
  extern void STSPIN230_Current_Reference_Setvalue(uint16_t); 
  extern void STSPIN230_START_Ref_Generation(void);
  extern void STSPIN230_STOP_Ref_Generation(void);
  extern void STSPIN230_Set_Ref_Generation(uint16_t);

  void EnableInput_CH1_E_CH2_E_CH3_D(void);
  void EnableInput_CH1_E_CH2_D_CH3_E(void);
  void EnableInput_CH1_D_CH2_E_CH3_E(void);
  void DisableInput_CH1_D_CH2_D_CH3_D(void);
  void Start_PWM_driving(void);
  void Stop_PWM_driving(void);
  void HF_TIMx_SetDutyCycle_CH1(uint16_t);
  void HF_TIMx_SetDutyCycle_CH2(uint16_t);
  void HF_TIMx_SetDutyCycle_CH3(uint16_t);
  void Current_Reference_Start(void);
  void Current_Reference_Stop(void);
  void Current_Reference_Setvalue(uint16_t);


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



/**
  * @} 
  */

#endif
