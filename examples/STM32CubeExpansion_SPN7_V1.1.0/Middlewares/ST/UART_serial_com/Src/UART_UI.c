/**
 ******************************************************************************
 * @file    UART_UI.c
 * @author  System lab - Automation and Motion control team
 * @version V1.1.0
 * @date    08-June-2017
 * @brief   This file provides a set of functions needed to manage the UART com.
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

/* Includes ------------------------------------------------------------------*/
#include "UART_UI.h"

/* Special messages */
#ifdef UART_COMM
 static uint8_t aUartDeleteCharMsg[] = " \b";
#endif
#define USART_ENTER_CHAR                0x0D // \r character
#define USART_BACKSPACE_CHAR            0x08 // \b character
#define USART_SPACE_CHAR                0x20 // ' ' character
#define aRxBuffer_MAX                     10


#ifdef UART_COMM  

extern SIXSTEP_Base_InitTypeDef SIXSTEP_parameters;      /*!< Main SixStep structure*/ 
extern SIXSTEP_PI_PARAM_InitTypeDef_t PI_parameters;     /*!< SixStep PI regulator structure*/ 
extern uint8_t Enable_start_button;
extern void CMD_Parser(char* pCommandString);
extern void MC_Set_PI_param(SIXSTEP_PI_PARAM_InitTypeDef_t *);

static uint16_t Uart_cmd_flag = 0;
const CMD_T CmdTable[] = {
  {"STARTM", CMD_STARTM},
  {"STOPMT", CMD_STOPMT}, 
  {"DIRECT", CMD_DIRECTION},   
  {"SETSPD", CMD_SETSPD}, 
  {"STATUS", CMD_STATUS},     
  {"GETSPD", CMD_GETSPD},    
  {"POTENZ", CMD_POTENZ}, 
  {"INIREF", CMD_INIREF},   
  {"POLESP", CMD_POLESP}, 
  {"ACCELE", CMD_ACCELE},   
  {"KP-PRM", CMD_KP_PRM},  
  {"KI-PRM", CMD_KI_PRM},    
  {"HELP", CMD_HELP},   
  {"", NULL}
};

uint8_t BUFF_RCV = RXBUFFERSIZE;
uint8_t aRxBuffer[10];     /*!< Buffer used for reception */
uint8_t row0TxBuffer[] = "\033[2J";  /*!< Buffer used for transmission */
uint8_t row1TxBuffer[] = "*******************************************************************************\n\r";
uint8_t row2TxBuffer[] = "*             X-NUCLEO-IHM07M1 - 6-Step Motor Control Expansion board        *\n\r";
uint8_t row3TxBuffer[] = "*******************************************************************************\n\r";
uint8_t row4TxBuffer[] = " List of commands:\n\r\n\r";
uint8_t row5TxBuffer[] = "  <STARTM> -- Start Motor\n\r";
uint8_t row6TxBuffer[] = "  <STOPMT> -- Stop Motor\n\r";
uint8_t row7aTxBuffer[] = "  <DIRECT> -- Set the Motor direction CW or CCW \n\r";
uint8_t row7TxBuffer[] = "  <SETSPD> -- Set the Motor Speed\n\r";
uint8_t row7bTxBuffer[] = "  <GETSPD> -- Get the Motor Speed\n\r";
uint8_t row7cTxBuffer[] = "  <STATUS> -- Get the Status of system \n\r";
uint8_t row71TxBuffer[] = "  <POTENZ> -- Enable/Disable the potentiometer\n\r";
uint8_t row71bTxBuffer[] = "  <HELP>   -- Show the help menu \n\r";
uint8_t rowVTxBuffer[] = "\n>>> Insert the value: ";
uint8_t rowVTBxBuffer[] = "\n>>> ENABLE <1> DISABLE <0>: ";
uint8_t rowV1TBxBuffer[] = "\n>>> CW <0> CCW <1>: ";
uint8_t rowMxBuffer[] = "\n>>> START MOTOR COMMAND RECEIVED ! <<< \n\r >";
uint8_t rowSxBuffer[] = "\n>>> STOP MOTOR COMMAND RECEIVED ! <<< \n\r >";
uint8_t rowETxBuffer[] = "\n>>> ERROR - PLEASE TYPE AGAIN ! <<< \n\r >";
uint8_t row8TxBuffer[] = "             ************* MAIN MOTOR PARAMETERS ***************       \n\r";
uint8_t row9TxBuffer[] = "  <INIREF> -- Start-up current reference (0-4095) \n\r";
uint8_t row10TxBuffer[] = "  <ACCELE> -- Motor acceleration value during startup \n\r";
uint8_t row11TxBuffer[] = "  <POLESP> -- Set the Motor pole pairs \n\r";
uint8_t row14TxBuffer[] = "          ****** PI REGULATOR PARAMETERS - SPEED LOOP  ******\n\r";
uint8_t row15TxBuffer[] = "  <KP-PRM> -- Set the PI proportional term \n\r";
uint8_t row16TxBuffer[] = "  <KI-PRM> -- Set the PI integral term \n\r\n\r >";
uint8_t rowLxBuffer[] = "\n--- OK ---\n\r >";
extern uint8_t UART_FLAG_RECEIVE;
extern uint8_t UART_FLAG_POTENZ;


/** @addtogroup MIDDLEWARES     MIDDLEWARES 
  * @brief  Middlewares Layer
  * @{ 
  */


/** @addtogroup UART_UI  UART_UI
  * @brief  Serial communication through PC serial terminal
  * @{ 
  */


/** @defgroup UART_Communication_Task    UART_Communication_Task
  *  @{
    * @brief UART start receive function
*/
void UART_Communication_Task()
{ 
 if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, 10) != HAL_OK) 
  {  
    huart2.gState = HAL_UART_STATE_READY; 
  }
 else UART_FLAG_RECEIVE = FALSE;
}
/**
  * @} 
  */

/** @defgroup MC_UI_INIT    MC_UI_INIT
  *  @{
    * @brief UART User Interface init
*/
 void MC_UI_INIT()
 {
   HAL_UART_Transmit(&huart2, (uint8_t *)row0TxBuffer, (COUNTOF(row0TxBuffer) - 1),5000);
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart2, (uint8_t *)row1TxBuffer, (COUNTOF(row1TxBuffer) - 1),5000);
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart2, (uint8_t *)row2TxBuffer, (COUNTOF(row2TxBuffer) - 1),5000);   
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }    
   HAL_UART_Transmit(&huart2, (uint8_t *)row3TxBuffer, (COUNTOF(row3TxBuffer) - 1),5000);
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart2, (uint8_t *)row4TxBuffer, (COUNTOF(row4TxBuffer) - 1),5000);    
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart2, (uint8_t *)row5TxBuffer, (COUNTOF(row5TxBuffer) - 1),5000);   
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart2, (uint8_t *)row6TxBuffer, (COUNTOF(row6TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart2, (uint8_t *)row7aTxBuffer, (COUNTOF(row7aTxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart2, (uint8_t *)row7TxBuffer, (COUNTOF(row7TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }
   HAL_UART_Transmit(&huart2, (uint8_t *)row7bTxBuffer, (COUNTOF(row7bTxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }  
   HAL_UART_Transmit(&huart2, (uint8_t *)row7cTxBuffer, (COUNTOF(row7cTxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart2, (uint8_t *)row71TxBuffer, (COUNTOF(row71TxBuffer) - 1),5000);
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    } 
   HAL_UART_Transmit(&huart2, (uint8_t *)row71bTxBuffer, (COUNTOF(row71bTxBuffer) - 1),5000);
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }  
   HAL_UART_Transmit(&huart2, (uint8_t *)row8TxBuffer, (COUNTOF(row8TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }     
   HAL_UART_Transmit(&huart2, (uint8_t *)row9TxBuffer, (COUNTOF(row9TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }       
   HAL_UART_Transmit(&huart2, (uint8_t *)row10TxBuffer, (COUNTOF(row10TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }       
   HAL_UART_Transmit(&huart2, (uint8_t *)row11TxBuffer, (COUNTOF(row11TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }      
   HAL_UART_Transmit(&huart2, (uint8_t *)row14TxBuffer, (COUNTOF(row14TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }
   HAL_UART_Transmit(&huart2, (uint8_t *)row15TxBuffer, (COUNTOF(row15TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }   
   HAL_UART_Transmit(&huart2, (uint8_t *)row16TxBuffer, (COUNTOF(row16TxBuffer) - 1),5000);     
   while (huart2.gState != HAL_UART_STATE_READY)
    {
    }  
   UART_Communication_Task();
   SIXSTEP_parameters.Button_ready = TRUE;
 }
/**
  * @} 
  */


/** @defgroup UART_num_decode    UART_num_decode
  *  @{
    * @brief UART Value decoding function
*/
uint32_t UART_num_decode()
{
 static char Value_Buffer[RXBUFFERSIZE];
 
 for(uint8_t i=0;i<BUFF_RCV;i++)
  {
    Value_Buffer[i] = aRxBuffer[i];
  }
return(atoi(Value_Buffer));
}
/**
  * @} 
  */

/** @defgroup UART_Set_Value    UART_Set_Value
  *  @{
    * @brief UART Main function
*/
void UART_Set_Value()
{
 if(Get_UART_Data() == USART_ENTER_CHAR && (huart2.gState != HAL_UART_STATE_BUSY_TX && huart2.gState != HAL_UART_STATE_BUSY_TX_RX)) 
  {
   if(Uart_cmd_flag == 0)
    {     
     CMD_Parser((char*)aRxBuffer);
     for(uint8_t index=0; index < aRxBuffer_MAX; index++)
     {
       aRxBuffer[index] = 0;
     }
     huart2.RxXferCount = huart2.RxXferSize;
     huart2.pRxBuffPtr = aRxBuffer;
    }
   else 
   {
    SIXSTEP_parameters.Uart_value_to_set = UART_num_decode();
    
    switch(SIXSTEP_parameters.Uart_cmd_to_set) 
    {
     case SETSPD_CMD:  /*!<  Set the new speed value command received */
        PI_parameters.Reference = SIXSTEP_parameters.Uart_value_to_set;
        SIXSTEP_parameters.Ramp_Start = 1;
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
        huart2.gState = HAL_UART_STATE_READY;    
        break;
     case INIREF_CMD:  /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */     
        SIXSTEP_parameters.Ireference = SIXSTEP_parameters.Uart_value_to_set;  
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;      
      break; 
     case POLESP_CMD:  /*!<  Set the Pole Pairs value command received */
        SIXSTEP_parameters.NUMPOLESPAIRS = SIXSTEP_parameters.Uart_value_to_set;  
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
      break;  
     case ACCELE_CMD:  /*!<  Set the Accelleration for Start-up of the motor command received */
        SIXSTEP_parameters.ACCEL = SIXSTEP_parameters.Uart_value_to_set;  
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
       break;   
     case DIRECT_CMD:  /*!<  Set the motor direction */
        SIXSTEP_parameters.CW_CCW = SIXSTEP_parameters.Uart_value_to_set;  
        MC_Set_PI_param(&PI_parameters);         
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
       break;            
     case KP_PRM_CMD:  /*!<  Set the KP PI param command received */
        PI_parameters.Kp_Gain = SIXSTEP_parameters.Uart_value_to_set;  
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
       break;  
     case KI_PRM_CMD:  /*!<  Set the KI PI param command received */
        PI_parameters.Ki_Gain = SIXSTEP_parameters.Uart_value_to_set;  
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
       break;     
      case POTENZ_CMD:  /*!<  Enable Potentiometer command received */
        BUFF_RCV = RXBUFFERSIZE;        
        Uart_cmd_flag = 0;       
        huart2.gState = HAL_UART_STATE_READY;
        HAL_UART_Transmit(&huart2, (uint8_t *)rowLxBuffer, (COUNTOF(rowLxBuffer) - 1),5000);        
        UART_FLAG_RECEIVE = TRUE;
        SIXSTEP_parameters.Potentiometer = SIXSTEP_parameters.Uart_value_to_set;          
      break;
      }  /* switch case */
     for(uint8_t index=0; index < aRxBuffer_MAX; index++)
     {
       aRxBuffer[index] = 0;
     }
     huart2.RxXferCount = huart2.RxXferSize;
     huart2.pRxBuffPtr = aRxBuffer;
    }  /* else */
  }  /* if */  
 if(Get_UART_Data() == USART_BACKSPACE_CHAR)
 {
   HAL_UART_Transmit(&huart2, (uint8_t *)aUartDeleteCharMsg, 2,5000); 
   huart2.RxXferCount+=2;
   huart2.pRxBuffPtr-=2;
   
 }
}
/**
  * @} 
  */

    
/** @defgroup CMD_MSG    CMD_MSG
  *  @{
    * @brief UART Transmit standard message
*/
void CMD_MSG()
{
    HAL_UART_Transmit(&huart2, (uint8_t *)rowVTxBuffer, (COUNTOF(rowVTxBuffer) - 1),5000);
    BUFF_RCV = RXBUFFERSIZE - 1;
    Uart_cmd_flag = 1;
    UART_FLAG_RECEIVE = TRUE;    
    huart2.gState = HAL_UART_STATE_READY;        
}
/**
  * @} 
  */

/** @defgroup CMD_MSG_BOOL    CMD_MSG_BOOL
  *  @{
    * @brief UART Transmit standard message for input data
*/
void CMD_MSG_BOOL()
{
    huart2.gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit(&huart2, (uint8_t *)rowVTBxBuffer, (COUNTOF(rowVTBxBuffer) - 1),5000);
    BUFF_RCV = RXBUFFERSIZE - 1;        
    Uart_cmd_flag = 1;
    UART_FLAG_RECEIVE = TRUE;       
}
/**
  * @} 
  */

/** @defgroup CMD_MSG_BOOL_DIRECT    CMD_MSG_BOOL_DIRECT
  *  @{
    * @brief UART Transmit CW or CCW message for input data
*/  
void CMD_MSG_BOOL_DIRECT()
{
    huart2.gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit(&huart2, (uint8_t *)rowV1TBxBuffer, (COUNTOF(rowV1TBxBuffer) - 1),5000);
    BUFF_RCV = RXBUFFERSIZE - 1;        
    Uart_cmd_flag = 1;
    UART_FLAG_RECEIVE = TRUE;       
}
/**
  * @} 
  */

/** @defgroup CMD_STARTM    CMD_STARTM
  *  @{
    * @brief UART Transmit Start motor message  
*/ 
void CMD_STARTM()
{     
  /*!<  Start Motor command received */
      huart2.gState = HAL_UART_STATE_READY;        
      HAL_UART_Transmit(&huart2, (uint8_t *)rowMxBuffer, (COUNTOF(rowMxBuffer) - 1),5000);         
      MC_StartMotor();
      Enable_start_button = FALSE;   
      UART_FLAG_RECEIVE = TRUE;    
}
/**
  * @} 
  */

/** @defgroup CMD_STOPMT    CMD_STOPMT
  *  @{
    * @brief UART Transmit Stop motor message  
*/
void CMD_STOPMT()
{      
  /*!<  Stop Motor command received */
      huart2.gState = HAL_UART_STATE_READY;         
      HAL_UART_Transmit(&huart2, (uint8_t *)rowSxBuffer, (COUNTOF(rowSxBuffer) - 1),5000);               
      MC_StopMotor();
      Enable_start_button = FALSE;         
      UART_FLAG_RECEIVE = TRUE;   
}
/**
  * @} 
  */

/** @defgroup CMD_SETSPD    CMD_SETSPD
  *  @{
    * @brief UART Change the motor speed  
*/ 
void CMD_SETSPD()
{    
 /*!<  Set the new speed value command received */
        CMD_MSG();
        SIXSTEP_parameters.Uart_cmd_to_set = SETSPD_CMD;
}
/**
  * @} 
  */

/** @defgroup CMD_GETSPD    CMD_GETSPD
  *  @{
    * @brief UART Get the motor speed  
*/
void CMD_GETSPD()
{    
 /*!<  Get Mechanical Motor Speed command received */  
      static char strLineMessage[40];

      huart2.gState = HAL_UART_STATE_READY; 
      sprintf(strLineMessage, "\n-- The Motor Speed is: %d RPM -- \r\n >", SIXSTEP_parameters.speed_fdbk_filtered); 
      HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage,sizeof(strLineMessage),5000);           
      UART_FLAG_RECEIVE = TRUE; 
}
/**
  * @} 
  */

/** @defgroup CMD_POTENZ    CMD_POTENZ
  *  @{
    * @brief UART Enable the potentiometer
*/ 
void CMD_POTENZ()
{    
 /*!<  Enable Potentiometer command received */
      CMD_MSG_BOOL();
      SIXSTEP_parameters.Uart_cmd_to_set = POTENZ_CMD;
}
/**
  * @} 
  */

/** @defgroup CMD_HELP    CMD_HELP
  *  @{
    * @brief UART Help command
*/ 
void CMD_HELP()
{
 /*!<  Help command received */
      huart2.gState = HAL_UART_STATE_READY; 
      MC_UI_INIT();       
      UART_FLAG_RECEIVE = TRUE; 
}  
/**
  * @} 
  */


/** @defgroup CMD_INIREF    CMD_INIREF
  *  @{
    * @brief UART Set the current reference
*/
void CMD_INIREF()
{
 /*!<  Set the new STARUP_CURRENT_REFERENCE value command received */
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = INIREF_CMD;
}
/**
  * @} 
  */


/** @defgroup CMD_POLESP    CMD_POLESP
  *  @{
    * @brief UART Set the motor poles pairs
*/
void CMD_POLESP()
{
 /*!<  Set the Pole Pairs value command received */
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = POLESP_CMD;      
}
/**
  * @} 
  */

/** @defgroup CMD_ACCELE    CMD_ACCELE
  *  @{
    * @brief UART Set the accelleration of the motor at start-up
*/
void CMD_ACCELE()
{
 /*!<  Set the Accelleration for Start-up of the motor command received */  
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = ACCELE_CMD;  
}
/**
  * @} 
  */

/** @defgroup CMD_DIRECTION   CMD_DIRECTION
  *  @{
    * @brief UART Set the motor direction
*/
void CMD_DIRECTION()
{ 
 /*!<  Enable the DEMAG dynamic control command received */
      CMD_MSG_BOOL_DIRECT();
      SIXSTEP_parameters.Uart_cmd_to_set = DIRECT_CMD;        
}
/**
  * @} 
  */

/** @defgroup CMD_KP_PRM    CMD_KP_PRM
  *  @{
    * @brief UART Set the  KP PI param
*/
void CMD_KP_PRM()
{ 
  /*!<  Set the KP PI param command received */  
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = KP_PRM_CMD;     
}
/**
  * @} 
  */

/** @defgroup CMD_KI_PRM    CMD_KI_PRM
  *  @{
    * @brief UART Set the KI PI param
*/
void CMD_KI_PRM()
{ 
 /*!<  Set the KI PI param command received */    
      CMD_MSG();
      SIXSTEP_parameters.Uart_cmd_to_set = KI_PRM_CMD;     
}
/**
  * @} 
  */

/** @defgroup CMD_STATUS    CMD_STATUS
  *  @{
    * @brief UART View the STATUS
*/
void CMD_STATUS()
{
   static char strLineMessage1[30];
   static char strLineMessage2[30];
   static char strLineMessage3[30];
   static char strLineMessage4[30];
   static char strLineMessage5[30];
   static char strLineMessage6[30]; 
   static char strLineMessage7[30];
   static char strLineMessage8[30];  
 
   huart2.gState = HAL_UART_STATE_READY;   
   switch (SIXSTEP_parameters.STATUS)
     {
         case STARTUP:
              sprintf(strLineMessage1, "\n-- The status is: STARTUP --\r\n >");
              HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage1,sizeof(strLineMessage1),5000);                
          break; 
          case VALIDATION:
             sprintf(strLineMessage2, "\n-- The status is: VALIDATION --\r\n >");
             HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage2,sizeof(strLineMessage2),5000);                   
          break;
          case START:
            sprintf(strLineMessage3, "\n-- The status is: START --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage3,sizeof(strLineMessage3),5000);                
          break;
          case STOP: 
            sprintf(strLineMessage4, "\n-- The status is: STOP --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage4,sizeof(strLineMessage4),5000);                
          break;   
          case RUN: 
            sprintf(strLineMessage5, "\n-- The status is: RUN --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage5,sizeof(strLineMessage5),5000);                
          break;             
          case ALIGNMENT: 
            sprintf(strLineMessage6, "\n-- The status is: ALIGNMENT --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage6,sizeof(strLineMessage6),5000);                
          break;   
          case SPEEDFBKERROR: 
            sprintf(strLineMessage7, "\n-- The status is: SPEEDFBKERROR --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage7,sizeof(strLineMessage7),5000);                  
          break;    
          default:
            sprintf(strLineMessage8, "\n-- The status is: IDLE --\r\n >");
            HAL_UART_Transmit(&huart2, (uint8_t*) strLineMessage8,sizeof(strLineMessage8),5000);                
          break; 
       }
      UART_FLAG_RECEIVE = TRUE; 
}
/**
  * @} 
  */

/** @defgroup CMD_Parser    CMD_Parser
  *  @{
    * @brief UART String parser 
*/
void CMD_Parser(char* pCommandString)
{
    static uint8_t CmdListIndex;
    char sCmd[16];
    
    strcpy(sCmd, pCommandString);
    strtok (sCmd, TOKEN);
    /* Command Callback identification */
    for(CmdListIndex=0;CmdTable[CmdListIndex].pCmdFunc!=NULL; CmdListIndex++) {
        if (strcmp(CmdTable[CmdListIndex].name, sCmd) == 0 ){
          break;
        }
    }
    if ( CmdListIndex < CMD_NUM ){
        // CMD OK --> extract parameters
        /* Check for valid callback */
        if(CmdTable[CmdListIndex].pCmdFunc!=NULL) {
            CmdTable[CmdListIndex].pCmdFunc();
        }
    }
    else{
          huart2.gState = HAL_UART_STATE_READY;
          HAL_UART_Transmit(&huart2, (uint8_t *)rowETxBuffer, (COUNTOF(rowETxBuffer) - 1),5000);   
          BUFF_RCV = RXBUFFERSIZE;        
          Uart_cmd_flag = 0;   
          UART_FLAG_RECEIVE = TRUE;  
          return;
    }
}

/**
  * @} 
  */


/**
  * @}  end UART_UI 
  */

/**
  * @}  end MIDDLEWARES
  */
#endif
