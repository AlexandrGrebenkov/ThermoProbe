/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "comp.h"
#include "crc.h"
#include "tim.h"

#include "Serial.h"
#include "DataTypes.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId TempSensorsTaskHandle;
osThreadId HumiditySensorHandle;
osTimerId UART_TimerHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void vTempSensorsTask(void const * argument);
void vHumiditySensor(void const * argument);
void UART_TimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
   HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
/* USER CODE END 2 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of UART_Timer */
  osTimerDef(UART_Timer, UART_TimerCallback);
  UART_TimerHandle = osTimerCreate(osTimer(UART_Timer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TempSensorsTask */
  osThreadDef(TempSensorsTask, vTempSensorsTask, osPriorityNormal, 0, 128);
  TempSensorsTaskHandle = osThreadCreate(osThread(TempSensorsTask), NULL);

  /* definition and creation of HumiditySensor */
  osThreadDef(HumiditySensor, vHumiditySensor, osPriorityNormal, 0, 128);
  HumiditySensorHandle = osThreadCreate(osThread(HumiditySensor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  uint32_t err;
  DataInit();
  while(1)
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    osDelay(200);
    /*
    err = HAL_UART_GetError(&huart1);
    if (USART2->ISR && USART_ISR_ORE)
    {
      USART2->ICR |= USART_ICR_ORECF; 
      USART2->CR1 |= USART_CR1_RXNEIE;
    }
    switch (err)
    {
      case HAL_UART_ERROR_NONE:   break;//!< No error           
      case HAL_UART_ERROR_PE:     break;//!< Parity error      
      case HAL_UART_ERROR_NE:     break;//!< Noise error         
      case HAL_UART_ERROR_FE:     break;//!< frame error         
      case HAL_UART_ERROR_ORE:    USART2->ICR |= USART_ICR_ORECF; USART2->CR1 |= USART_CR1_RXNEIE; break;//!< Overrun error       
      case HAL_UART_ERROR_DMA:    break;//!< DMA transfer error  
      case HAL_UART_ERROR_BUSY:   break;//!< Busy Error         
    }*/
  }
  /* USER CODE END StartDefaultTask */
}

/* vTempSensorsTask function */
void vTempSensorsTask(void const * argument)
{
  /* USER CODE BEGIN vTempSensorsTask */
 //Для расчёта среднего значения:
  //uint32_t      SUMM = 0;
  //uint8_t       counter = 0;
  
  uint8_t address = 0x5A;
  
  //для расчёта CRC-8
  uint8_t crc8;
  uint8_t crcbuf[5];
  crcbuf[0] = (address << 1);
  crcbuf[1] = 0x07;
  crcbuf[2] = (address << 1) + 1;
  
  uint8_t buf[3];
  Multi_I2Cs_Init();//Настройка пинов I2C-шин
  while(1)
  {
    for (int j = 0; j < IntegrSize; j++)
    {
      for (int i = 0; i < 4; i++)
      {
        taskENTER_CRITICAL();
        buf[0] = 0x07; //0х07 - Номер регистра с температурой
        TP.TSens[i].ec = i2csRead(&TP.TSens[i].I2C, buf, address, 3);
        taskEXIT_CRITICAL();
        if (TP.TSens[i].ec == 0)
        {//Если нет ошибок
          crcbuf[3] = buf[0];
          crcbuf[4] = buf[1];
          crc8 = HAL_CRC_Calculate(&hcrc, (uint32_t *)crcbuf, 5);
          if (buf[2] != crc8)
            asm("NOP");
          float T = (float)((buf[1] << 8) + buf[0]);
          TP.TSens[i].Integr[TP.TSens[i].IntegrCounter++] = (uint16_t)(TP.TSens[i].Rate[0] + TP.TSens[i].Rate[1]*T + TP.TSens[i].Rate[2]*T*T);      //Температура с датчика(в попугаях)
          Integr(i);
          TP.TSens[i].T = TP.TSens[i].Tobj*0.02 - 273.15; //Температура в градусах цельсия
        }
        osDelay(1);
      }
    }
    //Расчёт среднего:
    CalcAverage();
    
    TP.StatusReg &= ~(0x0F);
    for (int i = 0; i < 4; i++)
    {//Запись ошибок работы датчиков в статус-регистр
      if (TP.TSens[i].ec != 0)
        TP.StatusReg |= (1 << i);
    }
    if ((GetDeviceMode() == StreamMode) | (GetDeviceMode() == FullStreamMode)) //Если термозонд в режиме потоковой передачи, то слать данные после каждого замера
      SendMeasure();
    osDelay(10);
  }
  /* USER CODE END vTempSensorsTask */
}

/* vHumiditySensor function */
void vHumiditySensor(void const * argument)
{
  /* USER CODE BEGIN vHumiditySensor */
  HAL_COMP_Start_IT(&hcomp2);
  HAL_TIM_Base_Start_IT(&htim6);
  while(1)
  {
    HAL_GPIO_WritePin(HumSens_GPIO_Port, HumSens_Pin, GPIO_PIN_SET);
    osDelay(20);
    HAL_GPIO_WritePin(HumSens_GPIO_Port, HumSens_Pin, GPIO_PIN_RESET);
    osDelay(20);
  }
  /* USER CODE END vHumiditySensor */
}

/* UART_TimerCallback function */
void UART_TimerCallback(void const * argument)
{
  /* USER CODE BEGIN UART_TimerCallback */
  Parser(TP.UART.RxBuf);
  /* USER CODE END UART_TimerCallback */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
