/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6000.h"
#include "filter.h"
#include "pid.h"
#include "dshot.h"
#include "CRsF.h"
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "freertos.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_KP_PITCH_INNER		0.8f
#define PID_KD_PITCH_INNER		0.005f
#define PID_KI_PITCH_INNER		0.2f

#define PID_KP_PITCH_OUTER		3.0f
#define PID_KD_PITCH_OUTER		0.0f
#define PID_KI_PITCH_OUTER		0.0f

#define PID_KP_ROLL_INNER		0.8f
#define PID_KD_ROLL_INNER		0.005f
#define PID_KI_ROLL_INNER		0.2f

#define PID_KP_ROLL_OUTER		3.0f
#define PID_KD_ROLL_OUTER		0.0f
#define PID_KI_ROLL_OUTER		0.0f


#define PID_KP_YAW				3.5f
#define PID_KD_YAW				0.0f
#define PID_KI_YAW				0.0f

#define PID_KP_YAW_RATE			3.5f
#define PID_KD_YAW_RATE			0.0f
#define PID_KI_YAW_RATE			0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern osThreadId sensor_fusion_tHandle;
extern osThreadId control_taskHandle;
extern osThreadId rc_input_taskHandle;
extern osThreadId telemetry_taskHandle;

extern IIR_Filter_3D gyro_filtered;
extern Double_PID_Controller PID_Controller_Roll, PID_Controller_Pitch;
extern PID_Controller PID_Controller_Yaw, PID_Controller_Yaw_Rate;


extern MPU6000 mpu;

extern uint32_t DShot_MemoryBufferMotor1[MEM_BUFFER_LENGTH];
extern uint32_t DShot_DMABufferMotor1[DMA_BUFFER_LENGTH];
extern uint32_t DShot_MemoryBufferMotor2[MEM_BUFFER_LENGTH];
extern uint32_t DShot_DMABufferMotor2[DMA_BUFFER_LENGTH];
extern uint32_t DShot_MemoryBufferMotor3[MEM_BUFFER_LENGTH];
extern uint32_t DShot_DMABufferMotor3[DMA_BUFFER_LENGTH];
extern uint32_t DShot_MemoryBufferMotor4[MEM_BUFFER_LENGTH];
extern uint32_t DShot_DMABufferMotor4[DMA_BUFFER_LENGTH];




bool motor_armed;

uint8_t uart_rx_buffer[64];
CrsF_Frame receive_frame;
volatile bool uart_ready = false;


float ScaledControllerOutput[5];

#define CRSF_DMA_BUF_SIZE 128
extern uint8_t crsf_dma_buf[CRSF_DMA_BUF_SIZE];
uint16_t old_pos = 0;

bool PID_outer_loop_activation_flag;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	vTaskNotifyGiveFromISR(sensor_fusion_tHandle, &xHigherPriorityTaskWoken);

    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//Hàm call back gửi DSHOT cho động cơ
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM5)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            memcpy(DShot_DMABufferMotor2, DShot_MemoryBufferMotor2,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor2[0]));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            memcpy(DShot_DMABufferMotor3, DShot_MemoryBufferMotor3,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor3[0]));
        }
    }

    else if (htim->Instance == TIM3){
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            memcpy(DShot_DMABufferMotor1, DShot_MemoryBufferMotor1,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor1[0]));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            memcpy(DShot_DMABufferMotor4, DShot_MemoryBufferMotor4,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor4[0]));
        }
    }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_pitch(float Ax, float Ay, float Az) {
    return atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / M_PI;
}

float get_roll(float Ax, float Az) {
    return atan2f(-Ax, Az) * 180.0f / M_PI;
}

void init_PIDs(void)
{
//    PID_Init(&pid_roll,  1.5f, 0.0f, 0.05f, 400.0f, 100.0f);
//    PID_Init(&pid_pitch, 1.5f, 0.0f, 0.05f, 400.0f, 100.0f);
//    PID_Init(&pid_yaw,   2.0f, 0.0f, 0.10f, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Pitch.inner_loop, PID_KP_PITCH_INNER, PID_KI_PITCH_INNER, PID_KD_PITCH_INNER, 200.0f, 100.0f);
	PID_Init(&PID_Controller_Roll.inner_loop, PID_KP_ROLL_INNER, PID_KI_ROLL_INNER, PID_KD_ROLL_INNER, 200.0f, 100.0f);
	PID_Init(&PID_Controller_Yaw, PID_KP_YAW, PID_KI_YAW, PID_KD_YAW, 400.0f, 100.0f);

	PID_Init(&PID_Controller_Pitch.outer_loop, PID_KP_PITCH_OUTER, PID_KI_PITCH_OUTER, PID_KD_PITCH_OUTER, 200.0f, 50.0f);
	PID_Init(&PID_Controller_Roll.outer_loop, PID_KP_ROLL_OUTER, PID_KI_ROLL_OUTER, PID_KD_ROLL_OUTER, 200.0f, 50.0f);
	PID_Init(&PID_Controller_Yaw_Rate, PID_KP_YAW_RATE, PID_KI_YAW_RATE, PID_KD_YAW_RATE, 200.0f, 50.0f);

}

void CRSF_IdleHandler(void) {
    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
    uint16_t new_pos = CRSF_DMA_BUF_SIZE - dma_remaining;

    if(new_pos >= old_pos) {
        for(uint16_t i = old_pos; i < new_pos; i++) {
            if(Check_Status(crsf_dma_buf[i], &receive_frame)) {
                CRsF_Process(&receive_frame);
            }
        }
    } else {
        for(uint16_t i = old_pos; i < CRSF_DMA_BUF_SIZE; i++) {
            if(Check_Status(crsf_dma_buf[i], &receive_frame)) {
                CRsF_Process(&receive_frame);
            }
        }
        for(uint16_t i = 0; i < new_pos; i++) {
            if(Check_Status(crsf_dma_buf[i], &receive_frame)) {
                CRsF_Process(&receive_frame);
            }
        }
    }

    old_pos = new_pos;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //IIR_Filter_3D_Init(&acc_filtered, IIR_ACC_ALPHA, IIR_ACC_BETA);
  IIR_Filter_3D_Init(&gyro_filtered, IIR_GYR_ALPHA, IIR_GYR_BETA);
  MPU6000_Init(&mpu, &hspi1);

  mpu.state=0;
  for(int i=0;i<=14;i++) mpu.tx_buffer[i]=0xFF;
  init_PIDs();
  MPU6000_Calibrate(&mpu);

  Dshot_DMABuffer_init(DShot_DMABufferMotor1);
  Dshot_DMABuffer_init(DShot_DMABufferMotor2);
  Dshot_DMABuffer_init(DShot_DMABufferMotor3);
  Dshot_DMABuffer_init(DShot_DMABufferMotor4);

  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor1);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor2);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor3);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor4);

  Dshot_Calibrate(DShot_DMABufferMotor1);
  Dshot_Calibrate(DShot_DMABufferMotor2);
  Dshot_Calibrate(DShot_DMABufferMotor3);
  Dshot_Calibrate(DShot_DMABufferMotor4);

  //HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, DShot_DMABufferMotor1, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, DShot_DMABufferMotor1, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, DShot_DMABufferMotor2, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, DShot_DMABufferMotor3, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, DShot_DMABufferMotor4, DMA_BUFFER_LENGTH);
  motor_armed = true;

  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor1);
  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor2);
  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor3);
  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor4);

  HAL_Delay(3000);

  //HAL_UART_Receive_DMA(&huart6, crsf_dma_buf, CRSF_DMA_BUF_SIZE);
  //__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  // enable IDLE interrupt
  //HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();
  HAL_UART_Receive_DMA(&huart6, crsf_dma_buf, CRSF_DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  // enable IDLE interrupt
  HAL_TIM_Base_Start_IT(&htim2);
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 // }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2)
   { // <--- CHỈ KIỂM TRA TIM2
	   if (mpu.state == 0) {
	   mpu.state = 1; // request new transfer
	   MPU6000_Start_DMA(&mpu);
     }
   }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
