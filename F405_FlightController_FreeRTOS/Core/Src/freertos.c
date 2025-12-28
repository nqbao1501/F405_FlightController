/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
#include "ekf.h"

#define CRSF_DMA_BUF_SIZE 128
#define EKF_MODE 	0
#define COMP_MODE	1
#define SENSOR_FUSION_MODE	EKF_MODE
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6000 mpu;

TIM_HandleTypeDef *htim;

float rollHat_acc_rad;
float pitchHat_acc_rad;
IIR_Filter_3D acc_filtered;
IIR_Filter_3D gyro_filtered;

float dt = 0.001f;
float acc_x = 0.0f;
float acc_y = 0.0f;
float acc_z = 0.0f;
float gyro_p = 0.0f;
float gyro_q = 0.0f;
float gyro_r = 0.0f;

float pitch,roll,yaw,yawHat_acc_rad;
float roll_rad, pitch_rad, yaw_rad;
uint32_t global_counter = 0;
float rollHat_acc_rad;
float pitchHat_acc_rad;
float rollDot;
float pitchDot;
float roll_out;
float pitch_out;
float yaw_out;
float yawDot;
Double_PID_Controller PID_Controller_Roll, PID_Controller_Pitch;
PID_Controller PID_Controller_Yaw, PID_Controller_Yaw_Rate;
uint16_t throttle = 1000;

float m1 = 0;
float m2 = 0;
float m3 = 0;
float m4 = 0;

float  yaw_heading_reference = 0;

uint8_t crsf_dma_buf[CRSF_DMA_BUF_SIZE];
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;

uint32_t DShot_MemoryBufferMotor1[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor1[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor2[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor2[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor3[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor3[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor4[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor4[DMA_BUFFER_LENGTH] = {0};

char  USB_TX_Buffer[128];
float pitch_target;
float roll_target;

EKF_t ekf;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static inline float constrain(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    else if (value > max_val) return max_val;
    else return value;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId sensor_fusion_tHandle;
osThreadId control_taskHandle;
osThreadId rc_input_taskHandle;
osThreadId telemetry_taskHandle;
osSemaphoreId rc_ready_semHandleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void sensor_fusion_func(void const * argument);
void control_func(void const * argument);
void rc_input_func(void const * argument);
void telemetry_func(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of rc_ready_semHandle */
  osSemaphoreDef(rc_ready_semHandle);
  rc_ready_semHandleHandle = osSemaphoreCreate(osSemaphore(rc_ready_semHandle), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of sensor_fusion_t */
  osThreadDef(sensor_fusion_t, sensor_fusion_func, osPriorityRealtime, 0, 512);
  sensor_fusion_tHandle = osThreadCreate(osThread(sensor_fusion_t), NULL);

  /* definition and creation of control_task */
  osThreadDef(control_task, control_func, osPriorityHigh, 0, 512);
  control_taskHandle = osThreadCreate(osThread(control_task), NULL);

  /* definition and creation of rc_input_task */
  osThreadDef(rc_input_task, rc_input_func, osPriorityHigh, 0, 256);
  rc_input_taskHandle = osThreadCreate(osThread(rc_input_task), NULL);

  /* definition and creation of telemetry_task */
  osThreadDef(telemetry_task, telemetry_func, osPriorityLow, 0, 384);
  telemetry_taskHandle = osThreadCreate(osThread(telemetry_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_sensor_fusion_func */
/**
  * @brief  Function implementing the sensor_fusion_t thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_sensor_fusion_func */
void sensor_fusion_func(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN sensor_fusion_func */

  /* Infinite loop */
  for(;;)
  {
	  #if SENSOR_FUSION_MODE ==  COMP_MODE
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
	  MPU6000_Process_DMA(&mpu);
	  mpu.state = 0;

	  acc_x = mpu.acc[0];
	  acc_y = mpu.acc[1];
	  acc_z = mpu.acc[2];

	  IIR_Filter_3D_Update(&gyro_filtered, mpu.gyro[0], mpu.gyro[1], mpu.gyro[2], &gyro_p, &gyro_q, &gyro_r);
	  /* Ước tính góc (Complementary filter) */
	  rollHat_acc_rad = atan2f(acc_y, acc_z);
	  pitchHat_acc_rad = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));
	  float rollDot_rad = (gyro_p * (M_PI / 180.0f) + tanf(pitchHat_acc_rad) * sinf(rollHat_acc_rad) * gyro_q * (M_PI / 180.0f) + tanf(pitchHat_acc_rad) * cosf(rollHat_acc_rad) * gyro_r * (M_PI / 180.0f));
	  float pitchDot_rad = (cosf(rollHat_acc_rad) * gyro_q * (M_PI / 180.0f) - sinf(rollHat_acc_rad) * gyro_r * (M_PI / 180.0f));

	  roll_rad = (1.0f - COMP_ALPHA) * rollHat_acc_rad + COMP_ALPHA * (roll_rad + rollDot_rad * dt );
	  pitch_rad = (1.0f - COMP_ALPHA) * pitchHat_acc_rad + COMP_ALPHA * (pitch_rad + pitchDot_rad * dt );
	  yawDot = gyro_r;

	  rollDot = rollDot_rad * (180.0f / M_PI);
	  pitchDot = pitchDot_rad * (180.0f / M_PI);
	  roll = roll_rad * (180.0f / M_PI);
	  pitch = pitch_rad * (180.0f / M_PI);
	  yaw = yaw + yawDot * dt;
	  while (yaw>= 360.0f) yaw -= 360.0f;
	  while (yaw < 0.0f)         yaw += 360.0f;

	  global_counter++;
	  PID_outer_loop_activation_flag = (global_counter % 4 == 0);
	  xTaskNotifyGive(control_taskHandle);

	  #else
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
	  MPU6000_Process_DMA(&mpu);
	  mpu.state = 0;

	  acc_x = mpu.acc[0];
	  acc_y = mpu.acc[1];
	  acc_z = mpu.acc[2];
	  mpu.gyro[0] += mpu.gyro_offset[0];
	  mpu.gyro[1] += mpu.gyro_offset[1];

	  IIR_Filter_3D_Update(&gyro_filtered, mpu.gyro[0], mpu.gyro[1], mpu.gyro[2], &gyro_p, &gyro_q, &gyro_r);
	  float gyro_rad_s[3];
	  gyro_rad_s[0] = gyro_p * M_PI / 180.0f;
	  gyro_rad_s[1] = gyro_q * M_PI / 180.0f;
	  gyro_rad_s[2] = gyro_r * M_PI / 180.0f;

	  float acc_m_s2[3];
	  acc_m_s2[0] = acc_x * 9.81f;
	  acc_m_s2[1] = acc_y * 9.81f;
	  acc_m_s2[2] = acc_z * 9.81f;

	  EKF_Predict(&ekf, gyro_rad_s, dt);
	  EKF_UpdateAccel(&ekf, acc_m_s2);
	  yawDot = gyro_r;

	  quad_to_euler(&ekf, &roll_rad, &pitch_rad);
	  roll = roll_rad * (180.0f / M_PI);
	  pitch = pitch_rad * (180.0f / M_PI);
	  yaw = yaw + yawDot * dt;
	  while (yaw>= 360.0f) yaw -= 360.0f;
	  while (yaw < 0.0f)         yaw += 360.0f;

	  global_counter++;
	  PID_outer_loop_activation_flag = (global_counter % 4 == 0);
	  xTaskNotifyGive(control_taskHandle);
	  #endif

  }
  /* USER CODE END sensor_fusion_func */
}

/* USER CODE BEGIN Header_control_func */
/**
* @brief Function implementing the control_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_func */
void control_func(void const * argument)
{
  /* USER CODE BEGIN control_func */

  /* Infinite loop */
  for(;;)
  {
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		roll_target = (ScaledControllerOutput[CH_ROLL]- 1500.0f) * 0.08f;
		pitch_target = (ScaledControllerOutput[CH_PITCH]- 1500.0f) * -0.08f;
		roll_out = PID_Double_Calculation(&PID_Controller_Roll, roll_target, roll, rollDot, dt);
		pitch_out = PID_Double_Calculation(&PID_Controller_Pitch, pitch_target, pitch, pitchDot, dt);

		if (ScaledControllerOutput[CH_YAW] < 1485 || ScaledControllerOutput[CH_YAW] > 1515){
			yaw_heading_reference = yaw;
			yaw_out = PID_Yaw_Rate_Calculation(&PID_Controller_Yaw_Rate, (ScaledControllerOutput[CH_YAW] - 1500.0f) * 0.08f , yawDot, dt);
		}
		else{
			yaw_out = PID_Yaw_Angle_Calculation(&PID_Controller_Yaw, yaw_heading_reference , yaw, yawDot, dt);
		}

		// 3. === LOGIC MOTOR MIX (CŨNG TỪ MAIN.C) ===
		m1 = 100 + ScaledControllerOutput[CH_THROTTLE] - pitch_out - roll_out + yaw_out;
		m2 = 100 + ScaledControllerOutput[CH_THROTTLE] + pitch_out - roll_out - yaw_out;
		m3 = 100 + ScaledControllerOutput[CH_THROTTLE] - pitch_out + roll_out - yaw_out;
		m4 = 100 + ScaledControllerOutput[CH_THROTTLE] + pitch_out + roll_out + yaw_out;

		// 4. === LOGIC CLAMP (CŨNG TỪ MAIN.C) ===
		m1 = constrain(m1, 0, 1999);
		m2 = constrain(m2, 0, 1999);
		m3 = constrain(m3, 0, 1999);
		m4 = constrain(m4, 0, 1999);

		// 5. === CHUẨN BỊ DSHOT (TỪ MAIN.C WHILE(1)) ===
		// Logic này chuẩn bị dữ liệu cho ngắt DShot DMA sử dụng
		if (ScaledControllerOutput[CH_ARM] < 1500){
			Dshot_PrepareFrame(0, DShot_MemoryBufferMotor1);
			Dshot_PrepareFrame(0, DShot_MemoryBufferMotor2);
			Dshot_PrepareFrame(0, DShot_MemoryBufferMotor3);
			Dshot_PrepareFrame(0, DShot_MemoryBufferMotor4);
		}
		else{
			Dshot_PrepareFrame(m1, DShot_MemoryBufferMotor1);
			Dshot_PrepareFrame(m2, DShot_MemoryBufferMotor2);
			Dshot_PrepareFrame(m3, DShot_MemoryBufferMotor3);
			Dshot_PrepareFrame(m4, DShot_MemoryBufferMotor4);
		}

  }
  /* USER CODE END control_func */
}

/* USER CODE BEGIN Header_rc_input_func */
/**
* @brief Function implementing the rc_input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc_input_func */
void rc_input_func(void const * argument)
{
  /* USER CODE BEGIN rc_input_func */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(rc_ready_semHandleHandle, 100) == osOK)
	  {
	     CRSF_IdleHandler();

	  }
  }
  /* USER CODE END rc_input_func */
}

/* USER CODE BEGIN Header_telemetry_func */
/**
* @brief Function implementing the telemetry_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_telemetry_func */
void telemetry_func(void const * argument)
{
  /* USER CODE BEGIN telemetry_func */
	 uint8_t tx_buf[32];
	 TickType_t xLastWakeTime = xTaskGetTickCount();
	 uint8_t counter = 0;
  /* Infinite loop */
  for(;;)
  {
	  float yaw_rad_val = yaw * 0.01745329f;
	  CRsF_Pack_Attitude(tx_buf, pitch_rad, roll_rad, yaw_rad_val);
	  HAL_UART_Transmit_DMA(&huart3, tx_buf, 10);
	  counter++;
	  if(counter >= 10) {
	     counter = 0;
	     osDelay(10); // Né xung đột DMA

	            // Gửi Battery
	     CRsF_Pack_Battery(tx_buf);
	     HAL_UART_Transmit_DMA(&huart3, tx_buf, 12);

	     osDelay(10);
	     // Gửi Flight Mode
	     const char* mode = ScaledControllerOutput[CH_ARM] > 1500 ? "ARMED" : "STAB";
	     CRsF_Pack_FlightMode(tx_buf, mode);
	     // Tính độ dài gói tin Flight Mode
	     uint8_t len = tx_buf[1] + 2;
	     HAL_UART_Transmit_DMA(&huart3, tx_buf, len);
	  }

	  osDelayUntil(&xLastWakeTime, 100);
  }
  /* USER CODE END telemetry_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
