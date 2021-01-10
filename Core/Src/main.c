/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "referee_system.h"
#include "remote_control.h"
#include "motor_control.h"
#include "bmi088.h"
#include "ist8310.h"
#include "PID.h"
#include "INS.h"
#include "music.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t sys_time = 0; 
uint32_t cycle_num = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BMI088_GYRO_INT1_Pin)
	{
		BMI088_read_gyro(); //1000Hz
		
		AHRS_update(bmi088_gyro_calib,bmi088_accel,ist8310_mag); //INS cal
	}
	else if(GPIO_Pin == BMI088_ACCEL_INT1_Pin)
	{
		BMI088_read_accel(); //800Hz
	}
	else if(GPIO_Pin == IST8310_DRDY_Pin)
	{
		IST8310_read_mag(); //200Hz
	}
	else if(GPIO_Pin == KEY_Pin)
	{
		gyro_zero_calib_flag = 1;
	}
}

//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint16_t tmp;
	
	if(htim==&htim3) //500Hz
	{	
		if(robot_ctrl.movement_mode!=MOTOR_DISABLE){set_chassis_motor_speed(robot_motion.chassis_speed);} //底盘电机闭环控速
		else{CAN_cmd_chassis(0,0,0,0);}
	}
	else if(htim==&htim6) //500Hz
	{
		if(robot_ctrl.movement_mode!=MOTOR_DISABLE){set_gimbal_motor_speed(robot_motion.yaw_speed,robot_motion.pitch_speed);} //云台电机闭环控速
		else{CAN_cmd_gimbal(0,0);}
	}
	else if(htim==&htim7) //500Hz
	{
		if(robot_ctrl.movement_mode!=MOTOR_DISABLE){set_ammobooster_speed(robot_motion.shoot_speed,robot_motion.trigger_speed);} //发射机构电机闭环控速
		else{CAN_cmd_ammobooster(0,0,0);}
	}
	else if(htim==&htim11) //200Hz
	{
		if(robot_ctrl.movement_mode!=MOTOR_DISABLE){robot_motion_resolving(&robot_motion,&robot_ctrl);} //运动解算
	}
	else if(htim==&htim9) //2Hz
	{
		BMI088_read_temperature();
		tmp = PIDControl(&pid_gyrotemp, bmi088_temperature*100);
		__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,tmp);
		
		if(ABS(pid_gyrotemp.Error)<200){HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET); HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);}
		else{{HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET); HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);}}
	}
	else if(htim==&htim12) //20ms
	{
		if(buzzer_state==PLAYING_INIT_MUSIC) {PlayingSong(song_robomasterlickdog,13);}
		else if(buzzer_state==PLAYING_WARNING_SOUND) {PlayingSound(sound_warning,6);}
		else if(buzzer_state==PLAYING_ERROR_SOUND) {PlayingSound(sound_error,6);}
		else if(buzzer_state==PLAYING_GYROCALIB_SOUND) {PlayingSound(sound_gyrocalibrating,6);}
		else if(buzzer_state==PLAYING_AUTOAIMING_SOUND) {PlayingSound(sound_autoaiming,6);}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t rc_timeout_count=0;
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
  MX_TIM10_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();
	CAN_cmd_chassis(0,0,0,0);
	CAN_cmd_gimbal(0,0);
	CAN_cmd_ammobooster(0,0,0);
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim12);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); //buzzer
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1); //temperature
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,600); //close
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

	//
	SetBuzzerState(PLAYING_INIT_MUSIC); //播放启动音乐

	InitPID();
	init_vrefint_coefficient(); //校准基准电压
	
	BMI088_init();
	IST8310_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	StartTimer2();
	StartTimer5();
	
	//
  while (1)
  {
		ClearTimer2();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//陀螺仪校准
		if(gyro_zero_calib_flag==1)
		{
			HAL_Delay(1000); //等待稳定
			SetBuzzerState(PLAYING_GYROCALIB_SOUND);
			BMI088_calib_gyro_zero(bmi088_gyro,bmi088_gyro_deg); //标定陀螺仪零点 ~10s
			gyro_zero_calib_flag = 0;
			SetBuzzerState(PLAYING_STOP);
		}
		
		//裁判系统解析
		ParseRefereeSystemData();
		
		//遥控器状态
		if(rc_update_flag==1)
		{
			rc_update_flag=0;
			
			if(rc_timeout_count>=100){SetBuzzerState(PLAYING_STOP);}
			rc_timeout_count=0;
			
			//
		}
		else
		{
			rc_timeout_count++;
			if(rc_timeout_count>=100) //timeout-1s
			{
				//if(rc_timeout_count==1000){SetBuzzerState(PLAYING_WARNING_SOUND);}
				//CAN_cmd_chassis(0,0,0,0);
				//CAN_cmd_gimbal(0,0);
				//CAN_cmd_ammobooster(0,0,0);
			}
		}
		
		//OLED显示
		
		
		//发送上位机数据
		//uint8_t buf[2];
		//buf[0]=motor_status_gimbal[0].delt_angle>>8;
		//buf[1]=motor_status_gimbal[0].delt_angle;
		//HAL_UART_Transmit(&huart6,buf,2,100);
		
		
		
		if(cycle_num%100==0) //1Hz
		{
			get_temprate(); //获取片内温度
			get_battery_voltage(); //获取电源电压

		}
		
		//
		while(ReadTimer2()<10000); //10ms
		sys_time+=ReadTimer2();
		cycle_num++;
  }
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
