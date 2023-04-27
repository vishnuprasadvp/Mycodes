/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Tuya_Reset();
void Touch_Read(void);
void Uart_Read_data(void);
void Status_ReadData(void);
void Stop_Indication(void);
void switch_long_press(void);
void Rx_Data_evaluation(void);
void set_max_pwm(uint8_t max_val);
void set_min_pwm(uint8_t min_val);
void switch_operation(uint8_t SW_NO, uint8_t SW_STS);
void Tx_Switch_Satus(uint8_t SW_NO, uint8_t SW_STS);
void Master_Switch(uint8_t SW_STS);
void set_max_pwm(uint8_t max_val);
void Indication(void);
void EEPROM_initial(void);
void dimmer_operation(uint8_t Status,uint8_t level);
void DIMMER_TX(uint8_t DIMMER_COUNTER);
void Adc_read();
void adc_clear();
void Serial_out();
void adc_initial(void);
int  read_adc(int channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/*************************SWITCH_READ_PARAMETERS****************/

/********SWITCH_ON AND OFF************/

#define SW_ON          1
#define SW_OFF         0

/*******SWITCH_NUMBER***********/

#define  NUMBER_OF_SWITCHES  7

#define Switch_1       1
#define Switch_2       2
#define Switch_3       3
#define Switch_4       4
#define Switch_5       5
#define Switch_6       6
/*******DIMMMER SWITCHES***********/
#define Switch_7       7
#define DIMMER_1	   0x6a
#define DIMMER_2	   0x6a
/*********************DIMMER PARAMETERS*******************/
uint8_t dimmer_level=0;
uint8_t dimmer_flag = 0;
uint8_t timer2_period = 1;
uint8_t dimmer_value_change_flag = 0 ;
uint8_t dimmer_enable_flag = 0;
uint8_t Touch_Flag[9] = {0};

/*************************UART_READ_PARAMETERS****************/

#define  BUFFERSIZE        256
uint8_t  wrI = 0;                                                  // Write Index
uint8_t  rdI = 0;                                                  // Read Index
uint8_t  Count = 0;
uint8_t  Cyclic_Buffer[BUFFERSIZE] = {0};                          // Cyclic Data Buffer

// This Variables are used for detecting Start Bytes of UART Data

uint8_t  Rx_First_byte = 0;                                       // Used for temporary Data Byte Storing UART
uint8_t  Rx_Second_byte = 0;                                      // Used for temporary Data Byte Storing UART
uint8_t  RX_Data[100] = {0};                                      // Array to store UART Receive Data
uint8_t  RX_data_Count = 0;                                      // Counter used to count UART data Byte by Byte
uint8_t  RX_Buffer = 0;                                          // One Byte Buffer used to read UART Data
uint8_t  RX_flag_checking =0;			                         //checking flag
uint8_t  RX_data_length=0;										//find the length of incoming data
uint8_t  RX_Data_Start_Flag = 0;                                 // UART Start Byte Indication Flag
uint8_t  RX_Complete_flag = 0;                                   // Flag used to indicate completion of data reception

/*************************RX_data_evalution_PARAMETERS****************/

uint8_t  Rx_CNT = 0;                                             // Heartbeat Packet Distinction
uint32_t timer_val=0;
uint8_t  Tuya_Heart_Flag = 0;
uint8_t  Tuya_Reset_Flag = 0;									//Reseting mcu
uint32_t Tuya_Start_Time = 0;
uint8_t  Tuya_flag = 0;
uint8_t  Idication_Flag = 0;
uint8_t  Idication_Control_Flag = 0;
                //EEPROM ADDRESS MEMORY START
#define  EEPROM_ADD       0xA0
uint8_t  MIN_PWM = 20;
uint8_t  MAX_PWM = 100;
uint8_t  ZERO_PWM = 0;
uint8_t  Temp_var = 0;
/***********************PWM_INDICATION*********************/
#define  Delay              50
#define  Z_PWM               0                                   //PWM 0 for complete off
#define  L_PWM			    20                                   //PWM for indication LED (Low)
#define  H_PWM             100                                   //PWM for indication LED (High)
#define  MAX_SAMPLE       5000

uint8_t  PID[50] = {0x55,0xAA,0x03,0x01,0x00,0x2A,0x7B,0x22,0x70,0x22,0x3A,0x22,0x77,0x70,0x70,0x77,0x77,0x67,0x6E,0x35,0x65,0x76,0x6F,0x72,0x74,0x75,0x70,0x39,0x22,0x2C,0x22,0x76,0x22,0x3A,0x22,0x31,0x2E,0x30,0x2E,0x30,0x22,0x2C,0x22,0x6D,0x22,0x3A,0x30,0x7D,0x8C};
uint8_t  Work_Mode[7] = {0x55,0xAA,0x03,0x02,0x00,0x00,0x04};                 // Querying the Module Working Mode set by the MCU
uint8_t  WiFi_Status[7] = {0x55,0xAA,0x03,0x03,0x00,0x00,0x05};              // Reporting WiFi Status
uint8_t  Longpress_Tx_Buff[10]={0x55,0xAA,0x03,0x04,0x00,0x00,0x06};        /*****************************long press comment*******************/
uint8_t  Hbt[8] = {0x55,0xAA,0x03,0x00,0x00,0x01,0x00,0x03};               // First Heartbeat Response
uint8_t  Hbt1[8] = {0x55,0xAA,0x03,0x00,0x00,0x01,0x01,0x04};             // Second Heartbeat Response
/*****************************DIMMER PARAMETERS***********************************/


/***********************A D C****************************/
/***********************ADC*****************/
uint32_t ADCValue1_0 = 0;
uint32_t ADCValue1_1 = 0;
uint32_t ADCValue1_2 = 0;
uint32_t ADCValue1_3 = 0;
uint32_t ADCValue1_4 = 0;
uint32_t ADCValue1_5 = 0;
uint32_t ADCValue1_6 = 0;
uint32_t ADCValue1_7 = 0;

uint32_t adc0_init = 0;
uint32_t adc1_init = 0;
uint32_t adc2_init = 0;
uint32_t adc3_init = 0;
uint32_t adc4_init = 0;
uint32_t adc5_init = 0;
uint32_t adc6_init = 0;
uint32_t adc7_init = 0;
uint32_t ADC_Tempvalue[8]= {0};
uint16_t ADC_Counter = 0;
uint32_t ADC_Start_Time = 0;

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // PWM start for Switch Status indication LEDs

  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                      //LED0
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);                      //LED1
  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);						 //LED2
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);                      //LED3
  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);						 //LED4
  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);                      //LED5
  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);					     //LED6
  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                      //DIMMER_UP
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                      //DIMMER_DOWN
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);                      //indication_LED

      //INITIAL STATE PWM ON WITH MINIMUM POWER ALL SWITCH

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,MIN_PWM);          //LED0
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,MIN_PWM);          //LED1
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,MIN_PWM);          //LED2
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,MIN_PWM);          //LED3
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,MIN_PWM);          //LED4
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,MIN_PWM);          //LED5
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,MIN_PWM);          //LED6
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,ZERO_PWM);          //DIMMER_UP
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,ZERO_PWM);          //DIMMER_DOWN
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,Z_PWM);            //indication

        EEPROM_initial();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	       HAL_UART_Receive_IT(&huart2, &RX_Buffer,1);
	  	   Touch_Read();
	  	   Uart_Read_data();
	  	   Rx_Data_evaluation();
	  	   Indication();
	  	   Adc_read();
	  	   if( Tuya_flag == 1)
	  	   {
	  		   uint32_t Now = HAL_GetTick();
	  		   if(Tuya_Heart_Flag == 1)
	  		   {
	  			   Tuya_Heart_Flag = 0;
	  		   }
	  		   else
	  		   {
	  			   if((Now - Tuya_Start_Time) > 45000)
	  			   {
	  				   Tuya_Start_Time = Now;
	  				   Tuya_Reset_Flag = 1;
	  				   Tuya_Reset();
	  			   }
	  		   }
	  	    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
///////////STM32RESTART//////////////
void Adc_read()
{
      uint16_t Adc_read = 0;

/***********************************************************************************************************************/
      //S1
	  Adc_read = read_adc(1);
	  ADC_Tempvalue[0] += Adc_read;
	  if(Touch_Flag[0] == 2)
	  {
		 Adc_read = read_adc(1);
		 if(Adc_read >= adc0_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		 {
		     ADCValue1_0 = Adc_read - adc0_init + ADCValue1_0;
		 }

		 if(Adc_read < adc0_init)
		 {
			 ADCValue1_0 = adc0_init - Adc_read + ADCValue1_0;
		 }
	  }
/***********************************************************************************************************************/
	  //S2
	  Adc_read = read_adc(2);
	  ADC_Tempvalue[1] += Adc_read;
	  if(Touch_Flag[1] == 2)
	  {
	 	  Adc_read = read_adc(2);
		  if(Adc_read >= adc1_init)             				     // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_1 = Adc_read - adc1_init + ADCValue1_1;
		  }

		  if(Adc_read < adc1_init)
		  {
			  ADCValue1_1 = adc1_init - Adc_read + ADCValue1_1;
		  }
	   }
/***********************************************************************************************************************/
	 //S3
	 Adc_read = read_adc(3);
	 ADC_Tempvalue[2] += Adc_read;
	 if(Touch_Flag[2] == 2)
	 {
		 Adc_read = read_adc(3);
		 if(Adc_read >= adc2_init)             					 // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		 {
			 ADCValue1_2 =Adc_read - adc2_init + ADCValue1_2;
		 }

		 if(Adc_read < adc2_init)
		 {
			 ADCValue1_2 = adc2_init - Adc_read + ADCValue1_2;
		 }
	  }
/***********************************************************************************************************************/
	  //S4
	  Adc_read = read_adc(4);
	  ADC_Tempvalue[3] += Adc_read;
	  if(Touch_Flag[3] == 2)
	  {
		  Adc_read = read_adc(4);
		  if(Adc_read >= adc3_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_3 = Adc_read - adc3_init + ADCValue1_3;
		  }

		  if(Adc_read < adc3_init)
		  {
			  ADCValue1_3 = adc3_init - Adc_read + ADCValue1_3;
		  }
	   }
/***********************************************************************************************************************/
	  //S5
	  Adc_read = read_adc(5);
	  ADC_Tempvalue[4] += Adc_read;
	  if(Touch_Flag[4] == 2)
	  {
		  Adc_read = read_adc(5);
		  if(Adc_read >= adc4_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_4 = Adc_read - adc4_init + ADCValue1_4;
		  }

		  if(Adc_read < adc4_init)
		  {
			  ADCValue1_4 = adc4_init - Adc_read + ADCValue1_4;
		  }
	   }
/***********************************************************************************************************************/
	  //S6
	  Adc_read = read_adc(6);
      ADC_Tempvalue[5] += Adc_read;
	  if(Touch_Flag[5] == 2)
	  {
	      Adc_read = read_adc(6);
		  if(Adc_read >= adc5_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_5 = Adc_read - adc5_init + ADCValue1_5;
		  }

		  if(Adc_read < adc5_init)
		  {
			  ADCValue1_5 = adc5_init - Adc_read + ADCValue1_5;
		  }
	   }
/***********************************************************************************************************************/
	  //S7
	  Adc_read = read_adc(7);
	  ADC_Tempvalue[6] += Adc_read;
	  if(Touch_Flag[6] == 2)
	  {
		  Adc_read = read_adc(7);
		  if(Adc_read >= adc6_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_6 = Adc_read - adc6_init + ADCValue1_6;
		  }

		  if(Adc_read<adc6_init)
		  {
			  ADCValue1_6 = adc6_init - Adc_read + ADCValue1_6;
		  }
	   }

/***********************************************************************************************************************/
	  //S8
	  Adc_read = read_adc(8);
	  ADC_Tempvalue[7] += Adc_read;
	  if(Touch_Flag[7] == 2)
	  {
		  Adc_read = read_adc(8);
		  if(Adc_read >= adc7_init)             					  // CHECKING WITH INITIAL VALUE IF INITIAL VALUE IS GREATER THAN PRESENT VALUE
		  {
			  ADCValue1_7 = Adc_read - adc7_init + ADCValue1_7;
		  }

		  if(Adc_read < adc7_init)
		  {
			  ADCValue1_7 = adc7_init - Adc_read + ADCValue1_7;
		  }
	   }
/**********************************************************************************************************************/
	   ADC_Counter = ADC_Counter + 1;

       if(ADC_Counter == MAX_SAMPLE)
       {
				   /************ TAKING THE AVG *****************/

          float temp_ADC1_0 = 0;
	      float temp_ADC1_1 = 0;
	      float temp_ADC1_2 = 0;
	      float temp_ADC1_3 = 0;
	      float temp_ADC1_4 = 0;
	      float temp_ADC1_5 = 0;
	      float temp_ADC1_6 = 0;
	      float temp_ADC1_7 = 0;

		  ADCValue1_0 = ADCValue1_0/MAX_SAMPLE;
		  ADCValue1_1 = ADCValue1_1/MAX_SAMPLE;
		  ADCValue1_2 = ADCValue1_2/MAX_SAMPLE;
		  ADCValue1_3 = ADCValue1_3/MAX_SAMPLE;
		  ADCValue1_4 = ADCValue1_4/MAX_SAMPLE;
		  ADCValue1_5 = ADCValue1_5/MAX_SAMPLE;
		  ADCValue1_6 = ADCValue1_6/MAX_SAMPLE;
		  ADCValue1_7 = ADCValue1_7/MAX_SAMPLE;

     /***********************************************/
		  temp_ADC1_0 = ADCValue1_0;
		  temp_ADC1_1 = ADCValue1_1;
		  temp_ADC1_2 = ADCValue1_2;
		  temp_ADC1_3 = ADCValue1_3;
		  temp_ADC1_4 = ADCValue1_4;
		  temp_ADC1_5 = ADCValue1_5;
		  temp_ADC1_6 = ADCValue1_6;
		  temp_ADC1_7 = ADCValue1_7;

		  ADCValue1_0 = (temp_ADC1_0*0.0079+0.0049)*230;
		  ADCValue1_1 = (temp_ADC1_1*0.0079+0.0049)*230;
		  ADCValue1_2 = (temp_ADC1_2*0.0079+0.0049)*230;
		  ADCValue1_3 = (temp_ADC1_3*0.0079+0.0049)*230;
		  ADCValue1_4 = (temp_ADC1_4*0.0079+0.0049)*230;
		  ADCValue1_5 = (temp_ADC1_5*0.0079+0.0049)*230;
		  ADCValue1_6 = (temp_ADC1_6*0.0079+0.0049)*230;
		  ADCValue1_7 = (temp_ADC1_7*0.0141-0.0553)*230;

		  uint32_t Now = HAL_GetTick();

		  if(Now - ADC_Start_Time >= 3000)
          {
               ADC_Start_Time =Now;
//               if( Tuya_flag == 1)
//               {
			   Serial_out();
//               }
               adc_clear();                                      // CLEARING THE ADC VALUES AFTER TAKING THE SAMPLES
           }

           adc0_init = ADC_Tempvalue[0]/MAX_SAMPLE;
		   adc1_init = ADC_Tempvalue[1]/MAX_SAMPLE;
		   adc2_init = ADC_Tempvalue[2]/MAX_SAMPLE;
		   adc3_init = ADC_Tempvalue[3]/MAX_SAMPLE;
		   adc4_init = ADC_Tempvalue[4]/MAX_SAMPLE;
		   adc5_init = ADC_Tempvalue[5]/MAX_SAMPLE;
		   adc6_init = ADC_Tempvalue[6]/MAX_SAMPLE;
		   adc7_init = ADC_Tempvalue[7]/MAX_SAMPLE;

/***************************************************************************************************/
		   ADC_Counter = 0;
		   for(int i = 0; i < 8; i++)
		   {
			    ADC_Tempvalue[i] = 0;
		   }

  /* USER CODE END 5 */

       }

}

/***********************************************************/
void adc_initial(void)
{
	  for(int i = 1; i <= 1000; i++)
	  {
		   adc0_init += read_adc(1);
		   adc1_init += read_adc(2);
		   adc2_init += read_adc(3);
		   adc3_init += read_adc(4);
		   adc4_init += read_adc(5);
		   adc5_init += read_adc(6);
		   adc6_init += read_adc(7);
		   adc7_init += read_adc(8);
	   }

	   adc0_init = adc0_init/MAX_SAMPLE;
	   adc1_init = adc1_init/MAX_SAMPLE;
	   adc2_init = adc2_init/MAX_SAMPLE;
	   adc3_init = adc3_init/MAX_SAMPLE;
	   adc4_init = adc4_init/MAX_SAMPLE;
	   adc5_init = adc5_init/MAX_SAMPLE;
	   adc6_init = adc6_init/MAX_SAMPLE;
	   adc7_init = adc7_init/MAX_SAMPLE;
}

/**********************************************
 * CLEARING ADC VALUES AFTER TAKING THE SAMPLES
 **********************************************/
void adc_clear()
{
	   ADCValue1_0 = 0;
	   ADCValue1_1 = 0;
	   ADCValue1_2 = 0;
	   ADCValue1_3 = 0;
	   ADCValue1_4 = 0;
	   ADCValue1_5 = 0;
	   ADCValue1_6 = 0;
	   ADCValue1_7 = 0;

	   adc0_init = 0;
	   adc1_init = 0;
	   adc2_init = 0;
	   adc3_init = 0;
	   adc4_init = 0;
	   adc5_init = 0;
	   adc6_init = 0;
	   adc7_init = 0;
}
/*****************************************************************
 * READING ADC VALUE OF EACH CHANNEL ON DEMAND
 * Configuring each channel and using polling
 *****************************************************************/

int read_adc(int channel)
{
	   ADC_ChannelConfTypeDef sConfig = {0};
	   int adc = 0;
	   switch(channel)
	   {
		   case 0:
			     sConfig.Channel = ADC_CHANNEL_0;
		   break;

		   case 1:
			     sConfig.Channel = ADC_CHANNEL_1;
		   break;

		   case 2:
			     sConfig.Channel = ADC_CHANNEL_2;
		   break;

		   case 3:
			     sConfig.Channel = ADC_CHANNEL_3;
		   break;

		   case 4:
			     sConfig.Channel = ADC_CHANNEL_4;
		   break;

		   case 5:
			     sConfig.Channel = ADC_CHANNEL_5;
		   break;

		   case 6:
			     sConfig.Channel = ADC_CHANNEL_7;
		   break;

		   case 7:
			     sConfig.Channel = ADC_CHANNEL_6;
		   break;

		   case 8:
			     sConfig.Channel = ADC_CHANNEL_8;
		   break;

		   default:
		   break;
		}

		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		    Error_Handler();
		}

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		adc = HAL_ADC_GetValue(&hadc1) ;
		HAL_ADC_Stop(&hadc1);
		return adc;
}

/******************************************************************
 * SENDING ADC VALUE TO TUYA USING UART
******************************************************************/

void Serial_out()
{
	    uint8_t  ADC_Tx_Buff[15];
	    uint8_t  bytes[2];
	    uint8_t  Check_Sum = 0;
	    uint32_t Tx_Data_Sum = 0;

	    ADC_Tx_Buff[0]  = 0x55;
	    ADC_Tx_Buff[1]  = 0xAA;
	    ADC_Tx_Buff[2]  = 0x03;
	    ADC_Tx_Buff[3]  = 0x07;
	    ADC_Tx_Buff[4]  = 0x00;
	    ADC_Tx_Buff[5]  = 0x08;
	    ADC_Tx_Buff[7]  = 0x02;
	    ADC_Tx_Buff[8]  = 0x00;
	    ADC_Tx_Buff[9]  = 0x04;
	    ADC_Tx_Buff[10] = 0x00;
	    ADC_Tx_Buff[11] = 0x00;


	    ADC_Tx_Buff[6] = 0x68;
	  	bytes[0] = ADCValue1_0 >> 8;
	  	bytes[1] = ADCValue1_0 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	     Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x69;
	  	bytes[0] = ADCValue1_1 >> 8;
	  	bytes[1] = ADCValue1_1 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6B;
	  	bytes[0] = ADCValue1_2 >> 8;
	  	bytes[1] = ADCValue1_2 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6C;
	  	bytes[0] = ADCValue1_3 >> 8;
	  	bytes[1] = ADCValue1_3 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6D;
	  	bytes[0] = ADCValue1_4 >> 8;
	  	bytes[1] = ADCValue1_4 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	    }
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6E;
	  	bytes[0] = ADCValue1_5 >> 8;
	  	bytes[1] = ADCValue1_5 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6F;
	  	bytes[0] = ADCValue1_6 >> 8;
	  	bytes[1] = ADCValue1_6 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i = 0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;


	  	ADC_Tx_Buff[6] = 0x6A;
	  	bytes[0] = ADCValue1_7 >> 8;
	  	bytes[1] = ADCValue1_7 & 0x00FF;
	  	ADC_Tx_Buff[12] = bytes[0];
	  	ADC_Tx_Buff[13] = bytes[1];

	  	for(uint8_t i =0; i <= 13; i++)
	  	{
	  	  	 Tx_Data_Sum += ADC_Tx_Buff[i];
	  	}
	  	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	  	ADC_Tx_Buff[14] = Check_Sum ;
	  	HAL_UART_Transmit(&huart2,ADC_Tx_Buff,sizeof(ADC_Tx_Buff),100);
	  	Check_Sum = 0;
	  	Tx_Data_Sum = 0;
}

/*
 *
 */

void Tuya_Reset()
{
      if(Tuya_Reset_Flag == 1)
      {
         Tuya_Reset_Flag = 0;
         HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
         HAL_Delay(Delay);
         HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
      }
}
/***********************SWITCH_OPERATION**************************/
void Touch_Read(void)
{
	/*********************************** SWITCH1 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH0_GPIO_Port,TOUCH0_Pin))
	{
			static uint32_t Start_Time = 0;
			static uint8_t long_press = 0;
		case GPIO_PIN_SET:
			;
			if(Touch_Flag[0] == 0)             //OFF _CHECKING
			{
				switch_operation(Switch_1,SW_ON);

				Start_Time=HAL_GetTick();

				long_press = 0;
			}
			else if(Touch_Flag[0] == 2)          //ON
			{
				switch_operation(Switch_1,SW_OFF);
				Start_Time=HAL_GetTick();
				long_press = 0;
			}
			if((HAL_GPIO_ReadPin(TOUCH0_GPIO_Port,TOUCH0_Pin)==GPIO_PIN_SET))
			{
				if(Touch_Flag[0]==1||Touch_Flag[0]==3)
				{
					if(long_press==0)
					{
					   uint32_t now_time = HAL_GetTick();

						if((now_time-Start_Time) > 5000)
						{
							long_press=1;
							switch_long_press();	/////////LONG_PRESS_FOR WIFI/////////////

						}
					 }
				 }
		    }
			HAL_Delay(10);
			break;
		case GPIO_PIN_RESET:
			;
			if(Touch_Flag[0] == 1)
			{
				Touch_Flag[0] = 2;
			}
			else if(Touch_Flag[0] == 3)
			{
				Touch_Flag[0] = 0;
			}
		break;
	default:
		break;
	}
/*********************************** SWITCH 2 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH1_GPIO_Port,TOUCH1_Pin))
	{
			static uint32_t Start_Time = 0;
			static uint8_t long_press = 0;
		case GPIO_PIN_SET:
			;
			if(Touch_Flag[1]==0)
			{
				switch_operation(Switch_2, SW_ON);
				long_press = 0;
				Start_Time = HAL_GetTick();
			}
			else if(Touch_Flag[1]==2)
			{
				switch_operation(Switch_2, SW_OFF);
				long_press = 0;
				 Start_Time = HAL_GetTick();
			}
			if(HAL_GPIO_ReadPin(TOUCH1_GPIO_Port,TOUCH1_Pin)==GPIO_PIN_SET)
			{
				if(long_press==0)
				{
					uint32_t  now_time=HAL_GetTick();

					if((now_time-Start_Time)>5000)
					{
						long_press=1;
						switch_long_press();             /////////LONG_PRESS_FOR WIFI/////////////

					}

				}
			}
			HAL_Delay(10);
			break;

		case GPIO_PIN_RESET:
			;
			if(Touch_Flag[1]==1) //off
			{
				Touch_Flag[1]=2;
			}
			else if(Touch_Flag[1]==3) //on
			{
				Touch_Flag[1]=0;
			}
			break;
		default:
			break;
	}
	/*********************************** SWITCH 3 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH2_GPIO_Port,TOUCH2_Pin))
	{
		case GPIO_PIN_SET:
			;
			 if(Touch_Flag[2]==0)      /************0,1,2,3**************/
				 {

				  switch_operation(Switch_3, SW_ON);

				 }
			 if(Touch_Flag[2]==2)
				 {
				  switch_operation(Switch_3, SW_OFF);

				 }
			 HAL_Delay(10);
			 break;
		case GPIO_PIN_RESET:
			;
				 if(Touch_Flag[2]==1)
				 {
					 Touch_Flag[2]=2;

				 }
				 if(Touch_Flag[2]==3)
				 {
			        Touch_Flag[2]=0;
				 }
				 break;
	   default:
			break;

	}
	/*********************************** SWITCH 4 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH3_GPIO_Port,TOUCH3_Pin))
	{
	  case GPIO_PIN_SET:
		  ;
		 if(Touch_Flag[3]==0)
		 	 {
			     switch_operation(Switch_4, SW_ON);
		 	 }
		 if(Touch_Flag[3]==2)
			 {
				 switch_operation(Switch_4, SW_OFF);
			 }
		 HAL_Delay(10);
		 break;
	 case GPIO_PIN_RESET:
		 ;
			 if(Touch_Flag[3]==1)
			 {
				 Touch_Flag[3]=2;

			 }
			 if(Touch_Flag[3]==3)
			 {
		         Touch_Flag[3]=0;
			 }
		break;
	default:
		break;

	}
	/*********************************** SWITCH 5 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH4_GPIO_Port,TOUCH4_Pin))
	{
	case GPIO_PIN_SET:
		;
		 if(Touch_Flag[4]==0)
		 	  {
			      switch_operation(Switch_5, SW_ON);
		 	  }
		 if(Touch_Flag[4]==2)
		 	{
		 		 switch_operation(Switch_5, SW_OFF);
		 	}
		 HAL_Delay(10);
		 break;
	case GPIO_PIN_RESET:
		;
			 if(Touch_Flag[4]==1)
			 {
				 Touch_Flag[4]=2;

			 }
			 if(Touch_Flag[4]==3)
			 {
		        Touch_Flag[4]=0;
			 }
			 break;
	default:
		break;

	}
	/*********************************** SWITCH 6 ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH5_GPIO_Port,TOUCH5_Pin))
		{
		case GPIO_PIN_SET:
			;
			 if(Touch_Flag[5]==0)
			 	  {
				     switch_operation(Switch_6, SW_ON);
			 	  }
			 if(Touch_Flag[5]==2)
			 	{
			 		switch_operation(Switch_6, SW_OFF);
			 	}
			 HAL_Delay(10);
			 break;
		case GPIO_PIN_RESET:
			;
				 if(Touch_Flag[5]==1)
				 {
					 Touch_Flag[5]=2;

				 }
				 if(Touch_Flag[5]==3)
				 {
			         Touch_Flag[5]=0;

				 }
				 break;
		default:
			break;

		}
	/***********************************  DIMMER SWITCH  ****************************/
	switch(HAL_GPIO_ReadPin(TOUCH6_GPIO_Port,TOUCH6_Pin))
		{
		 case GPIO_PIN_SET:

			 if(Touch_Flag[6]==0)      //OFF
			 {
			     switch_operation(Switch_7,SW_ON);
			 }
			 else if(Touch_Flag[6]==2)     //ON
			 {
				 switch_operation(Switch_7,SW_OFF);
			 }
			 HAL_Delay(10);
			 break;
		 case GPIO_PIN_RESET:

			 if(Touch_Flag[6]==1)
			 {
				 Touch_Flag[6]=2;
			 }
			 else if(Touch_Flag[6]==3)
			 {
				Touch_Flag[6]=0;
			 }
			 break;
		default:
			break;
		//DIMMER SWITCH OPERATIONS
		}
		if(dimmer_flag==1)
			{
				switch(HAL_GPIO_ReadPin(DIMMER_UP_GPIO_Port, DIMMER_UP_Pin))    /***************DIMMMER SWITCH_1 UP*******************/
				{
					case GPIO_PIN_SET:
					 if(Touch_Flag[7]==0)
					 {
						 Touch_Flag[7] = 1;
						 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MAX_PWM);

						 if(dimmer_level < 100)
							 {
							     dimmer_level = dimmer_level + 5;
							     if(dimmer_level > 100)
							     {
							    	 dimmer_level = 100;
							     }
							     dimmer_operation(SW_ON,dimmer_level);
							     HAL_Delay(10);
							 }

					 }
					break;
					case GPIO_PIN_RESET:
						if(Touch_Flag[7]==1)
						{
							Touch_Flag[7]=0;
							__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MIN_PWM);
						}
						break;
					default:
						break;
				}

				switch(HAL_GPIO_ReadPin(DIMMER_DOWN_GPIO_Port,DIMMER_DOWN_Pin))                    /***************DIMMMER SWITCH_2 DOWN*******************/
				{
					case GPIO_PIN_SET:
					 if(Touch_Flag[8]==0)
					 {
						 Touch_Flag[8]=1;
						 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,MAX_PWM);
						 if(dimmer_level > 1)
							 {
							     dimmer_level = dimmer_level - 5;
							     if(dimmer_level < 1)
							     {
							    	 dimmer_level = 1;
							     }
							     dimmer_operation(SW_ON,dimmer_level);
							     HAL_Delay(10);
							 }
					 }

					break;
					case GPIO_PIN_RESET:
						if(Touch_Flag[8]==1)
						{
							Touch_Flag[8]=0;
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,MIN_PWM);
						}
						break;
					default:
						break;

				}
			}
}



void switch_operation(uint8_t SW_NO,uint8_t SW_STS)
{
	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD, SW_NO,I2C_MEMADD_SIZE_16BIT, &SW_STS, 1, 100)==HAL_OK)
	{
		HAL_Delay(20);
	}
	/*********************************** SWITCH 1 ****************************/
 switch(SW_NO)
		{
	 case Switch_1:
						if(SW_STS==0x01)
						{
							Touch_Flag[0]=1;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,MAX_PWM);      //LED0
							HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,GPIO_PIN_SET); //RELAY0 SET PB1
							Tx_Switch_Satus(Switch_1,SW_ON);
							HAL_Delay(Delay);
						}
						else if(SW_STS==0x00)
						{
							 Touch_Flag[0]=3;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,MIN_PWM);         //LED0RESET
							HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,GPIO_PIN_RESET);//RELAY OFF
							Tx_Switch_Satus(1,SW_OFF);
							HAL_Delay(Delay);
						}

	 break;
	/*********************************** SWITCH 2 ****************************/
	case Switch_2:
						if(SW_STS==0x01)
			        	{
							Touch_Flag[1]=1;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,MAX_PWM);       //LED1
							HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,GPIO_PIN_SET); //RELAY2
							Tx_Switch_Satus(Switch_2,SW_ON);
							HAL_Delay(Delay);
						 }
						else if(SW_STS==0x00)
						{
							 Touch_Flag[1]=3;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,MIN_PWM);         //LED_RESET
							HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,GPIO_PIN_RESET);//RELAY_OFF
							Tx_Switch_Satus(Switch_2,SW_OFF);
							HAL_Delay(Delay);
						}
						break;
		/*********************************** SWITCH 3 ****************************/
	case Switch_3:
		                if(SW_STS==0x01)
						{
							Touch_Flag[2]=1;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,MAX_PWM); //LED2
							HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,GPIO_PIN_SET);  //RELAY3
							Tx_Switch_Satus(Switch_3,SW_ON);
							HAL_Delay(Delay);
						}
						else if(SW_STS==0x00)
						{
							 Touch_Flag[2]=3;
						    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,MIN_PWM);       //LED_RESET
							HAL_GPIO_WritePin(RELAY2_GPIO_Port,RELAY2_Pin,GPIO_PIN_RESET);//RELAY_OFF
							Tx_Switch_Satus(Switch_3,SW_OFF);
							HAL_Delay(Delay);
						}
				break;
		/*********************************** SWITCH 4 ****************************/
	case Switch_4:
		                if(SW_STS==0x01)
						{
							Touch_Flag[3]=1;
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,MAX_PWM);  //LED3
							HAL_GPIO_WritePin(RELAY3_GPIO_Port,RELAY3_Pin,GPIO_PIN_SET); //RELAY4
							HAL_GPIO_WritePin(RELAY4_GPIO_Port,RELAY4_Pin,GPIO_PIN_SET); //RELAY 5
							Tx_Switch_Satus(Switch_4,SW_ON);
							HAL_Delay(Delay);
						}
						else if(SW_STS==0x00)
						{
							 Touch_Flag[3]=3;
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,MIN_PWM);   //LED_RESET
							HAL_GPIO_WritePin(RELAY4_GPIO_Port,RELAY4_Pin,GPIO_PIN_RESET); //RELAY OFF
							HAL_GPIO_WritePin(RELAY3_GPIO_Port,RELAY3_Pin,GPIO_PIN_RESET); //RELAY4

							Tx_Switch_Satus(Switch_4,SW_OFF);
							HAL_Delay(Delay);
						}
				break;
	/*********************************** SWITCH 5 ****************************/
	case Switch_5:
		               if(SW_STS==0x01)
						{
							Touch_Flag[4]=1;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,MAX_PWM);      //LED4
							HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,GPIO_PIN_SET); //RELAY5
							Tx_Switch_Satus(Switch_5,SW_ON);
							HAL_Delay(Delay);
						}
						else if(SW_STS==0x00)
						{
							 Touch_Flag[4]=3;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,MIN_PWM);  //LED_RESET
							HAL_GPIO_WritePin(RELAY5_GPIO_Port,RELAY5_Pin,GPIO_PIN_RESET);//RELAY_OFF
							Tx_Switch_Satus(Switch_5,SW_OFF);
							HAL_Delay(Delay);
						}
				break;
				/*********************************** SWITCH 6 ****************************/
	case Switch_6:
					   if(SW_STS==0x01)
						{
							Touch_Flag[5]=1;
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,MAX_PWM); //LED5
							HAL_GPIO_WritePin(RELAY6_GPIO_Port,RELAY6_Pin,GPIO_PIN_SET); //RELAY6
							Tx_Switch_Satus(Switch_6,SW_ON);
							HAL_Delay(Delay);
						}
						else if(SW_STS==0x00)
						{
							 Touch_Flag[5]=3;
							__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,MIN_PWM);          //LED_RESET
							HAL_GPIO_WritePin(RELAY6_GPIO_Port,RELAY6_Pin,GPIO_PIN_RESET); //RELAY6
							Tx_Switch_Satus(Switch_6,SW_OFF);
							HAL_Delay(Delay);
						}
				break;
	case Switch_7:
				               if(SW_STS==0x01)
								{

							     	Touch_Flag[6]=1;
									Tx_Switch_Satus(Switch_7,SW_ON);
									dimmer_operation(SW_ON, dimmer_level);
									HAL_Delay(Delay);
								}
								else if(SW_STS==0x00)
								{

									Touch_Flag[6]=3;
									Tx_Switch_Satus(Switch_7,SW_OFF);
									dimmer_operation(SW_OFF, dimmer_level);
									HAL_Delay(Delay);
								}
						break;

	default:
		break;

		}
}

void dimmer_operation(uint8_t Status,uint8_t level)
{
	       static uint8_t temp_flag = 0;
	       dimmer_level = level;
	       timer2_period = (uint8_t)(75 - ((level - 1) * 0.7));
	       uint8_t temp_value[2] = {Status,level};
	       if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD,23,I2C_MEMADD_SIZE_16BIT ,temp_value,2, 10)==HAL_OK)
			{
				HAL_Delay(20);
			}

		  if(Status==0x01)
		  {
			  if(temp_flag == 0)
			  {
				  temp_flag = 1;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MIN_PWM);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,MIN_PWM);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,MAX_PWM);
			  }
			  if(level == 100)
			  {
				dimmer_flag = 1;
				dimmer_enable_flag = 0;
				dimmer_value_change_flag = 0;
				HAL_Delay(1);
				 HAL_GPIO_WritePin(DIMMER_GPIO_Port, DIMMER_Pin,GPIO_PIN_SET);

			  }
			  else if(level<100)
			  {
				HAL_GPIO_WritePin(DIMMER_GPIO_Port, DIMMER_Pin,GPIO_PIN_RESET);
				dimmer_flag = 1;
				dimmer_enable_flag = 1;
				dimmer_value_change_flag = 1;
			  }

		  }
		  else if(Status==0x00)
		  {
			  if(temp_flag == 1)
			  {
				  temp_flag = 0;
			    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MIN_PWM);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,ZERO_PWM);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,ZERO_PWM);
			  }
			  dimmer_enable_flag = 0;
			  HAL_GPIO_WritePin(DIMMER_GPIO_Port, DIMMER_Pin,GPIO_PIN_RESET);
			  dimmer_flag = 0;
			  dimmer_value_change_flag=0;
		  }

	   DIMMER_TX(level);

}
/*****************DIMMER_INTERRUPT CALL_BACK*********************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //FALLING_EDGE DETECTION INTTERUPT
{
    if(GPIO_Pin == INTERRUPT_Pin) // If The INT Source Is EXTI Line9 (PA0 Pin)
    {
    	if(dimmer_value_change_flag == 1 )
    	{
    		dimmer_value_change_flag =0;
    		if((timer2_period >= 5) && (timer2_period <= 75) )
    		{
    		__HAL_TIM_SET_AUTORELOAD(&htim2,timer2_period);
    		}
    	}
      if(dimmer_enable_flag == 1)
	   {
     	   HAL_TIM_Base_Start_IT(&htim2);
	   }

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //DIMMMER
{
	if(htim->Instance == TIM2)
	{

	 HAL_TIM_Base_Stop_IT(&htim2);
	 HAL_GPIO_WritePin(DIMMER_GPIO_Port, DIMMER_Pin,GPIO_PIN_SET);
	 HAL_Delay(1);
     HAL_GPIO_WritePin(DIMMER_GPIO_Port, DIMMER_Pin, GPIO_PIN_RESET);

	}

}
void switch_long_press(void)
{
       HAL_UART_Transmit(&huart2,Longpress_Tx_Buff,sizeof(Longpress_Tx_Buff),100);
}
void Tx_Switch_Satus(uint8_t SW_NO, uint8_t SW_STS)
{
	/*
		    *  Switch_Touch_Tx_Buff[0] and Switch_Touch_Tx_Buff[1] are start bytes
		    *  Switch_Touch_Tx_Buff[2] is Version
		    *  Switch_Touch_Tx_Buff[3] is Comment byte to indicate which type of data is sending
		    *  Switch_Touch_Tx_Buff[4] and Switch_Touch_Tx_Buff[5] Data Length
		    *  Switch_Touch_Tx_Buff[6] Switch Number
		    *  Switch_Touch_Tx_Buff[7] Data Type
		    *  Switch_Touch_Tx_Buff[8] and Switch_Touch_Tx_Buff[9] Data Length
		    *  Switch_Touch_Tx_Buff[10] Switch Status
		    *  Switch_Touch_Tx_Buff[11] CRC {CRC is remainder  sum of all byte divided by 256 }
			*/

			uint8_t Switch_Touch_Tx_Buff[12] = {0x55,0xAA,0x03,0x07,0x00,0x05,0x01,0x01,0x00,0x01,0x00,0x00};  //Data frame for sending Switch Status

			uint8_t Check_Sum = 0;

			if(SW_NO == Switch_7)
			{

				Switch_Touch_Tx_Buff[6] = 0x65;

			}
			else
			{
			    Switch_Touch_Tx_Buff[6] = SW_NO; //SWITCH_NUMBE
			}

			Switch_Touch_Tx_Buff[10] = SW_STS; //SWITCH ON AND OFF

			uint32_t Tx_Data_Sum = 0;

		    for(uint8_t i =0;i < sizeof(Switch_Touch_Tx_Buff); i++)
		    {
		       Tx_Data_Sum += Switch_Touch_Tx_Buff[i];
		    }

		    Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);

		    Switch_Touch_Tx_Buff[11] = Check_Sum ;

		    HAL_UART_Transmit(&huart2,Switch_Touch_Tx_Buff,sizeof(Switch_Touch_Tx_Buff),100);


}
void DIMMER_TX(uint8_t DIMMER_COUNTER)
{
 uint8_t DIMMER_Tx_Buff[15] = {0x55,0xAA,0x03,0x07,0x00,0x08,0x6a,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x00};//Data frame for sending DIMMER Status
 uint8_t Check_Sum = 0;
 uint32_t DIMMER_COUNTED=DIMMER_COUNTER;
				DIMMER_Tx_Buff[10]= DIMMER_COUNTED>>24 & 0xFF;
				DIMMER_Tx_Buff[11]= DIMMER_COUNTED>>16 & 0xFF;
				DIMMER_Tx_Buff[12]= DIMMER_COUNTED>>8 & 0xFF;
				DIMMER_Tx_Buff[13]= DIMMER_COUNTED>>0 & 0xFF;

			uint32_t Tx_Data_Sum = 0;

			  for(uint8_t i =0;i < sizeof(DIMMER_Tx_Buff); i++)
				    {
				       Tx_Data_Sum += DIMMER_Tx_Buff[i];
				    }

			  Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);

			  DIMMER_Tx_Buff[14] = Check_Sum ;

			  HAL_UART_Transmit(&huart2,DIMMER_Tx_Buff,sizeof(DIMMER_Tx_Buff),100);
}

/**********************UART callback routine*************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     Cyclic_Buffer[wrI]=RX_Buffer;
	 wrI=( wrI + 1 )%256;
	 Count++;
	 HAL_UART_Receive_IT(&huart2, &RX_Buffer,1);

}
/****************USART_READ****************/
void Uart_Read_data()
{
  if(Count !=0)
		 {
	        RX_Buffer =  Cyclic_Buffer[rdI];
	        rdI=( rdI + 1 )%256;
	 		 Count--;
	 		 if(Count == 0)
	 			       {
	 				       rdI = wrI;
	 			       }

	 		         Rx_Second_byte	 =RX_Buffer;
	 		      if((Rx_Second_byte == 0xAA) && (Rx_First_byte == 0x55))
	 			       {
	 			           RX_data_Count = 0;
	 			           RX_Data_Start_Flag = 1;
	 			           RX_Data[RX_data_Count] = Rx_First_byte;
	 			           RX_data_Count++;
	 			       }
	 			   if(RX_Data_Start_Flag == 1)
	 			       {
	 			           RX_Data[RX_data_Count] = RX_Buffer;
	 			           if(RX_data_Count == 5)
	 			           {
	 			        	  RX_data_length = RX_Data[RX_data_Count];

	 			           }
	 			           if(RX_data_Count == (6 + RX_data_length))
	 			           {

	 			               RX_Data_Start_Flag = 0;
	 			               RX_Complete_flag = 1;
	 			           }
	 			          RX_data_Count++;
	 			       }
	 			       Rx_First_byte  = Rx_Second_byte;
		 }
}
void Rx_Data_evaluation(void)
{
	if(RX_Complete_flag==1)
	{

		uint32_t Rx_Data_Sum = 0;

		uint8_t Check_Sum = 0;

		RX_Complete_flag = 0;

		for(uint8_t i=0 ; i< (RX_data_Count - 1);i++)
		{
			Rx_Data_Sum +=	RX_Data[i];
		}

		 Check_Sum = (uint8_t)((Rx_Data_Sum)%(256));

		if( Check_Sum == RX_Data[(RX_data_Count - 1)])
		{
		   if(RX_data_Count >= 5)
		   {
/*******************HEART_BEAT_sending**************/
			 if(RX_Data[3]==0x00)
			 {
					if(Rx_CNT == 0)
					{
					   HAL_UART_Transmit(&huart2, Hbt, sizeof(Hbt), 100);
					   Rx_CNT = 1;
					   timer_val = HAL_GetTick();
					}
			        else
			        {
					   uint32_t  Now = HAL_GetTick();
					   if(Now - timer_val < 2000)
					   {
							 Rx_CNT = 0;
							 HAL_UART_Transmit(&huart2, Hbt, sizeof(Hbt), 100);
							 Now=timer_val;
						}
						else if(Rx_CNT==1)
						{
							 HAL_UART_Transmit(&huart2, Hbt1, sizeof(Hbt1), 100);
							 Now=timer_val;
						}
			         }
			    		   Tuya_Heart_Flag = 1;
			    		   Tuya_Start_Time = HAL_GetTick();
			 }
/******************PID_SENDING*********************/
			else if(RX_Data[3]==0x01)
			{
				HAL_UART_Transmit(&huart2, PID, sizeof(PID), 100);
				Tuya_flag = 1;
			}
/*************WORKING_MODE********************/
			else if(RX_Data[3] == 0x02)
			{
				HAL_UART_Transmit(&huart2, Work_Mode, sizeof(Work_Mode), 100);
			}
/*****************WI_FI_CONNECTION*********************/
			else if(RX_Data[3] == 0x03)
			{
/*******AUTO-CONFIGURATION*********/
				 if(RX_Data[6] == 0x00)
				 {
					 Idication_Flag = 2;
				 }
/***************AP-MODE-INDICATION********************/
				 if(RX_Data[6] == 0x01)
				 {
					 Idication_Flag = 3;
				 }
				else if (RX_Data[6] == 0x02)
				{
					Idication_Flag = 1;
				}
				else if (RX_Data[6] == 0x04)
				{
					Stop_Indication();
					Idication_Flag = 0;
				}
				 HAL_UART_Transmit(&huart2,WiFi_Status, sizeof(WiFi_Status), 100);
			}
/****************SWITCH_OPERATIONS**************************/
			else if (RX_Data[3] == 0x06)
			{

				if (RX_Data[6] == 0x0D)
				{
					Master_Switch(RX_Data[10]);
				}
				 else if (RX_Data[6] == 0x6F)
				{
					set_max_pwm(RX_Data[13]);
				}
				else if (RX_Data[6] == 0x70)
				{
					set_min_pwm(RX_Data[13]);
				}
				else if(RX_Data[6]==0x6A )
				{
					dimmer_operation(dimmer_flag, RX_Data[13]);
				}
				else
				{
					if(RX_Data[6] == 0x65)
					{
						 RX_Data[6] = 0x07;
					}
				     switch_operation (RX_Data[6],RX_Data[10]);
				}
			}
		    else if(RX_Data[3] == 0x08)
		    {
				 Status_ReadData();
		    }
	    }
     }
			    RX_data_Count = 0;
   }
 }

void Indication(void)
{
	      static uint32_t Indication_Start_Time = 0;
		  static uint8_t Breath = 0;
		  static uint8_t Breath_Flag = 0;
		  uint32_t Now = 0;
		  if(Idication_Flag == 1)
		  {
			  Now = HAL_GetTick();

			  if( Now - Indication_Start_Time > 50 )
			  {
				  Indication_Start_Time = Now;
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(Breath*2));      //LED7

				  	  if(Breath == 50)
					 {
						 Breath_Flag = 1;
					 }
					 else if (Breath == 1)
					 {
						 Breath_Flag = 0;
					 }

					 if (Breath_Flag == 0)
					 {
						 Breath++;
					 }
					 else if (Breath_Flag == 1)
					 {
						 Breath--;
					 }
			   }
		  }
		  else if(Idication_Flag == 2)                 /****************AUTO****************/
		  {
			 Now = HAL_GetTick();
			 if( Now - Indication_Start_Time > 250 )
			 {
				Indication_Start_Time =  Now ;

				if(Idication_Control_Flag  == 0)
				{
					Idication_Control_Flag = 1;

					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,H_PWM);      //LED7    /*************TOGGLE-LED*************/
				}
				else if(Idication_Control_Flag  == 1)
				{
					Idication_Control_Flag = 0;

					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,L_PWM);      //LED7
				}
		     }
		  }
		  else if(Idication_Flag == 3)
		  {
			Now = HAL_GetTick();
			if( Now - Indication_Start_Time > 1500 )
			{
			  Indication_Start_Time =  Now ;
			  if(Idication_Control_Flag  == 0)
			  {
				  Idication_Control_Flag = 1;
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,H_PWM);      //LED7
				}
				else if(Idication_Control_Flag  == 1)
				{
					   Idication_Control_Flag = 0;
					   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,L_PWM);      //LED7
				 }
		     }

	       }
 }
void Stop_Indication(void)
{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,Z_PWM);      //LED7
		Status_ReadData();
}
/***************MASTER_SWITCH_APP_SIDE***********************/
void Master_Switch(uint8_t SW_STS)
{
	 uint8_t Master_Switch[12] = {0x55,0xAA,0x03,0x07,0x00,0x05,0x0D,0x01,0x00,0x01,0x00,0x00};

	uint8_t Check_Sum = 0;

	Master_Switch[10] = SW_STS;

	if(Master_Switch[10]==0x01)
	{
		switch_operation(Switch_1, SW_ON);
		switch_operation(Switch_2, SW_ON);
		switch_operation(Switch_3, SW_ON);
		switch_operation(Switch_4, SW_ON);
		switch_operation(Switch_5, SW_ON);
		switch_operation(Switch_6, SW_ON);
		switch_operation(Switch_7, SW_ON);
	}
	else
	{
		switch_operation(Switch_1, SW_OFF);
		switch_operation(Switch_2, SW_OFF);
		switch_operation(Switch_3, SW_OFF);
		switch_operation(Switch_4, SW_OFF);
		switch_operation(Switch_5, SW_OFF);
		switch_operation(Switch_6, SW_OFF);
		switch_operation(Switch_7, SW_OFF);
	}
	 uint32_t Tx_Data_Sum = 0;

		for(int i=0;i <= sizeof(Master_Switch) ;i++)
		{

			Tx_Data_Sum+=Master_Switch[i];
		}
		Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);

		Master_Switch[11] = Check_Sum ;

		HAL_UART_Transmit(&huart2, Master_Switch, sizeof(Master_Switch), 100);
}

void set_max_pwm(uint8_t max_val)
{
	MAX_PWM=max_val;
	 if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD, 21, I2C_MEMADD_SIZE_16BIT, &MAX_PWM, 1, 100) == HAL_OK)
	        {
	            HAL_Delay(20);
	        }
	        if(Touch_Flag[0] == 2)                   //SWITCH_ON CASE
	        {
	             __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,MAX_PWM);
	        }
	        if(Touch_Flag[1] == 2)
	        {
	        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,MAX_PWM);
	        }
	        if(Touch_Flag[2] == 2)
	        {
	        	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,MAX_PWM);
	        }
	        if(Touch_Flag[3] == 2)
	        {
	        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,MAX_PWM);
	        }
	        if(Touch_Flag[4] == 2)
	        {
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,MAX_PWM);
	        }
	        if(Touch_Flag[5] == 2)
	        {
	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,MAX_PWM);
	        }
	        if(Touch_Flag[6] == 2)
	        {
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,MAX_PWM);
	        }
	        if(Touch_Flag[7] == 2)
	        {
	            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,MAX_PWM);
	        }
	        if(Touch_Flag[8] == 2)
			{
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MAX_PWM);
			}

}
void set_min_pwm(uint8_t min_val)
{
	MIN_PWM=min_val;
	 if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD, 20, I2C_MEMADD_SIZE_16BIT, &MIN_PWM, 1, 100) == HAL_OK)
	        {
	            HAL_Delay(20);
	        }
	            if(Touch_Flag[0] == 0)                   //SWITCH_ON CASE
		        {
		             __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,MIN_PWM);
		        }
		        if(Touch_Flag[1] == 0)
		        {
		        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,MIN_PWM);
		        }
		        if(Touch_Flag[2] == 0)
		        {
		        	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,MIN_PWM);
		        }
		        if(Touch_Flag[3] == 0)
		        {
		        	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,MIN_PWM);
		        }
		        if(Touch_Flag[4] == 0)
		        {
		            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,MIN_PWM);
		        }
		        if(Touch_Flag[5] == 0)
		        {
		            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,MIN_PWM);
		        }
		        if(Touch_Flag[6] == 0)
		        {
		            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,MIN_PWM);
		        }
		        if(Touch_Flag[7] == 0)
		        {
		            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,MIN_PWM);
		        }
		        if(Touch_Flag[8] == 0)
				{
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,MIN_PWM);
				}

}
void Status_ReadData()
{
	uint8_t  Status_Tx_Buff[50]={0x55,0xAA,0x03,0x07,0x00,0x28,0x01,0x01,0x00,0x01,0x00,0x02,0x01,0x00,0x01,0x00,0x03,0x01,0x00,0x01,0x00,0x04,0x01,0x00,0x01,0x00,0x05,0x01,0x00,0x01,0x00,0x06,0x01,0x00,0x01,0x00,0x65,0x01,0x00,0x01,0x00};
	uint8_t  Switch_StatusNew[8];
	uint8_t  Check_Sum = 0;
	uint32_t Tx_Data_Sum = 0;
	uint8_t  SW_State=0;
	uint8_t  k =0;


	for(uint8_t j=1;j<=NUMBER_OF_SWITCHES;j++)
	{
		if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, j, I2C_MEMADD_SIZE_16BIT, &SW_State, 1, 10) == HAL_OK)
		{
			  HAL_Delay(20);
			  Switch_StatusNew[k]= SW_State;
			  k++;
		}
	}
	uint8_t temp[2]={0};
	if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, 23, I2C_MEMADD_SIZE_16BIT, temp, 2, 10) == HAL_OK)
	{
		HAL_Delay(20);
	}
	Status_Tx_Buff[10] =   Switch_StatusNew[0];
	Status_Tx_Buff[15] =   Switch_StatusNew[1];
	Status_Tx_Buff[20] =   Switch_StatusNew[2];
	Status_Tx_Buff[25] =   Switch_StatusNew[3];
	Status_Tx_Buff[30] =   Switch_StatusNew[4];
	Status_Tx_Buff[35] =   Switch_StatusNew[5];
	Status_Tx_Buff[40] =   Switch_StatusNew[6];
	Status_Tx_Buff[49] =   temp[2];
	for(uint8_t i = 0; i <= 50; i++)
	{
	   Tx_Data_Sum += Status_Tx_Buff[i];
	}

	Check_Sum  = (uint8_t)((Tx_Data_Sum)%256);
	Status_Tx_Buff[50] = Check_Sum ;
	HAL_UART_Transmit(&huart2,  Status_Tx_Buff, sizeof(Status_Tx_Buff),100);
	Check_Sum = 0;
	Tx_Data_Sum = 0;

}
//*****************************EEPROM*********************************//
void EEPROM_initial()
{
	if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, 22,I2C_MEMADD_SIZE_16BIT, &Temp_var, 1, 100)==HAL_OK)
	{
		HAL_Delay(Delay);
	}
	 if(Temp_var!=1)
		    {
			    Temp_var=1;
			    if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD, 20, I2C_MEMADD_SIZE_16BIT, &MIN_PWM, 1, 10) == HAL_OK)
			    {
			        HAL_Delay(20);
			    }

			    if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADD, 21, I2C_MEMADD_SIZE_16BIT, &MAX_PWM, 1, 10) == HAL_OK)
			    {
			        HAL_Delay(20);
			    }


			    if(HAL_I2C_Mem_Write(&hi2c1,EEPROM_ADD,22,I2C_MEMADD_SIZE_16BIT,&Temp_var,1,100)==HAL_OK)
			    {
			        HAL_Delay(20);
			    }
		    }

		    if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, 20, I2C_MEMADD_SIZE_16BIT, &MIN_PWM, 1, 10) == HAL_OK)
		    {
		        HAL_Delay(20);
		    }

		    if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, 21, I2C_MEMADD_SIZE_16BIT, &MAX_PWM, 1, 10) == HAL_OK)
		    {
		        HAL_Delay(20);
		    }
	        for(uint8_t i=1;i<=NUMBER_OF_SWITCHES;i++)
	        {
	              HAL_Delay(10);
	              uint8_t  SW_State=0;

	              if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD, i, I2C_MEMADD_SIZE_16BIT, &SW_State, 1, 10) == HAL_OK)
	              {

	                   HAL_Delay(20);

	                   if(i==7)
	                   {
	                	   uint8_t temp_value[2] ={0};
	                	   if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADD,23,I2C_MEMADD_SIZE_16BIT ,temp_value,2, 10)==HAL_OK)
	                	  	  {
	                	  	        		HAL_Delay(20);
	                	  	        		dimmer_operation(SW_State, temp_value[1]);
	                	  	   }
	                   }

	              }
	              switch_operation(i, SW_State);
	        }

}

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
