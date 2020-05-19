/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
const short L1=66;
const short L2=40;
const short L3=37;
float cx[4]={0};
float cy[4]={0};
float cz[4]={0};


#define JonintNum 12
#define RF_J1 0
#define LF_J1 1
#define RR_J1 2
#define LR_J1 3

#define RF_J2 4
#define LF_J2 5
#define RR_J2 6
#define LR_J2 7

#define RF_J3 8
#define LF_J3 9
#define RR_J3 10
#define LR_J3 11

#define ARM_J1 12
#define ARM_J2 13
#define ARM_J3 14
#define WAIST  15 

#define RF 100
#define RR 200
#define LF 300
#define LR 400






typedef struct Kinematics
{	float StartAngle[JonintNum];
	uint32_t StepNum;//数据步距
}KinematicsArm;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#define JonintNum 12
#define RF_J1 0
#define LF_J1 1
#define RR_J1 2
#define LR_J1 3

#define RF_J2 4
#define LF_J2 5
#define RR_J2 6
#define LR_J2 7

#define RF_J3 8
#define LF_J3 9
#define RR_J3 10
#define LR_J3 11

#define ARM_J1 12
#define ARM_J2 13
#define ARM_J3 14
#define WAIST  15 

//typedef struct Kinematics
//{	float StartAngle[JonintNum];
//	uint32_t StepNum;//数据步距
//}KinematicsArm;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
KinematicsArm KMGecko;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;

#define RXBUFFERSIZE   1 //缓存大小
#define USART3_REC_LEN  			33  	//按顺序加速度包，角速度包和角度包各11个字节
#define USART4_REC_LEN  			20  	//定义最大接收字节数 20
#define SEND3_LEN	33		//发送字节数
#define SEND4_LEN	33		//发送字节数

uint8_t USART_RX3_BUF[USART3_REC_LEN],USART_RX4_BUF[USART4_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX3_STA=0,USART_RX4_STA=0;       //接收状态标记	
uint8_t aRxBuffer3[RXBUFFERSIZE],aRxBuffer4[RXBUFFERSIZE];//HAL库使用的串口接收缓冲


uint32_t AD1_Ch_Value[11]; //0:电压检测 1~10：舵机电流检测  1：
double  AD1_Ch_Value_f[11];
uint32_t AD3_Ch_Value[2];//舵机电流检测
uint32_t adc_LT1 = 0;
uint32_t adc_LT2 = 0;
uint32_t adc_LT3 = 0;
uint32_t adc_RT1 = 0;
uint32_t adc_RT2 = 0;
uint32_t adc_RT3 = 0;
uint32_t adc_LB1 = 0;
uint32_t adc_LB2 = 0;
uint32_t adc_LB3 = 0;
uint32_t adc_RB1 = 0;
uint32_t adc_RB2 = 0;
uint32_t adc_RB3 = 0;
uint32_t adc_BAT = 0;
int8_t operating_flag = 0;

double adc_LT1_d = 0;
double adc_LT2_d = 0;
double adc_LT3_d = 0;
double adc_RT1_d = 0;
double adc_RT2_d = 0;
double adc_RT3_d = 0;
double adc_LB1_d = 0;
double adc_LB2_d = 0;
double adc_LB3_d = 0;
double adc_RB1_d = 0;
double adc_RB2_d = 0;
double adc_RB3_d = 0;
double adc_BAT_d = 0;

	
double  AD3_Ch_Value_f[2];
#define AD_AVERAGE 10

uint16_t 	Time8Channel1HighTime = 0, Time8Channel2HighTime = 0, Time8Channel3HighTime = 0,Time8Channel4HighTime = 0, Time4Channel3HighTime = 0, Time4Channel4HighTime = 0;
uint16_t 	Time8Channel1Period = 0, Time8Channel2Period = 0, Time8Channel3Period = 0, Time8Channel4Period = 0, Time4Channel3Period = 0, Time4Channel4Period = 0;
uint8_t  	Time8Channel1Edge = 0, Time8Channel2Edge = 0, Time8Channel3Edge = 0, Time8Channel4Edge = 0, Time4Channel3Edge = 0, Time4Channel4Edge = 0;
//	uint16_t 	Time8Channel1Percent, Time8Channel2Percent, Time8Channel3Percent, Time8Channel4Percent, Time4Channel3Percent, Time4Channel4Percent;

uint16_t 	Time8Channel1RisingTimeLast=0, Time8Channel1RisingTimeNow = 0, Time8Channel1FallingTime = 0;
uint16_t 	Time8Channel2RisingTimeLast=0, Time8Channel2RisingTimeNow = 0, Time8Channel2FallingTime = 0;
uint16_t 	Time8Channel3RisingTimeLast=0, Time8Channel3RisingTimeNow = 0, Time8Channel3FallingTime = 0;
uint16_t 	Time8Channel4RisingTimeLast=0, Time8Channel4RisingTimeNow = 0, Time8Channel4FallingTime = 0;
uint16_t 	Time4Channel3RisingTimeLast=0, Time4Channel3RisingTimeNow = 0, Time4Channel3FallingTime = 0;
uint16_t 	Time4Channel4RisingTimeLast=0, Time4Channel4RisingTimeNow = 0, Time4Channel4FallingTime = 0;

uint16_t LT_PWM1_Value=0,LT_PWM2_Value=0,LT_PWM3_Value=0,RT_PWM1_Value=0,RT_PWM2_Value=0,RT_PWM3_Value=0,LB_PWM1_Value=0,LB_PWM2_Value=0,LB_PWM3_Value=0,RB_PWM1_Value=0,RB_PWM2_Value=0,RB_PWM3_Value=0;
uint16_t LED_Value=100;
uint8_t  LED_Flag = 1;


uint16_t testnum=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartAngleInit(void);                              
void InitRobotPosion(void);                                
void Angle(float angle,int8_t footnumber);
void Robot_Run_Left(void);
void reverse(int,float,float,float);
void setcurrentposition (int,float,float,float);
void Robot_Run_Line(void);
void Robot_Run_Right(void);
void reverse4(float,float,float);
void reverse4R(float,float,float);
void reverse4L(float,float,float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void All_PWM_Start()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void User_PWM_SetPulse(TIM_HandleTypeDef *htim,uint32_t channel,uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
	HAL_TIM_PWM_Start(htim, channel);  
}

void Set_All_PWM_Pulse(uint16_t value)
{
	User_PWM_SetPulse(&htim1, TIM_CHANNEL_1, value);
	User_PWM_SetPulse(&htim1, TIM_CHANNEL_2, value);
	User_PWM_SetPulse(&htim1, TIM_CHANNEL_3, value);
	User_PWM_SetPulse(&htim1, TIM_CHANNEL_4, value);
	
	User_PWM_SetPulse(&htim3, TIM_CHANNEL_1, value);
	User_PWM_SetPulse(&htim3, TIM_CHANNEL_2, value);
	User_PWM_SetPulse(&htim3, TIM_CHANNEL_3, value);
	User_PWM_SetPulse(&htim3, TIM_CHANNEL_4, value);
	
	User_PWM_SetPulse(&htim4, TIM_CHANNEL_1, value);
	User_PWM_SetPulse(&htim4, TIM_CHANNEL_2, value);

	User_PWM_SetPulse(&htim12, TIM_CHANNEL_1, value);
	
	User_PWM_SetPulse(&htim15, TIM_CHANNEL_1, value);
	User_PWM_SetPulse(&htim15, TIM_CHANNEL_2, value);
}

void All_Capture_IT_Start()
{
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(ADC_HandleTypeDef *hadc,uint32_t ch)   
{
	ADC_ChannelConfTypeDef ADC_ChanConf;
    
	ADC_ChanConf.Channel=ch;                                   //通道
	ADC_ChanConf.Rank=ADC_REGULAR_RANK_1;                  	//1个序列
	ADC_ChanConf.SamplingTime=ADC_SAMPLETIME_64CYCLES_5;      	//采样时间       
	ADC_ChanConf.SingleDiff=ADC_SINGLE_ENDED;  				//单边采集          		
	ADC_ChanConf.OffsetNumber=ADC_OFFSET_NONE;             	
	ADC_ChanConf.Offset=0;   
	HAL_ADC_ConfigChannel(hadc,&ADC_ChanConf);        //通道配置

	HAL_ADC_Start(hadc);                               //开启ADC
	
	HAL_ADC_PollForConversion(hadc,10);                //轮询转换
	return (uint16_t)HAL_ADC_GetValue(hadc);	            //返回最近一次ADC规则组的转换结果
}

//获取指定通道的转换值，取times次,然后平均 
//times:获取次数
//返回值:通道ch的times次转换结果平均值
uint16_t Get_Adc_Average(ADC_HandleTypeDef *hadc,uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(hadc,ch);
		HAL_Delay(5);
	}
	return temp_val/times;
} 

void Read_Radio_Channel(TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL,uint16_t HighTime,uint16_t Period,uint8_t Edge,uint16_t RisingTimeLast, uint16_t RisingTimeNow, uint16_t FallingTime)
{
		{
			if(Edge == 0)
			{
				RisingTimeNow = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL);
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL);
				Edge = 1;
				if(RisingTimeLast == 0)
				{
					Period = 0;
				}
				else
				{
					if(RisingTimeNow > RisingTimeLast)
					{
						Period = RisingTimeNow - RisingTimeLast;
					}
					else
					{
						Period = RisingTimeNow + 1000 - RisingTimeLast + 1;
					}
				}
				RisingTimeLast = RisingTimeNow;
			}
			else if(Edge == 1)
			{
				FallingTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL);	
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL);
			
				if(FallingTime < RisingTimeNow)
				{
					HighTime = FallingTime + 1000 - RisingTimeNow;
				}
				else if(FallingTime > RisingTimeNow)
				{
					HighTime = FallingTime - RisingTimeNow;
				}
				else
				{
					HighTime = (HighTime>500)?1000:0;
				}

			Edge = 0;
			}
		}
}
//AD采集任务函数
void ReadAD()
{
	uint8_t i,j;
	uint32_t adcBuf[11];
	for(j=0;j<11;j++)
		adcBuf[j] = 0;
	while(1)
	{
//		转换AD1
		for(i=0;i< AD_AVERAGE ;i++)//转换AD1 取10次均值
		{
			for(j=0;j<11;j++)//转换AD1
			{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1,0xffff);
				adcBuf[i]+=HAL_ADC_GetValue(&hadc1);
			}
		}
		
		HAL_ADC_Stop(&hadc1);
		for(j=0;j<11;j++)
		{
			AD1_Ch_Value[j] = adcBuf[j]/AD_AVERAGE;
			AD1_Ch_Value_f[j] = (double)AD1_Ch_Value[j]*3.3/65536;
		}
		
	//转换AD3	
		for(j=0;j<11;j++)
			adcBuf[j] = 0;
		for(i=0;i< AD_AVERAGE ;i++)////转换AD3 取10次均值
		{
			for(j=0;j<2;j++)
			{
				HAL_ADC_Start(&hadc3);
				HAL_ADC_PollForConversion(&hadc3,0xffff);
//				adc3Buf = HAL_ADC_GetValue(&hadc3);
//				adcBuf[i]+=adc3Buf;
				adcBuf[i]+=HAL_ADC_GetValue(&hadc3);
			}
		}
		HAL_ADC_Stop(&hadc3);
		for(j=0;j<2;j++)
		{
			AD3_Ch_Value[j] = adcBuf[j]/AD_AVERAGE;
			AD3_Ch_Value_f[j] = (double)AD1_Ch_Value[j]*3.3/65536;
		}		
		for(j=0;j<11;j++)
			adcBuf[j] = 0;
	}
}
void Read_All_Ad()
{
/*	adc_LT1 = Get_Adc_Average(&hadc1,ADC_CHANNEL_15,AD_AVERAGE);
	adc_LT2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_14,AD_AVERAGE);
	adc_LT3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_18,AD_AVERAGE);
	adc_RT1 = Get_Adc_Average(&hadc3,ADC_CHANNEL_0, AD_AVERAGE);
	adc_RT2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_10,AD_AVERAGE);
	adc_RT3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_11,AD_AVERAGE);
	adc_LB1 = Get_Adc_Average(&hadc1,ADC_CHANNEL_7, AD_AVERAGE);
	adc_LB2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_19,AD_AVERAGE);
	adc_LB3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_4, AD_AVERAGE);
	adc_RB1 = Get_Adc_Average(&hadc1,ADC_CHANNEL_17,AD_AVERAGE);
	adc_RB2 = Get_Adc_Average(&hadc3,ADC_CHANNEL_1, AD_AVERAGE);
	adc_RB3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_16,AD_AVERAGE);
	adc_BAT = Get_Adc_Average(&hadc1,ADC_CHANNEL_3, AD_AVERAGE);
*/
			adc_LT1 = Get_Adc_Average(&hadc1,ADC_CHANNEL_14,AD_AVERAGE);
			adc_LT2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_15,AD_AVERAGE);
		adc_LT3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_18,AD_AVERAGE);
		adc_RT1 = Get_Adc_Average(&hadc3,ADC_CHANNEL_0, AD_AVERAGE);
		adc_RT2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_10,AD_AVERAGE);
		adc_RT3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_11,AD_AVERAGE);
			adc_LB1 = Get_Adc_Average(&hadc1,ADC_CHANNEL_19, AD_AVERAGE);
			adc_LB2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_4,AD_AVERAGE);
			adc_LB3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_7, AD_AVERAGE);
			adc_RB1 = Get_Adc_Average(&hadc3,ADC_CHANNEL_1,AD_AVERAGE);
			adc_RB2 = Get_Adc_Average(&hadc1,ADC_CHANNEL_17, AD_AVERAGE);
		adc_RB3 = Get_Adc_Average(&hadc1,ADC_CHANNEL_16,AD_AVERAGE);
		adc_BAT = Get_Adc_Average(&hadc1,ADC_CHANNEL_3, AD_AVERAGE);

	adc_LT1_d = adc_LT1*3.3/65536;
	adc_LT2_d = adc_LT2*3.3/65536;
	adc_LT3_d = adc_LT3*3.3/65536;
	adc_RT1_d = adc_RT1*3.3/65536;
	adc_RT2_d = adc_RT2*3.3/65536;
	adc_RT3_d = adc_RT3*3.3/65536;
	adc_LB1_d = adc_LB1*3.3/65536;
	adc_LB2_d = adc_LB2*3.3/65536;
	adc_LB3_d = adc_LB3*3.3/65536;
	adc_RB1_d = adc_RB1*3.3/65536;
	adc_RB2_d = adc_RB2*3.3/65536;
	adc_RB3_d = adc_RB3*3.3/65536;
	adc_BAT_d = adc_BAT*3.3/65536;
}

//PWM控制任务函数
void User_PWM_SetValue(TIM_HandleTypeDef *htim,uint32_t channel,uint16_t value)
{
	uint16_t value_t;
	if(value < 10000) value_t =  value;
	else value_t = 0;

	__HAL_TIM_SET_COMPARE(htim,channel,value_t);
}
void PWMCtrl()
{
		//舵机运算控制开始
		
		//舵机运算控制结束
		
		//PWM控制12个舵机
		/*
		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,LT_PWM1_Value);
		User_PWM_SetValue(&htim1, TIM_CHANNEL_2,LT_PWM2_Value);
		User_PWM_SetValue(&htim1, TIM_CHANNEL_4,LT_PWM3_Value);
		User_PWM_SetValue(&htim4, TIM_CHANNEL_2,RT_PWM1_Value);
		User_PWM_SetValue(&htim15, TIM_CHANNEL_1,RT_PWM2_Value);
		User_PWM_SetValue(&htim15, TIM_CHANNEL_2,RT_PWM3_Value);
		User_PWM_SetValue(&htim3, TIM_CHANNEL_4,LB_PWM1_Value);
		User_PWM_SetValue(&htim3, TIM_CHANNEL_3,LB_PWM2_Value);
		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,LB_PWM3_Value);
		User_PWM_SetValue(&htim4, TIM_CHANNEL_1,RB_PWM1_Value);
		User_PWM_SetValue(&htim3, TIM_CHANNEL_1,RB_PWM2_Value);
		User_PWM_SetValue(&htim3, TIM_CHANNEL_2,RB_PWM3_Value);
*/
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_4,LT_PWM1_Value);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,LT_PWM2_Value);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_3,LT_PWM3_Value);
//		User_PWM_SetValue(&htim4, TIM_CHANNEL_2,RT_PWM1_Value);
//		User_PWM_SetValue(&htim15, TIM_CHANNEL_1,RT_PWM2_Value);
//		User_PWM_SetValue(&htim15, TIM_CHANNEL_2,RT_PWM3_Value);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_4,LB_PWM1_Value);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,LB_PWM2_Value);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,LB_PWM3_Value);
//		User_PWM_SetValue(&htim4, TIM_CHANNEL_1,RB_PWM1_Value);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_1,RB_PWM2_Value);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_2,RB_PWM3_Value);
//		
//				User_PWM_SetValue(&htim3, TIM_CHANNEL_4,500);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,500);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_3,500);
//		User_PWM_SetValue(&htim4, TIM_CHANNEL_2,500);
//		User_PWM_SetValue(&htim15, TIM_CHANNEL_1,500);
//		User_PWM_SetValue(&htim15, TIM_CHANNEL_2,500);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_4,500);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,500);
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,500);
//		User_PWM_SetValue(&htim4, TIM_CHANNEL_1,500);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_1,500);
//		User_PWM_SetValue(&htim3, TIM_CHANNEL_2,500);
}

//遥控器通道捕获任务，通过中断实时获取遥控数据
/*Time8Channel1HighTime*/ //控制壁虎机拐弯方向，小于1300左拐，大于1700右拐，正常情况下为1500
/*Time8Channel2HighTime*/ //控制壁虎机前进、后退，小于1300后退，大于1300前进，正常情况下为1500
/*Time8Channel3HighTime*/ //控制壁虎机爬行速度，正常情况下为1500
/*Time8Channel4HighTime*/ //控制壁虎机灯的亮度，正常情况下为1500

void Redio_Ctrl()
{
	//通过采集遥控器数据，控制机器人方向、速度
	/*Time8Channel1HighTime*/ //控制壁虎机拐弯方向，小于1300左拐，大于1700右拐，正常情况下为1500
	if(Time8Channel4HighTime < 1300)
	{
//添加左拐控制代码
	}
	else if(Time8Channel4HighTime > 1700)
	{
//添加右拐控制代码	
	}
	
	if(Time8Channel3HighTime < 1300)
	{
//添加后退控制代码
	}
	else if(Time8Channel3HighTime > 1300)
	{
//添加前进控制代码	
	}

	if(Time8Channel2HighTime < 1300)
	{
//添加速度控制代码
	}
	else if(Time8Channel2HighTime < 1500)
	{
//添加速度控制代码	
	}
	else if(Time8Channel2HighTime < 1700)
	{
//添加速度控制代码	
	}
	else if(Time8Channel2HighTime <= 2000)
	{
//添加速度控制代码	
	}

	if(Time8Channel1HighTime < 1300)
	{
//添加LED控制代码
	if(LED_Value >=50)
	{
		LED_Value -= 50;
	}
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,LED_Value);
	HAL_Delay(10);

	}
	else if(Time8Channel1HighTime > 1700)
	{
//添加LED控制代码	
	if(LED_Value <=950)
	{
		LED_Value += 50;
	}
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,LED_Value);
	HAL_Delay(10);
	}

}

uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{	
    if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,buf)!=HAL_OK)return 0;//接收数据
	return FDCAN1_RxHeader.DataLength>>16;	
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t key;

	uint8_t canbuf[8];

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	All_PWM_Start();
	Set_All_PWM_Pulse(0);
	All_Capture_IT_Start();
	StartAngleInit();//关节矫正赋值
	InitRobotPosion();//robot位置初始化
	HAL_Delay(5000);

	setcurrentposition(RF,66,40,-37);
	setcurrentposition(RR,66,40,-37);
	setcurrentposition(LF,66,40,-37);
	setcurrentposition(LR,66,40,-37);

	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(USART_RX3_STA&0x8000)
//			{ 
//				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);	//发送数据时指示灯
////			printf("\r\n您发送的消息为:\r\n");
//				HAL_UART_Transmit(&huart4,(uint8_t*)USART_RX3_BUF,SEND4_LEN,1000);	//向无线模块发送接收到的数据
//				while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC)!=SET){};		//等待发送结束
////			printf("\r\n\r\n");//插入换行

//				USART_RX3_STA=0;
//				HAL_Delay(10);
//				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);	//发送完毕后指示灯灭
//			}
		if (operating_flag == 0)
				{
					InitRobotPosion();
				}
				else if (operating_flag==1)
				{
					Robot_Run_Line();
				}
				else if (operating_flag==2)
				{
					Robot_Run_Left();
				}
				else if (operating_flag==3)
				{
					Robot_Run_Right();
				}
  }
  /* USER CODE END 3 */
}


/********************************************************************************************************
*date: 2020-05-16
*function: onboard caculation
*author: NUAA baolin
********************************************************************************************************/ 
#include "reverse.h"

void setcurrentposition(int ft,float x,float y,float z)
{
	if (ft==RF)
	{cx[0]=x;
	 cy[0]=y;
	 cz[0]=z;
	}
	
	if (ft==RR)
	{cx[1]=x;
	 cy[1]=y;
	 cz[1]=z;
	}
	
	if (ft==LF)
	{cx[2]=x;
	 cy[2]=y;
	 cz[2]=z;
	}

	if (ft==LR)
	{cx[3]=x;
	 cy[3]=y;
	 cz[3]=z;
	}

}
void reverse4(float width,float length,float height)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	int t2=30;
	
	int step=100;
	

	for(int t=0;t<step + 1;t++)		
		{		
			for(int i=0;i<4;i++)
				{	
//					if (i==1||i==3)
//						{
//							length=-length;
//							}
						
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
						
					a[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					a[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					a[i][0]= (float) (a[i][0]/100);
					a[i][1]= (float) (a[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
					
					a[0][0]=-a[0][0];
					a[0][1]=-a[0][1];
				
					a[1][1]=-a[1][1];
				
					a[2][2]=-a[2][2];
				
					a[3][0]=-a[3][0];
					a[3][2]=-a[3][2];
				
					Angle(a[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(a[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(a[1][0]+KMGecko.StartAngle[RR_J3],RR_J3);
					Angle(a[1][1]+KMGecko.StartAngle[RR_J2],RR_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RR_J1],RR_J1);
						
					Angle(a[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(a[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(a[3][0]+KMGecko.StartAngle[LR_J3],LR_J3);
					Angle(a[3][1]+KMGecko.StartAngle[LR_J2],LR_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LR_J1],LR_J1);		
								
					HAL_Delay(t2);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RR,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LR,x[3],y[3],z[3]);

	
}
void reverse(int fn,float width,float length,float height)
	//length x 左右
  //width y 前进
{
	float x;
	float y;
	float z;
	int t1=30;
	
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	
	int step =100;
	
	
	if (fn==RF)
{
		for(int t=0;t<step + 1;t++)		
		{
			x=cx[0] + width / step * t;
			y=cy[0] + length / step * t;
			z=cz[0] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[0][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[0][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
							- atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			a[0][0]=(int) (a[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
			a[0][1]=(int) (a[0][1]/3.1415926*180*100);
			a[0][2]=(int) (a[0][2]/3.1415926*180*100);

			a[0][0]= -(float) (a[0][0]/100);
			a[0][1]= -(float) (a[0][1]/100);
			a[0][2]= (float) (a[0][2]/100);
			
			Angle(a[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
			Angle(a[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
			Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
			
			HAL_Delay(t1);
		}
			setcurrentposition(RF,x,y,z);
}
	
	if (fn==RR)
	{
		//length=-length;
		for(int t=0;t<step + 1;t++)		
		{
			x=cx[1] + width / step * t;
			y=cy[1] + length / step * t;
			z=cz[1] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[1][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[1][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					 - atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[1][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			a[1][0]=(int) (a[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
			a[1][1]=(int) (a[1][1]/3.1415926*180*100);
			a[1][2]=(int) (a[1][2]/3.1415926*180*100);

			a[1][0]= (float) (a[1][0]/100);
			a[1][1]= -(float) (a[1][1]/100);
			a[1][2]= (float) (a[1][2]/100);
			
			Angle(a[1][0]+KMGecko.StartAngle[RR_J3],RR_J3);
			Angle(a[1][1]+KMGecko.StartAngle[RR_J2],RR_J2);
			Angle(a[1][2]+KMGecko.StartAngle[RR_J1],RR_J1);
			
			HAL_Delay(t1);
		}
			setcurrentposition(RR,x,y,z);
	}
	if (fn==LF)
	{

		for(int t=0;t<step + 1;t++)		
		{
			x=cx[2] + width / step * t;
			y=cy[2] + length / step * t;
			z=cz[2] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[2][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[2][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					    - atan2(sqrt(x * x + z * z - L3 * L3) , y);
	  //a[0][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
		//				- atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[2][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			a[2][0]=(int) (a[2][0]/3.1415926*180*100);//lf alpha 角，最内侧
			a[2][1]=(int) (a[2][1]/3.1415926*180*100);
			a[2][2]=(int) (a[2][2]/3.1415926*180*100);

			a[2][0]= (float) (a[2][0]/100);
			a[2][1]= (float) (a[2][1]/100);
			a[2][2]= -(float) (a[2][2]/100);
			
			Angle(a[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
			Angle(a[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
			Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
			HAL_Delay(t1);
		}
			setcurrentposition(LF,x,y,z);
	}
	
	if (fn==LR)
	{
		//length=-length;
		for(int t=0;t<step + 1;t++)		
		{
			x=cx[3] + width / step * t;
			y=cy[3] + length / step * t;
			z=cz[3] - (height / (step * step) * t * t) + (2 * height / step * t);
			
			a[3][0] = asin(-L3 / sqrt(x * x + z * z) ) - atan2(z,x);
			a[3][1] = asin((x * x + y * y + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x * x +  y * y + z * z - L3 * L3)) )
					 - atan2(sqrt(x * x + z * z - L3 * L3) , y);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
			
			a[3][0]=(int) (a[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
			a[3][1]=(int) (a[3][1]/3.1415926*180*100);
			a[3][2]=(int) (a[3][2]/3.1415926*180*100);

			a[3][0]= -(float) (a[3][0]/100);
			a[3][1]= (float) (a[3][1]/100);
			a[3][2]= -(float) (a[3][2]/100);
			
			Angle(a[3][0]+KMGecko.StartAngle[LR_J3],LR_J3);
			Angle(a[3][1]+KMGecko.StartAngle[LR_J2],LR_J2);
			Angle(a[3][2]+KMGecko.StartAngle[LR_J1],LR_J1);
			
			HAL_Delay(t1);
		}
			setcurrentposition(LR,x,y,z);
	}
}


void reverse4R(float width,float length,float height)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	int t2=30;
	
	int step=100;
	

	for(int t=0;t<step + 1;t++)		
		{		
			
			for(int i=0;i<2;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
						
					a[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					a[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					a[i][0]= (float) (a[i][0]/100);
					a[i][1]= (float) (a[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(int i=2;i<4;i++)
				{	
					x[i]=cx[i] - width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
						
					a[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					a[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					a[i][0]= (float) (a[i][0]/100);
					a[i][1]= (float) (a[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}	
				
					a[0][0]=-a[0][0];
					a[0][1]=-a[0][1];
				
					a[1][1]=-a[1][1];
				
					a[2][2]=-a[2][2];
				
					a[3][0]=-a[3][0];
					a[3][2]=-a[3][2];
				
					Angle(a[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(a[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(a[1][0]+KMGecko.StartAngle[RR_J3],RR_J3);
					Angle(a[1][1]+KMGecko.StartAngle[RR_J2],RR_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RR_J1],RR_J1);
						
					Angle(a[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(a[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(a[3][0]+KMGecko.StartAngle[LR_J3],LR_J3);
					Angle(a[3][1]+KMGecko.StartAngle[LR_J2],LR_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LR_J1],LR_J1);		
								
					HAL_Delay(t2);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RR,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LR,x[3],y[3],z[3]);

	
}




void reverse4L(float width,float length,float height)
{
	float x[4]={0};
	float y[4]={0};
	float z[4]={0};
	
	float a[4][3] = {0};
	int t2=30;
	
	int step=100;
	

	for(int t=0;t<step + 1;t++)		
		{		
			for(int i=0;i<2;i++)
				{	
					x[i]=cx[i] - width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
						
					a[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					a[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					a[i][0]= (float) (a[i][0]/100);
					a[i][1]= (float) (a[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
				
			for(int i=2;i<4;i++)
				{	
					x[i]=cx[i] + width / step * t;
					y[i]=cy[i] + length / step * t;
					z[i]=cz[i] + height / step * t;
					
					a[i][0] = asin(-L3 / sqrt(x[i] * x[i] + z[i] * z[i]) ) - atan2(z[i],x[i]);
					a[i][1] = asin((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (x[i] * x[i] +  y[i] * y[i] + z[i] * z[i] - L3 * L3)) )
						    	 - atan2(sqrt(x[i] * x[i] + z[i] * z[i] - L3 * L3) , y[i]);
					a[i][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - x[i] * x[i] - y[i] * y[i] - z[i] * z[i]) / (2 * L1 * L2));
						
					a[i][0]=(int) (a[i][0]/3.1415926*180*100);//rf alpha 角，最内侧
					a[i][1]=(int) (a[i][1]/3.1415926*180*100);
					a[i][2]=(int) (a[i][2]/3.1415926*180*100);

					a[i][0]= (float) (a[i][0]/100);
					a[i][1]= (float) (a[i][1]/100);
					a[i][2]= (float) (a[i][2]/100);
				}
					
					a[0][0]=-a[0][0];//RF
					a[0][1]=-a[0][1];
				
					a[1][1]=-a[1][1];//RR
				
					a[2][2]=-a[2][2];//LF
				
					a[3][0]=-a[3][0];//LR
					a[3][2]=-a[3][2];
				
					Angle(a[0][0]+KMGecko.StartAngle[RF_J3],RF_J3);
					Angle(a[0][1]+KMGecko.StartAngle[RF_J2],RF_J2);
					Angle(a[0][2]+KMGecko.StartAngle[RF_J1],RF_J1);
						
					Angle(a[1][0]+KMGecko.StartAngle[RR_J3],RR_J3);
					Angle(a[1][1]+KMGecko.StartAngle[RR_J2],RR_J2);
					Angle(a[1][2]+KMGecko.StartAngle[RR_J1],RR_J1);
						
					Angle(a[2][0]+KMGecko.StartAngle[LF_J3],LF_J3);
					Angle(a[2][1]+KMGecko.StartAngle[LF_J2],LF_J2);
					Angle(a[2][2]+KMGecko.StartAngle[LF_J1],LF_J1);
						
					Angle(a[3][0]+KMGecko.StartAngle[LR_J3],LR_J3);
					Angle(a[3][1]+KMGecko.StartAngle[LR_J2],LR_J2);
					Angle(a[3][2]+KMGecko.StartAngle[LR_J1],LR_J1);		
								
					HAL_Delay(t2);
	}
					setcurrentposition(RF,x[0],y[0],z[0]);
					setcurrentposition(RR,x[1],y[1],z[1]);
					setcurrentposition(LF,x[2],y[2],z[2]);
					setcurrentposition(LR,x[3],y[3],z[3]);

	
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 100-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 5000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 100-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 200-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 10000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
	
		HAL_UART_Receive_IT(&huart4, (uint8_t *)aRxBuffer4, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer3, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* 串口接收数据子程序 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)//如果是串口3
	{
		if((USART_RX3_STA&0x8000)==0)//接收未完成
		{	
			switch(USART_RX3_STA)
			{
				case 0:
					if(aRxBuffer3[0]==0x55)//判断包头是否为0x55，否则重新接收
					{
						USART_RX3_BUF[USART_RX3_STA&0X3FFF]=aRxBuffer3[0] ;
						USART_RX3_STA++;
					}
					break;
					
				case 1:
					if(aRxBuffer3[0]==0x51)//判断包头是否为0x51，否则重新接收
					{
						USART_RX3_BUF[USART_RX3_STA&0X3FFF]=aRxBuffer3[0] ;
						USART_RX3_STA++;
					}else
					{
						USART_RX3_STA=0;
					}
					break;
					
				default :
					USART_RX3_BUF[USART_RX3_STA&0X3FFF]=aRxBuffer3[0] ;
					USART_RX3_STA++;
					if(USART_RX3_STA==(USART3_REC_LEN-1))	USART_RX3_STA|=0x8000;//接收完成	  
			}	
		if(USART_RX3_STA&0x8000)
			{ 
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);	//发送数据时指示灯
//			printf("\r\n您发送的消息为:\r\n");
				HAL_UART_Transmit(&huart4,(uint8_t*)USART_RX3_BUF,SEND4_LEN,1000);	//向无线模块发送接收到的数据
				while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC)!=SET){};		//等待发送结束
//			printf("\r\n\r\n");//插入换行

				USART_RX3_STA=0;
//				HAL_Delay(10);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);	//发送完毕后指示灯灭
			}

		}

	}	else
	{
			if(huart->Instance==UART4)//如果是串口4
		{
			if((USART_RX4_STA&0x8000)==0)//接收未完成
			{
				if(USART_RX4_STA&0x4000)//接收到了0x0d
				{
					if(aRxBuffer4[0]!=0x0a)USART_RX4_STA=0;//接收错误,重新开始
					else USART_RX4_STA|=0x8000;	//接收完成了 
				}
				else //还没收到0X0D
				{	
					if(aRxBuffer4[0]==0x0d)USART_RX4_STA|=0x4000;
					else
					{
						USART_RX4_BUF[USART_RX4_STA&0X3FFF]=aRxBuffer4[0] ;
						USART_RX4_STA++;
						if(USART_RX4_STA>(USART4_REC_LEN-1))USART_RX4_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}
		}

	}
	
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
//	if(htim->Instance == TIM8)
//	{
//		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_1,Time8Channel1HighTime,Time8Channel1Period,Time8Channel1Edge,Time8Channel1RisingTimeLast, Time8Channel1RisingTimeNow, Time8Channel1FallingTime);
//		}
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_2,Time8Channel2HighTime,Time8Channel2Period,Time8Channel2Edge,Time8Channel2RisingTimeLast, Time8Channel2RisingTimeNow, Time8Channel2FallingTime);
//		}
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_3,Time8Channel3HighTime,Time8Channel3Period,Time8Channel3Edge,Time8Channel3RisingTimeLast, Time8Channel3RisingTimeNow, Time8Channel3FallingTime);
//		}
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_4,Time8Channel4HighTime,Time8Channel4Period,Time8Channel4Edge,Time8Channel4RisingTimeLast, Time8Channel4RisingTimeNow, Time8Channel4FallingTime);
//		}
//	}
//	else if(htim->Instance == TIM4)
//	{
//		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_3,Time4Channel3HighTime,Time4Channel3Period,Time4Channel3Edge,Time4Channel3RisingTimeLast, Time4Channel3RisingTimeNow, Time4Channel3FallingTime);
//		}
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//		{	
//			Read_Radio_Channel(htim,TIM_CHANNEL_4,Time4Channel4HighTime,Time4Channel4Period,Time4Channel4Edge,Time4Channel4RisingTimeLast, Time4Channel4RisingTimeNow, Time4Channel4FallingTime);
//		}
//	}
	if(htim->Instance == TIM8)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if(Time8Channel1Edge == 0)
			{
				Time8Channel1RisingTimeNow = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
				Time8Channel1Edge = 1;
				if(Time8Channel1RisingTimeLast == 0)
				{
					Time8Channel1Period = 0;
				}
				else
				{
					if(Time8Channel1RisingTimeNow > Time8Channel1RisingTimeLast)
					{
						Time8Channel1Period = Time8Channel1RisingTimeNow - Time8Channel1RisingTimeLast;
					}
					else
					{
						Time8Channel1Period = Time8Channel1RisingTimeNow + 5000 - Time8Channel1RisingTimeLast + 1;
					}
				}
				Time8Channel1RisingTimeLast = Time8Channel1RisingTimeNow;
			}
			else if(Time8Channel1Edge == 1)
			{
				Time8Channel1FallingTime = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);	
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
			
				if(Time8Channel1FallingTime < Time8Channel1RisingTimeNow)
				{
					Time8Channel1HighTime = Time8Channel1FallingTime + 5000 - Time8Channel1RisingTimeNow;
				}
				else if(Time8Channel1FallingTime > Time8Channel1RisingTimeNow)
				{
					Time8Channel1HighTime = Time8Channel1FallingTime - Time8Channel1RisingTimeNow;
				}
				else
				{
					Time8Channel1HighTime = (Time8Channel1HighTime>500)?1000:0;
				}

			Time8Channel1Edge = 0;
			}
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(Time8Channel2Edge == 0)
			{
				Time8Channel2RisingTimeNow = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
				Time8Channel2Edge = 1;
				if(Time8Channel2RisingTimeLast == 0)
				{
					Time8Channel2Period = 0;
				}
				else
				{
					if(Time8Channel2RisingTimeNow > Time8Channel2RisingTimeLast)
					{
						Time8Channel2Period = Time8Channel2RisingTimeNow - Time8Channel2RisingTimeLast;
					}
					else
					{
						Time8Channel2Period = Time8Channel2RisingTimeNow + 5000 - Time8Channel2RisingTimeLast + 1;
					}
				}
				Time8Channel2RisingTimeLast = Time8Channel2RisingTimeNow;
			}
			else if(Time8Channel2Edge == 1)
			{
				Time8Channel2FallingTime = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);	
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
			
				if(Time8Channel2FallingTime < Time8Channel2RisingTimeNow)
				{
					Time8Channel2HighTime = Time8Channel2FallingTime + 5000 - Time8Channel2RisingTimeNow;
				}
				else if(Time8Channel2FallingTime > Time8Channel2RisingTimeNow)
				{
					Time8Channel2HighTime = Time8Channel2FallingTime - Time8Channel2RisingTimeNow;
				}
				else
				{
					Time8Channel2HighTime = (Time8Channel2HighTime>500)?1000:0;
				}

			Time8Channel2Edge = 0;
			}
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(Time8Channel3Edge == 0)
			{
				Time8Channel3RisingTimeNow = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
				Time8Channel3Edge = 1;
				if(Time8Channel3RisingTimeLast == 0)
				{
					Time8Channel3Period = 0;
				}
				else
				{
					if(Time8Channel3RisingTimeNow > Time8Channel3RisingTimeLast)
					{
						Time8Channel3Period = Time8Channel3RisingTimeNow - Time8Channel3RisingTimeLast;
					}
					else
					{
						Time8Channel3Period = Time8Channel3RisingTimeNow + 5000 - Time8Channel3RisingTimeLast + 1;
					}
				}
				Time8Channel3RisingTimeLast = Time8Channel3RisingTimeNow;
			}
			else if(Time8Channel3Edge == 1)
			{
				Time8Channel3FallingTime = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3);	
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
			
				if(Time8Channel3FallingTime < Time8Channel3RisingTimeNow)
				{
					Time8Channel3HighTime = Time8Channel3FallingTime + 5000 - Time8Channel3RisingTimeNow;
				}
				else if(Time8Channel3FallingTime > Time8Channel3RisingTimeNow)
				{
					Time8Channel3HighTime = Time8Channel3FallingTime - Time8Channel3RisingTimeNow;
				}
				else
				{
					Time8Channel3HighTime = (Time8Channel3HighTime>500)?1000:0;
				}

			Time8Channel3Edge = 0;
			}
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(Time8Channel4Edge == 0)
			{
				Time8Channel4RisingTimeNow = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4);
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
				Time8Channel4Edge = 1;
				if(Time8Channel4RisingTimeLast == 0)
				{
					Time8Channel4Period = 0;
				}
				else
				{
					if(Time8Channel4RisingTimeNow > Time8Channel4RisingTimeLast)
					{
						Time8Channel4Period = Time8Channel4RisingTimeNow - Time8Channel4RisingTimeLast;
					}
					else
					{
						Time8Channel4Period = Time8Channel4RisingTimeNow + 5000 - Time8Channel4RisingTimeLast + 1;
					}
				}
				Time8Channel4RisingTimeLast = Time8Channel4RisingTimeNow;
			}
			else if(Time8Channel4Edge == 1)
			{
				Time8Channel4FallingTime = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4);	
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
			
				if(Time8Channel4FallingTime < Time8Channel4RisingTimeNow)
				{
					Time8Channel4HighTime = Time8Channel4FallingTime + 5000 - Time8Channel4RisingTimeNow;
				}
				else if(Time8Channel4FallingTime > Time8Channel4RisingTimeNow)
				{
					Time8Channel4HighTime = Time8Channel4FallingTime - Time8Channel4RisingTimeNow;
				}
				else
				{
					Time8Channel4HighTime = (Time8Channel4HighTime>500)?1000:0;
				}

			Time8Channel4Edge = 0;
			}
		}
	}
if (Time8Channel3HighTime ==2000)
{
	operating_flag = 1; // run forward
}
else if (Time8Channel1HighTime ==1000)//&&(Time8Channel3HighTime == 1500)&&(Time8Channel4HighTime == 1500))
{
	operating_flag = 2; // left
}
else if (Time8Channel1HighTime ==2000 )//&&(Time8Channel3HighTime == 1500)&&(Time8Channel4HighTime == 1500))
{
	operating_flag = 3; // right
}
else
{
	operating_flag = 0; // pause
}
}
/********************************************************************************************************
*date：2018-7-3
*author：NUAA google
*function:tranfer angle to arr numerical value
*input:angle:[-90,90],footnumber: //LF_J1 LR_J1 RF_J1 RR_J1
*																  //LF_J2 LR_J2 RF_J2 RR_J2
*                                 //LF_J3 LR_J3 RF_J3 RR_J3
*                                 //ARM_J1 ARM_J2 ARM_J3 WAIST
*output:ArrValue
*remarks：转换角度变为占空比数值
********************************************************************************************************/


void Angle(float angle,int8_t footnumber)
{
	int16_t ArrValue = 0;
	//ArrValue = 250 + (int32_t) (angle *0.18);
	ArrValue = angle * 5 + 750;
	//ArrValue = (int32_t) (angle * 5.56 + 750)	;
	if(footnumber == LF_J1)
	{
		User_PWM_SetValue(&htim4, TIM_CHANNEL_2,ArrValue);
	}
		else if(footnumber == LR_J1)
	{
		User_PWM_SetValue(&htim3, TIM_CHANNEL_1,ArrValue);
	}
		else if(footnumber == RF_J1)
	{
		User_PWM_SetValue(&htim3, TIM_CHANNEL_3,ArrValue);
	}
		else if(footnumber == RR_J1)
	{
		User_PWM_SetValue(&htim1, TIM_CHANNEL_2,ArrValue);
	}
		else if(footnumber == LF_J2)
	{
		User_PWM_SetValue(&htim15, TIM_CHANNEL_1,ArrValue);
	}
		else if(footnumber == LR_J2)
	{
		User_PWM_SetValue(&htim3, TIM_CHANNEL_2,ArrValue);
	}
		else if(footnumber == RF_J2)
	{
		User_PWM_SetValue(&htim3, TIM_CHANNEL_4,ArrValue);
	}
		else if(footnumber == RR_J2)
	{
		User_PWM_SetValue(&htim1, TIM_CHANNEL_4,ArrValue);
	}
		else if(footnumber == LF_J3)
	{
		User_PWM_SetValue(&htim15, TIM_CHANNEL_2,ArrValue);
	}
		else if(footnumber == LR_J3)
	{
		User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
	}
		else if(footnumber == RF_J3)
	{
		User_PWM_SetValue(&htim1, TIM_CHANNEL_1,ArrValue);
	}
		else if(footnumber == RR_J3)
	{
		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
	}
}

void Robot_Run_Line(void)
{
	int x=0;//左右length
	int y=30;//前进width
	int z=30;//高度height
	
	reverse(RF,0,y,z);
	reverse(RF,0,0,-z);
	
	reverse(RR,0,y,z);
	reverse(RR,0,0,-z);
	
	reverse4(0,-y/2,0);
	
	reverse(LF,0,y,z);
	reverse(LF,0,0,-z);
	
	reverse(LR,0,y,z);
	reverse(LR,0,0,-z);
	
	reverse4(0,-y/2,0);
	
}

void Robot_Run_Left(void)
	{
	int x=15;//左右length
	int y=10;//前进width
	int z=30;//高度height
		
	reverse(LF,x,y,z);
	reverse(LF,0,0,-z);
	
	reverse(LR,x,y,z);
	reverse(LR,0,0,-z);	
		
	reverse4L(-x/2,-y/2,0);
			
	reverse(RF,-x,y,z);
	reverse(RF,0,0,-z);
	
	reverse(RR,-x,y,z);
	reverse(RR,0,0,-z);

	reverse4L(-x/2,-y/2,0);

}

void Robot_Run_Right(void)
{
	int x=16;//左右length
	int y=10;//前进width
	int z=30;//高度height
		
	reverse(RF,x,y,z);
	reverse(RF,0,0,-z);
	
	reverse(RR,x,y,z);
	reverse(RR,0,0,-z);	
		
	reverse4R(-x/2,-y/2,0);
			
	reverse(LF,-x,y,z);
	reverse(LF,0,0,-z);
	
	reverse(LR,-x,y,z);
	reverse(LR,0,0,-z);

	reverse4R(-x/2,-y/2,0);
}

/*初始位置矫零*/

void StartAngleInit(void)
{
			/**直角初始位置**/
	//关节1、2、3分别为踝关节、膝关节、髋关节
	//number 3
	KMGecko.StartAngle[RF_J3] = 10; //
	KMGecko.StartAngle[RF_J2] = 30; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 45; //52// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RR_J3] = -7;//-10//-25;	
	KMGecko.StartAngle[RR_J2] = 56;//50;	small figure turning to the inside
	KMGecko.StartAngle[RR_J1] = 55;//30;large figure turning to the inside
  //number 0
	KMGecko.StartAngle[LF_J3] = -20;//-10
	KMGecko.StartAngle[LF_J2] = 25;//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -56;//-58 small figure turning to the inside

	//number 1
	KMGecko.StartAngle[LR_J3] = 7;//10///-30;//原值为-30
	KMGecko.StartAngle[LR_J2] = 23;//0 large figure turning to the inside
	KMGecko.StartAngle[LR_J1] = -57;//;small figure turning to the inside

}
/********************************************************************************************************
*date: 2018-10-10
*function: Adjust Robot to initial posion and set px[],py[],pz[] as zero
*author: NUAA google
********************************************************************************************************/ 

void InitRobotPosion(void)
{
//	int i =0;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_x0[i] = 65;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_y0[i] = 32;
//	for(i = 0; i<LEGNUM;i++)
//	KMGecko.init_z0[i] = -32;

					/**直角的初始位置**/
	Angle(KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(KMGecko.StartAngle[LF_J1],LF_J1);
	Angle(KMGecko.StartAngle[LF_J2],LF_J2);
	
	Angle(KMGecko.StartAngle[RF_J1],RF_J1);
	Angle(KMGecko.StartAngle[RF_J2],RF_J2);		
	Angle(KMGecko.StartAngle[RF_J3],RF_J3);
	
	Angle(KMGecko.StartAngle[RR_J1],RR_J1);
	Angle(KMGecko.StartAngle[RR_J2],RR_J2);
	Angle(KMGecko.StartAngle[RR_J3],RR_J3);

	Angle(KMGecko.StartAngle[LR_J1],LR_J1);
	Angle(KMGecko.StartAngle[LR_J2],LR_J2);
	Angle(KMGecko.StartAngle[LR_J3],LR_J3);

					/**钝角的初始位置**/
//	Angle(KMGecko.StartAngle[LF_J3]+LF_J3_Line[0],LF_J3);
//	Angle(KMGecko.StartAngle[LF_J1]+LF_J1_Line[0],LF_J1);
//	Angle(KMGecko.StartAngle[LF_J2]+LF_J2_Line[0],LF_J2);
//	
//	Angle(KMGecko.StartAngle[RF_J1]+RF_J1_Line[0],RF_J1);
//	Angle(KMGecko.StartAngle[RF_J2]+RF_J2_Line[0],RF_J2);		
//	Angle(KMGecko.StartAngle[RF_J3]+RF_J3_Line[0],RF_J3);
//	
//	Angle(KMGecko.StartAngle[RR_J1]-RR_J1_Line[0],RR_J1);
//	Angle(KMGecko.StartAngle[RR_J2]-RR_J2_Line[0],RR_J2);
//	Angle(KMGecko.StartAngle[RR_J3]+RR_J3_Line[0],RR_J3);

//	Angle(KMGecko.StartAngle[LR_J1]-LR_J1_Line[0],LR_J1);
//	Angle(KMGecko.StartAngle[LR_J2]-LR_J2_Line[0],LR_J2);
//	Angle(KMGecko.StartAngle[LR_J3]+LR_J3_Line[0],LR_J3);
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
  while(1) 
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
