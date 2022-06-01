/*****************************************************************************************************
  *																																														 		 *
  *              8888888b.    888    888    .d8888b.    8888888b.    88888888888                   *
  *              888  "Y88b   888    888   d88P  Y88b   888  "Y88b       888                       *
  *              888    888   888    888   888    888   888    888       888                       *
  *              888    888   8888888888   888          888    888       888                       *
  *              888    888   888    888   888          888    888       888                       *
  *              888    888   888    888   888    888   888    888       888                       *
  *              888  .d88P   888    888   Y88b  d88P   888  .d88P       888                       *
  *              8888888P"    888    888    "Y8888P"    8888888P"        888                       *
  *      																																											     *
  *      																																											     *
  *      																																											     *
  *                                 d888    .d8888b.  888888b.                                     *
  *                                d8888   d88P  Y88b 888  "88b                                    *
  *                                  888          888 888  .88P                                    *
  *                                  888        .d88P 8888888K.                                    *
  *                                  888    .od888P"  888  "Y88b                                   *
  *                                  888   d88P"      888    888                                   *
  *                                  888   888"       888   d88P                                   *
  *                                8888888 888888888  8888888P"                                    *
  *																																														     *
******************************************************************************************************
*/


/************************************************************************************
	*		    														   								   A    B                 *
	*		    																																	      *
  *       release DC1 --> Encoder  					  TIM3         PB4  PB5               *
  *       release DC2 --> Encoder   					TIM2         PA15 PB3               *
  *       release DC3 --> Encoder    					TIM5         PA0  PA1               *
	*		    																																	      *
	*		    																																	      *
*************************************************************************************
	*																																							  *
	*														ADC1 Channel 2							 PA2	                  *
	*														ADC1 Channel 3							 PA3	                  *
	*														ADC1 Channel 4							 PA4	                  *
	*																																							  *
*************************************************************************************
	*																																							  *
	*						PWM_1						TIM 11											 PB9									  *					
	*						PWM_2						TIM 13											 PA6									  *		
	*						PWM_3						TIM 14											 PA7	                  *
	*																																							  *
*************************************************************************************
	*																												 IN1  IN2               *
	*																																								*
  *              release DC1 -->                           PD1  PD2               *
  *              release DC2 -->                           PD3  PD4               *
  *              release DC3 -->                           PD5  PD6               *
	*																																								*
	*																												 1    0  --> DEM LEN    *
	*																												 0    1  --> DEM XUONG  *
	*																																								*
*************************************************************************************
*/

/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) LEY3HC, 2020
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
 
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_delay.h"

#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_adc.h"

#include <stdio.h>
#include <stdlib.h>

#include "arm_math.h"

#define constrain(x, lower_limit, upper_limit)   ({ __typeof__ (x) _x = (x); __typeof__ (lower_limit) _lower_limit = (lower_limit);  __typeof__ (upper_limit) _upper_limit = (upper_limit);  min(max(_x, _lower_limit), _upper_limit); })
     
#define min(a, b)  ({  __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _b : _a; })

#define max(a, b) 	({  __typeof__ (a) _a = (a);	__typeof__ (b) _b = (b); _a > _b ? _a : _b; })


TIM_TimeBaseInitTypeDef  					TIM_TimeBaseStructure;
TIM_ICInitTypeDef        					TIM_ICInitStructure;

TM_PWM_TIM_t TIM11_Data;
TM_PWM_TIM_t TIM13_Data;
TM_PWM_TIM_t TIM14_Data;

uint16_t PrescalerValue = 0;


void PID_DY_UPDATE(float* Save_PID_Feeback, int32_t Target, int32_t Current, float kp, float ki, float kd);                // release 
void Move_Down_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);                     // release
void Move_Up_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);                      // release
void Turn_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);                        // release
void ENCODER_Read_Release(int32_t* Save_vitri_feedback, uint32_t* Dir, uint32_t* Cnt, TIM_TypeDef* TIMx);              // release 
void ADC_READ_AND_CONVERT(int32_t* Save_Feedback, ADC_TypeDef* ADCx, uint8_t channel);                                //release 

void Encoder1_Tim3_Init(void);         // release DC1 --> Encoder  TIM3 PB4 PB5
void Encoder2_Tim2_Init(void);        // release DC2 --> Encoder  TIM2 PA15 PB3
void Encoder3_Tim5_Init(void);       // release DC3 --> Encoder  TIM5 PA0 PA1
void ALL_ENCODER_INIT(void);        // release ECD --> All      ENCODER
void ALL_GPIO_INIT(void);          // release GPIO--> All      GPIO
void ALL_PWM_INIT(void);          // release PWM --> All      PWM
void ALL_ADC_INIT(void);         // release ADC --> All      ADC

 
void GPIO_PinMode(GPIO_TypeDef *GPIOx, uint32_t CLOCK, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType, GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd); //release
void timer4_config(void);       //  ngat 0.005 s  

typedef struct {
    float    Save_pid_feeback;
    int32_t  Save_vitri_encoder_feedback;
	  int32_t  Save_vitri_adc_feedback;
		uint32_t Save_Cnt_feedback;
		uint32_t Save_Dir_feedback;				 
}SAVE;

static SAVE                   DC1, DC2, DC3 ;


typedef struct {
    float    kp;
		float    ki;
	  float    kd;
}PID_Def;

static __IO PID_Def           PID_DC1, PID_DC2, PID_DC3;



int main(void) {
	/* Initialize Constant Kp Ki Kd  */
	PID_DC1.kp = 0.1;
	PID_DC1.ki = 0;
	PID_DC1.kd = 0;
	/* Initialize Constant Kp Ki Kd  */
	PID_DC2.kp = 0.1;
	PID_DC2.ki = 0;
	PID_DC2.kd = 0;
	/* Initialize Constant Kp Ki Kd  */
	PID_DC3.kp = 0.1;
	PID_DC3.ki = 0;
	PID_DC3.kd = 0;
	
	/* Initialize system */
	SystemInit();
	/* Initialize delay functions */
	TM_DELAY_Init();
	/* Initialize pins PD12, PD13 */
  TM_GPIO_Init(GPIOD, GPIO_Pin_12 | GPIO_Pin_13, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	/* Initialize Interupt */
	timer4_config();
  /* Initialize ADC */
  ALL_ADC_INIT();
	/* Initialize ENCODER */
	ALL_ENCODER_INIT();
	/* Initialize GPIO */
  ALL_GPIO_INIT();
	/* Initialize PWM */
	ALL_PWM_INIT();


	while (1) {

ADC_READ_AND_CONVERT(&DC1.Save_vitri_adc_feedback, ADC1, ADC_Channel_2);
ADC_READ_AND_CONVERT(&DC2.Save_vitri_adc_feedback, ADC1, ADC_Channel_3);
ADC_READ_AND_CONVERT(&DC3.Save_vitri_adc_feedback, ADC1, ADC_Channel_4);

ENCODER_Read_Release(&DC1.Save_vitri_encoder_feedback, &DC1.Save_Dir_feedback, &DC1.Save_Dir_feedback, TIM3);
ENCODER_Read_Release(&DC2.Save_vitri_encoder_feedback, &DC2.Save_Dir_feedback, &DC2.Save_Dir_feedback, TIM2);
ENCODER_Read_Release(&DC3.Save_vitri_encoder_feedback, &DC3.Save_Dir_feedback, &DC3.Save_Dir_feedback, TIM5);
		
//PID_DY_UPDATE(&DC1.Save_pid_feeback, DC1.Save_vitri_adc_feedback, DC1.Save_vitri_encoder_feedback, PID_DC1.kp, PID_DC1.ki, PID_DC1.kd); 

Turn_Release(1, &TIM11_Data, TM_PWM_Channel_1, DC1.Save_pid_feeback);
Turn_Release(2, &TIM13_Data, TM_PWM_Channel_1, DC2.Save_pid_feeback); 
Turn_Release(3, &TIM14_Data, TM_PWM_Channel_1, DC3.Save_pid_feeback); 		

	}  // while 
}  // main



/*
*****************************************************************************************
*												              PID																								*			
*																																												*
*****************************************************************************************
*/
void PID_DY_UPDATE(float* Save_PID_Feeback, int32_t Target, int32_t Current, float kp, float ki, float kd)
{
		//loi = vitrimm - vitri;
	
		static int32_t Bias, Last_Bias;
	  static float Pwm;
	
		int32_t dloi, iloi;
	
    Bias = Target - Current;    
	
		dloi = Bias - Last_Bias;
	
		iloi += Bias;
	
		iloi = constrain(iloi, -200, 200);

	  Last_Bias = Bias;
	
    Pwm = kp*Bias + ki*iloi+kd*dloi; 
    	                        
		Pwm = constrain(Pwm, -100, 100);
	
		*Save_PID_Feeback = Pwm ;
}


/*
*******************************************************************************************************
  *																	ADC_READ_AND_CONVERT                                             *
	*																																																 	 *
	*							r_read = TM_ADC_Read(ADCx, channel) ;     // ADC cua bien tro   // chan PA0				   *
	*																																																   *
	*							ratio1 = ((TM_ADC_Read(ADCx, channel)) / 4095);   // 4095 la 12 bit		               *
	*																																																   *
	*							convert_to_pwm1_new = ((TM_ADC_Read(ADCx, channel)) / 4095) * 1500 ; 							   *				
	*																																																   *
	*				bien tro quay gan mot vong thi dong co cung quay duoc 1 vong, quy ra 	xung cua dong co	   *
	*																																																   *
	*							1500 la so xung cua dong co quay 1 vong -> can chinh lai cho dung										 *																													     *
	*																																																   *				
	*																     																															 *
*******************************************************************************************************            
*/

void ADC_READ_AND_CONVERT(int32_t* Save_Feedback, ADC_TypeDef* ADCx, uint8_t channel)
{			
	
	*Save_Feedback = (int32_t)(((TM_ADC_Read(ADCx, channel)) / 4095) * 1500); 
	
}





/********************************************************************************
	*  	  	                                                                     *
	*				 Dir: huong cua dong co -> 1 :dem len, 0 : dem xuong                 *
	*                                                                            *
	*       *Cnt = TIMx->CNT;  <==> TIM_GetCounter(TIMx);  ->  dem encoder       *
	*                                                                            *
	*				*Save_vitri_feedback -> con tro toi vi tri doc duoc                  *
	*                                                                            *
*********************************************************************************
*/
void ENCODER_Read_Release(int32_t* Save_vitri_feedback, uint32_t* Dir, uint32_t* Cnt, TIM_TypeDef* TIMx)     
{
	
  *Dir = ((TIMx->CR1 & TIM_CR1_DIR)? (0) : 1);         // huong cua dong co //1 la dem len, 0 la dem xuong

  *Cnt = TIMx->CNT;     //TIM_GetCounter(TIMx);       // dem encoder
	
	*Save_vitri_feedback = *Cnt - 30000;              // dem tu 30000, len hoac xuong
	
}




/*
*********************************************************************************
	*  										Encoder_1  TIM3 PB4 PB5                                *
	*                                                                            *
*********************************************************************************
*/
void Encoder1_Tim3_Init(void)            
{   
    //Encoder  TIM3 PB4 PB5             
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    GPIO_PinMode(GPIOB, RCC_AHB1Periph_GPIOB,
                 GPIO_Pin_4 | GPIO_Pin_5, GPIO_Mode_AF,
                 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM3, 30000);
    TIM_Cmd(TIM3, ENABLE);
}





/********************************************************************************
	*  										Encoder_2  TIM2 PA15 PB3                               *
	*                                                                            *
*********************************************************************************
*/
void Encoder2_Tim2_Init(void)            
{   
    //Encoder 2 TIM2 PA15 PB3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOA,
                 GPIO_Pin_15, GPIO_Mode_AF,
                 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
    GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOB,
                 GPIO_Pin_3, GPIO_Mode_AF,
                 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM2, 30000);
    TIM_Cmd(TIM2, ENABLE);
		
}




/********************************************************************************
	*  										Encoder_3  TIM5 PA0 PA1                                *
	*                                                                            *
*********************************************************************************
*/
void Encoder3_Tim5_Init(void)            
{   
    //Encoder 3 TIM5 PA0 PA1	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOA,
                 GPIO_Pin_0 | GPIO_Pin_1, GPIO_Mode_AF,
                 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM5, 30000);
    TIM_Cmd(TIM5, ENABLE);
		
}




/*
*********************************************************************************
	*                              INIT ADC                                     *
	*  						                                                              *
	*                    ADC1 Channel 2							 PA2                        *                          
  *                    ADC1 Channel 3							 PA3                        *
  *                    ADC1 Channel 4							 PA4                        *
  *                                                                           *
*********************************************************************************
*/
void ALL_ADC_INIT(void)
{
	/* Initialize ADC1 on channel 2, this is pin PA2 */
		TM_ADC_Init(ADC1, ADC_Channel_2);
	/* Initialize ADC1 on channel 3, this is pin PA3 */
		TM_ADC_Init(ADC1, ADC_Channel_3);
	/* Initialize ADC1 on channel 4, this is pin PA4 */
		TM_ADC_Init(ADC1, ADC_Channel_4);
}





/*
*********************************************************************************
	*                                                                           *
	*  						             INIT ALL ENCODER                                 *
	*                                                                           *                          
*********************************************************************************
*/
void ALL_ENCODER_INIT(void)
{
	/* DC1 --> Encoder  TIM3  PB4  PB5   */
		   Encoder1_Tim3_Init();
 /* DC2 --> Encoder  TIM2  PA15 PB3   */    
			 Encoder2_Tim2_Init();
/* DC3 --> Encoder  TIM5  PA0  PA1   */
			Encoder3_Tim5_Init();
}

/*
*************************************************************************************
	*                           INIT ALL PWM                                        *
	*																																							  *
	*						PWM_1						TIM 11											 PB9									  *					
	*						PWM_2						TIM 13											 PA6									  *		
	*						PWM_3						TIM 14											 PA7	                  *
	*																																							  *
*************************************************************************************
*/
void ALL_PWM_INIT(void)
{

	
	/* Set PWM to 1KHz frequency on timer TIM11 */
	/* 1KHz = ?ms = ?us */
	TM_PWM_InitTimer(TIM11, &TIM11_Data, 1000);
	
	/* Initialize PWM on TIM11, Channel 1 and PinsPack 1 =  PB9*/
	TM_PWM_InitChannel(&TIM11_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_1);            
	/* Set default duty cycle */
	TM_PWM_SetChannelPercent(&TIM11_Data, TM_PWM_Channel_1, 0);
	
	/* Set PWM to 1KHz frequency on timer TIM11 */
	/* 1KHz = ?ms = ?us */
	TM_PWM_InitTimer(TIM13, &TIM13_Data, 1000);
	
	/* Initialize PWM on TIM13, Channel 1 and PinsPack 1 =  PA6*/
	TM_PWM_InitChannel(&TIM13_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_1);            
	/* Set default duty cycle */
	TM_PWM_SetChannelPercent(&TIM13_Data, TM_PWM_Channel_1, 0);
	
	/* Set PWM to 1KHz frequency on timer TIM11 */
	/* 1KHz = ?ms = ?us */
	TM_PWM_InitTimer(TIM14, &TIM14_Data, 1000);
	
	/* Initialize PWM on TIM14, Channel 1 and PinsPack 1 =  PA7*/
	TM_PWM_InitChannel(&TIM14_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_1);            
	/* Set default duty cycle */
	TM_PWM_SetChannelPercent(&TIM14_Data, TM_PWM_Channel_1, 0);
}


/*
*************************************************************************************
	*														INIT ALL GPIO 	                                    *
	*                                                                               *
	*																												 IN1  IN2               *
	*																																								*
  *              release DC1 -->                           PD1  PD2               *
  *              release DC2 -->                           PD3  PD4               *
  *              release DC3 -->                           PD5  PD6               *
	*																																								*
	*																												 1    0  --> DEM LEN    *
	*																												 0    1  --> DEM XUONG  *
	*																																								*
*************************************************************************************
*/
void ALL_GPIO_INIT(void)
{
		/* Init pins PD5, PD6 */
    TM_GPIO_Init(GPIOD, GPIO_Pin_5 | GPIO_Pin_6, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		/* Init pins PD3, PD4 */
    TM_GPIO_Init(GPIOD, GPIO_Pin_3 | GPIO_Pin_4, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		/* Init pins PD1, PD2 */
    TM_GPIO_Init(GPIOD, GPIO_Pin_1 | GPIO_Pin_2, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	
}

/*
*****************************************************************************************
*												  FUNCTION CHO DONG CO QUAY THUAN																*			
*																																												*
*****************************************************************************************
*/
void Move_Up_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent)
{
	   switch(DCx)
	 {
	   case 1:  	
			 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_1);   //   dem len 
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_2);   //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
		 case 2: 
			 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_3);   //   dem len 
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_4);   //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
		 
		 case 3:
		 	 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_5);   //   dem len 
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_6);   //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
	 }
 
}



/*
*****************************************************************************************
*												  FUNCTION CHO DONG CO QUAY THUAN																*			
*																																												*
*****************************************************************************************
*/
void Move_Down_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent)
{
	   switch(DCx)
	 {
	   case 1:  	
			 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_2);     //   dem xuong 
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_1);     //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
		 case 2: 
			 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_4);     //   dem xuong
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_3);     //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
		 
		 case 3:
		 	 TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_6);     //   dem xuong 
       TM_GPIO_SetPinLow(GPIOD, GPIO_PIN_5);     //
	     TM_PWM_SetChannelPercent(TIM_Data, Channel, percent); 
		 break;
	 }
 
}

/*
*****************************************************************************************
*											FUNCTION XAC DINH PERCENT - OR +  																*			
*																																												*
*****************************************************************************************
*/
void Turn_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent)
{

	if(percent > 0)
		{
			Move_Up_Release(DCx, TIM_Data, Channel, percent);		
		}
		
	if(percent < 0)
		{
			Move_Down_Release(DCx, TIM_Data, Channel,(-percent));		
		}

} 



void GPIO_PinMode(GPIO_TypeDef *GPIOx, uint32_t CLOCK, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType, GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(CLOCK, ENABLE);      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;   
    GPIO_Init(GPIOx, &GPIO_InitStructure);      
    GPIO_ResetBits(GPIOx, GPIO_Pin);
    return;
}


void timer4_config(void)    /// cau hinh cho ngat 0.005 s
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 1000000) - 1;
	
	
	TIM_DeInit(TIM4);
	/* Time base configuration */ 
  TIM_TimeBaseStructure.TIM_Period = 9999;//100hz
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//1mhz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	//TIM_ARRPreloadConfig(TIM4,TIM_UpdateSource_Regular);
	
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	
	TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);
	TIM_Cmd (TIM4, ENABLE);
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)                                    // ngat 0.005 s cho tinh toan PID
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
			PID_DY_UPDATE(&DC1.Save_pid_feeback, DC1.Save_vitri_adc_feedback, DC1.Save_vitri_encoder_feedback, PID_DC1.kp, PID_DC1.ki, PID_DC1.kd);
			PID_DY_UPDATE(&DC2.Save_pid_feeback, DC2.Save_vitri_adc_feedback, DC1.Save_vitri_encoder_feedback, PID_DC2.kp, PID_DC2.ki, PID_DC2.kd);
			PID_DY_UPDATE(&DC3.Save_pid_feeback, DC3.Save_vitri_adc_feedback, DC1.Save_vitri_encoder_feedback, PID_DC3.kp, PID_DC3.ki, PID_DC3.kd);
			TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_12);               // cho den(onboard) sang biet dc ham ngat co chay ko (ko quan tam cung dc)     
			TM_GPIO_SetPinHigh(GPIOD, GPIO_PIN_13);                // cho den(onboard) sang biet dc ham ngat co chay ko (ko quan tam cung dc)

		}
}

