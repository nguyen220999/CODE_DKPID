#include "stm32f10x.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>

/*
truyen nhan du lieu tu GUI thông qua USART2_Pin PA2-TX, PA3-RX

xuat xung PWM dieu khien dong co: Timer-1 Pin PA8, Timer-1  Pin PA9

doc encoder: Timer-2 Pin PA0, Pin PA1

truyen nhan du lieu: Timer-4

dong co su dung encoder 334 xung

thoi gian lay mau 15ms

hoan thanh ngay 26/6/2020-BK.HCM

Tran Hoang Nguyen-1712395-PFIEV-CDT

*/
union ReceiveData
{
	double Receive;
	unsigned char ReceiveByte[8];
}nhan;

/*Khoi tao bien cau hinh*/
GPIO_InitTypeDef			GPIO_InitStructure;
USART_InitTypeDef			USART_InitStructure;
NVIC_InitTypeDef			NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef       TIM_OCInitStructure;
TIM_ICInitTypeDef       TIM_ICInitStructure;


void delay(uint32_t cout);
void Encoder_Config(void);

void TIMbase_Configuration(void);
void TIM4_IRQHandler(void);
void SendChuoi(char *str);
void PWM1_Config(void);	//khoi tao 2 timer cap xung
//void PWM3_Config(void);
void USART_Config(unsigned int BaudRate); // Khai bao UART
void USART2_IRQHandler(void);



int8_t k;
int32_t direct, Counter, Rotary=0, InputCapture, pre_InputCapture;
int32_t Position_PV=0, Pulse_per_second, CaptureNumber;
int32_t Time=0, count=0, SetPoint=0; 
int32_t u, pre_u, SetSetPoint=0, error, pre_error, sum_error=0;
volatile static float a, Speed_PV=0, chieu;


volatile static unsigned char RX;
char tam[13];
static double T=0.015, Kp=0, Ki=0, Kd=0;

char RX_Buffer[100]="\0";
int i=0, j=0;



int main(void)
{
	
	Encoder_Config();
	USART_Config(38400);	//khai bao UART voi toc do baud 38400
	PWM1_Config();
	TIMbase_Configuration();
	PWM1_Config();
	
  while (1)
  {
		chieu= TIM_GetCounter(TIM2);
		a =abs(TIM_GetCounter(TIM2) - 32768); 
		
	  Speed_PV = (((float)a/ (334*2)) / 15 ) * 1000 * 60;
		
		if( chieu< 32768) Speed_PV= -Speed_PV; 
		
		TIM_SetCounter(TIM2, 32768);
		TIM_Cmd(TIM2,ENABLE);
		delay(150);
  }
}

void Encoder_Config(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStruct;
GPIO_InitTypeDef  GPIO_InitStructure;
//Configure peripheral clocks
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
//Configure pins
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//Configure Timer
TIM_TimeBaseStructure.TIM_Prescaler = 0;
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_Period = 65535;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  



TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//Debounce filter
TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
TIM_ICInitStruct.TIM_ICFilter=4;
TIM_ICInit(TIM2, &TIM_ICInitStruct);
TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
TIM_ICInitStruct.TIM_ICFilter=4;
TIM_ICInit(TIM2, &TIM_ICInitStruct);
//Setup quadrature encoder and enable timer
TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
TIM_SetCounter(TIM2, 32768);
TIM_Cmd(TIM2, ENABLE); 


TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void PWM1_Config(void)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		//Time base config
		TIM_TimeBaseStructure.TIM_Prescaler = 0;   // system clock = 72 Mhz
		TIM_TimeBaseStructure.TIM_Period = 7199; // 65535 //
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_Pulse = 0;


		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		
		TIM_ARRPreloadConfig(TIM1, ENABLE);
/* TIM1 enable counter */
		TIM_Cmd(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void TIMbase_Configuration(void)
{
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
/* Time base configuration */
TIM_TimeBaseStructure.TIM_Prescaler=840-1;
TIM_TimeBaseStructure.TIM_Period = 999;/*so xung trong 1 chu ky*/
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;/* chon mode counter dem tang*/
TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);/*thiet lap ngat khi tran bo nho co thong so TIM_IT_Update*/
TIM_Cmd(TIM4, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}


void TIM4_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
	if((SetSetPoint-SetPoint)>0) SetPoint=SetPoint+5;
	if((SetSetPoint-SetPoint)<0) SetPoint=SetPoint-5;

	}
	k++;
			if(k==7)
				{
			k=0;
			
			if(Speed_PV>=0) tam[0]='+';
			else tam[0]='-';
			tam[1]=abs(Speed_PV)/1000+0x30;
			tam[2]=(abs(Speed_PV)%1000)/100+0x30;
			tam[3]=((abs(Speed_PV)%1000)%100)/10+0x30;
			tam[4]=(((abs(Speed_PV)%1000)%100)%10)+0x30;
			tam[5]='a';
			if(SetPoint>0) tam[6]='+';
			else tam[6]='-';
			tam[7]=abs(SetPoint)/1000+0x30;
			tam[8]=(abs(SetPoint)%1000)/100+0x30;
			tam[9]=((abs(SetPoint)%1000)%100)/10+0x30;
			tam[10]=(((abs(SetPoint)%1000)%100)%10)+0x30;
			tam[11]='b';
			tam[12]=abs(Rotary)/1000+0x30;
			tam[12]='\0';
			SendChuoi(tam);
			for(int m=0;m<13;m++)
			tam[m]='\0';
					}

					
	error=SetPoint-Speed_PV;
	sum_error=error+sum_error;
	
	u=Kp*error+Ki*sum_error*0.015+Kd*(error-pre_error)/0.015;
  pre_error = error;
	if(u >450)
		u=450;
	else if(u<-450)
	{
		u=-450;
	}
	
	
	
	
	
	if(u>=0)
	{

	TIM1->CCR2=(500-u)*7199/500;
	TIM1->CCR1=7199;

	}
	
else 
	{

	TIM1->CCR2=7199;
	TIM1->CCR1=(500+u)*7199/500;
	}
	

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

}


void USART_Config(unsigned int BaudRate)
{
//Khoi tao xung clock cho cac ngoai vi
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// khoi tao chan Tx cho UART (PB6)
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//In hoac out
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
// khoi tao chan Tx cho UART (PB7)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = BaudRate; // toc do baud
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Khung truyen 8 bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //1 bit stop
	USART_InitStructure.USART_Parity = USART_Parity_No; //khong kiem tra chan le
	USART_InitStructure.USART_HardwareFlowControl = 	USART_HardwareFlowControl_None; //vo hieu hoa dong dieu khien phan cung
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); //cho phép uart 1 hoat dong
//khai bao su dung chan pb6,7
	
	USART_ClearFlag(USART2,USART_IT_RXNE);	//xoa co ngat cho lan dau su dung
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);		//ngat nhan
	USART_Cmd(USART2, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void SendChuoi(char *str)
{
	while(*str)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
		USART_SendData(USART2, *str);
		str++;
	}
}



void delay(uint32_t cout)// delay 0.1ms
{
	
	//while(cout--);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  	TIM3->PSC = 3599;		// clk = SystemCoreClock /2 /(PSC+1) = 1KHz
  	TIM3->ARR = cout-1;
  	TIM3->CNT = 0;
  	TIM3->EGR = 1;		// update registers;

  	TIM3->SR  = 0;		// clear overflow flag
  	TIM3->CR1 = 1;		// enable Timer
  	while (!TIM3->SR);
  	TIM3->CR1 = 0;		// stop Timer
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}


void USART2_IRQHandler(void)
{
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
		
			RX=USART_ReceiveData(USART2);
			if(RX=='E')
			{
				for(i=0;i<8;i++)	//xu ly Kp
				{
					nhan.ReceiveByte[i]=RX_Buffer[i];
				}
				Kp=nhan.Receive;				

				for(i=0;i<8;i++)	//xu ly Ki
				{
					nhan.ReceiveByte[i]=RX_Buffer[i+8];
				}
				Ki=nhan.Receive;
			
				for(i=0;i<8;i++)	//xu ly Kd
				{
					nhan.ReceiveByte[i]=RX_Buffer[i+16];
				}
				Kd=nhan.Receive;

				

				for(i=0;i<8;i++)	//xu ly SetSetPoint
				{
					nhan.ReceiveByte[i]=RX_Buffer[i+24];
				}
				SetSetPoint=nhan.Receive;						
			for(i=0;i<100;i++)
			{
				RX_Buffer[i]='\0';
			}
			i=0;
			}
			else
			{
				RX_Buffer[i++]=RX;
			}
			}
			USART_ClearITPendingBit(USART2,USART_IT_RXNE);		//xoa co
			
			
	
}
