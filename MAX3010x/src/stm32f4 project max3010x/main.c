#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rtc.h>

#include "I2C.h"
#include "VCP.h"

#include "tiny_printf.h"

#include "max3010x.h"

#define __FPU_PRESENT
#define __FPU_USED

volatile uint32_t TimingDelay;

volatile uint32_t micros = 0;

volatile bool max30102_ready = false;

void Delay(__IO uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay){
   }
}

void SysTick_Handler(void)
{
   if(TimingDelay){
      --TimingDelay;
   }
   ++micros;
}

void Configure_PB0(void) {
   /* Set variables used */
   GPIO_InitTypeDef GPIO_InitStruct;
   EXTI_InitTypeDef EXTI_InitStruct;
   NVIC_InitTypeDef NVIC_InitStruct;

   /* Enable clock for GPIOB */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
   /* Enable clock for SYSCFG */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   /* Set pin as input */
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOB, &GPIO_InitStruct);

   /* Tell system that you will use PB12 for EXTI_Line12 */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

   /* PB12 is connected to EXTI_Line12 */
   EXTI_InitStruct.EXTI_Line = EXTI_Line0;
   /* Enable interrupt */
   EXTI_InitStruct.EXTI_LineCmd = ENABLE;
   /* Interrupt mode */
   EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
   /* Triggers on rising and falling edge */
   EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
   /* Add to EXTI */
   EXTI_Init(&EXTI_InitStruct);

   /* Add IRQ vector to NVIC */
   /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
   NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
   /* Set priority */
   NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
   /* Set sub priority */
   NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
   /* Enable interrupt */
   NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
   /* Add to NVIC */
   NVIC_Init(&NVIC_InitStruct);
}

void EXTI0_IRQHandler(void) {
   /* Make sure that interrupt flag is set */
   if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
      /* Do your stuff when PB12 is changed */
      /* Clear interrupt flag */
      max30102_ready = true;
      EXTI_ClearITPendingBit(EXTI_Line0);
   }
}

int main(void)
{
   if(SysTick_Config(SystemCoreClock / 1000 / 1000)){
      while(1){}
   }

   USBVCPInit();
   Init_I2C1();
   Configure_PB0();

   Init_MAX3010x();

   Read_MAX3010x();

   int id_state = 0;
   int recheck_state = 0;
   int sn = 1;  

   Filter_Data irf = {0};
   Filter_Data redf = {0};  

   uint32_t irACValueSqSum = 0;
   uint32_t redACValueSqSum = 0;
   uint16_t samplesRecorded = 0;
   uint16_t pulsesDetected = 0;
   uint32_t currentSaO2Value = 0;  

   while(1){
      Read_MAX3010x();
      /*
	 if(max30102_ready){
	 max30102_ready = false;
	 Read_MAX3010x();

	 }*/
      int32_t ORG_IR = IR;
      int32_t ORG_RED = RED;

      int32_t IRac = DCRemove(IR,&IRcw);
      int32_t REDac = DCRemove(RED,&REDcw);



      IRac = MeanDiff(IRac);
      REDac = MeanDiff(REDac);

      IRac = LowPassButterworthFilter(IRac,&irf);
      REDac = LowPassButterworthFilter(REDac,&redf);


      irACValueSqSum += IRac * IRac;
      redACValueSqSum += REDac * REDac;
      samplesRecorded++;  
      if(detectPulse(IRac)){
	 pulsesDetected++;


	 float red_log_rms = log( sqrt(redACValueSqSum/samplesRecorded) );
	 float ir_log_rms = log( sqrt(irACValueSqSum/samplesRecorded) );
	 float ratioRMS = 0.0f;
	 if(red_log_rms != 0.0f && ir_log_rms != 0.0f){
	    ratioRMS = red_log_rms / ir_log_rms;
	 }
	 currentSaO2Value = 110.0f - 14.0f * ratioRMS;	// SPo2 value by pulse-rate


	 if(pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0){
	    irACValueSqSum = 0;
	    redACValueSqSum = 0;
	    samplesRecorded = 0;
	 }     
      }


      BalanceIntesities();

      unsigned char str[255];
      sprintf(str,"R%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d, ",micros,ORG_IR,ORG_RED,IRcw,REDcw,IRac,REDac,irACValueSqSum,redACValueSqSum,redLEDCurrent,pulsesDetected,samplesRecorded);

      if(USB_VCP_GetStatus() == USB_VCP_CONNECTED) {
	 if(sn == 1){
	    uint8_t c;
	    if (USB_VCP_Getc(&c) == USB_VCP_DATA_OK) {
	       /* Return data back */
	       if(c == 'D' && id_state == 0){
		  ++id_state;
	       }
	       if(c == 'I' && id_state == 1){
		  ++id_state;
	       }
	       if(c == 'D' && id_state == 2){
		  ++id_state;
		  USB_VCP_Puts("GID:TonyTonyTonyTony");
		  sn = 0;

	       }
	    }
	 }
	 else{
	    uint8_t c;
	    if (USB_VCP_Getc(&c) == USB_VCP_DATA_OK) {
	       /* Return data back */
	       if(c == 'C' && recheck_state == 0){
		  ++recheck_state;
	       }
	       if(c == 'H' && recheck_state == 1){
		  ++recheck_state;
	       }
	       if(c == 'K' && recheck_state == 2){
		  ++recheck_state;
		  char chk[] = {0x65,0x65,0x65,0x65,0x65,0x0d,0x0a};
		  USB_VCP_Puts(chk);
	       }
	    }
	 }

	 if(recheck_state == 3){
	    USB_VCP_Puts(str);
	 }
      }
      Delay(13000);
   }

   return(0); // System will implode
}    
