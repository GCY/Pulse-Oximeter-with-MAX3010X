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


#include "./usb_cdc_device/usbd_usr.h"
#include "./usb_cdc_device/usbd_cdc_core.h"
#include "./usb_cdc_device/usb_conf.h"
#include "./usb_cdc_device/usbd_desc.h"
#include "./usb_cdc_device/usbd_cdc_vcp.h"


//#include "stm32f4_usb_vcp"

//#include "tm_stm32f4_usb_vcp.h"
#include "tiny_printf.h"

#define MAX30100_ADDRESS 0xAE
#define MAX30100_INT_STATUS 0x00
#define MAX30100_INT_ENABLE     0x01
#define MAX30100_FIFO_WR_PTR    0x02 
#define MAX30100_OVRFLOW_CTR    0x03 
#define MAX30100_FIFO_RD_PTR    0x04  
#define MAX30100_FIFO_DATA      0x05 
#define MAX30100_MODE_CONFIG    0x06 
#define MAX30100_SPO2_CONFIG    0x07 
#define MAX30100_LED_CONFIG     0x09 
#define MAX30100_TEMP_INTG      0x16 
#define MAX30100_TEMP_FRAC      0x17  
#define MAX30100_REV_ID         0xFE  
#define MAX30100_PART_ID        0xFF 
#define POR_PART_ID             0x11
typedef enum{
   MAX30100_MODE_HR_ONLY = 0x02,
   MAX30100_MODE_SPO2_HR = 0x03
}Mode;
typedef enum{ // This is the same for both LEDs
  pw200,    // 200us pulse
  pw400,    // 400us pulse
  pw800,    // 800us pulse
  pw1600    // 1600us pulse
}pulseWidth;

typedef enum{
  sr50,    // 50 samples per second
  sr100,   // 100 samples per second
  sr167,   // 167 samples per second
  sr200,   // 200 samples per second
  sr400,   // 400 samples per second
  sr600,   // 600 samples per second
  sr800,   // 800 samples per second
  sr1000   // 1000 samples per second
}sampleRate;
typedef enum{
  so2,	// SO2 interrupt
  hr,	// Heart-rate interrupt
  temp,	// Temperature interrupt
  full,	// FIFO full interrupt
}interruptSource;
typedef enum{
  i0,    // No current
  i4,    // 4.4mA
  i8,    // 7.6mA
  i11,   // 11.0mA
  i14,   // 14.2mA
  i17,   // 17.4mA
  i21,   // 20.8mA
  i24,	 // 24mA
  i27,   // 27.1mA
  i31,   // 30.6mA
  i34,   // 33.8mA
  i37,   // 37.0mA
  i40,   // 40.2mA
  i44,   // 43.6mA
  i47,   // 46.8mA
  i50    // 50.0mA
}ledCurrent;
typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

volatile uint32_t TimingDelay;

volatile uint32_t micros = 0;

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

void init_I2C2(){

    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    // 設定 I2C1 
    I2C_InitStruct.I2C_ClockSpeed = 100000; // 設定 I2C 時鐘速度為 100kHz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C 模式
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
    I2C_InitStruct.I2C_OwnAddress1 = 0x00; // own address, not relevant in master mode
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable; // disable acknowledge when reading (can be changed later on)
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 設定 I2C 地址長度為 7 bit
    I2C_Init(I2C2, &I2C_InitStruct); // 初始化 I2C1

    // 啟用 I2C1
    I2C_Cmd(I2C2, ENABLE);

}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
    // wait until I2C1 is not busy anymore
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

    // Send I2C1 START condition 
    I2C_GenerateSTART(I2Cx, ENABLE);

    // wait for I2C1 EV5 --> Slave has acknowledged start condition
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    // Send slave Address for write 
    I2C_Send7bitAddress(I2Cx, address, direction);

    /* wait for I2C1 EV6, check if 
    * either Slave has acknowledged Master transmitter or
    * Master receiver mode, depending on the transmission
    * direction
    */ 
    if(direction == I2C_Direction_Transmitter){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    } else if(direction == I2C_Direction_Receiver){
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

/* This function transmits one byte to the slave device
* Parameters:
* I2Cx --> the I2C peripheral e.g. I2C1 
* data --> the data byte to be transmitted
*/
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2C_SendData(I2Cx, data);
    // wait for I2C1 EV8_2 --> byte has been transmitted
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
* and acknowledges the byte (requests another byte)
*/
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
    uint8_t data;
    // enable acknowledge of recieved data
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This function reads one byte from the slave device
* and doesn't acknowledge the recieved data 
*/
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
    uint8_t data;
    // disabe acknowledge of received data
    // nack also generates stop condition after last byte received
    // see reference manual for more info
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    // wait until one byte has been received
    while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    // read data from I2C data register and return data byte
    data = I2C_ReceiveData(I2Cx);
    return data;
}

/* This funtion issues a stop condition and therefore
* releases the bus
*/
void I2C_stop(I2C_TypeDef* I2Cx){
    // Send I2C1 STOP Condition 
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	I2C_start(I2Cx, address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, reg);
	I2C_write(I2Cx, data);
	I2C_stop(I2Cx);
}

void I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
	I2C_start(I2Cx, address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, reg);
	while (count--) {
		I2C_write(I2Cx, *data++);
	}
	I2C_stop(I2Cx);
}

/* Private */
#define USB_VCP_RECEIVE_BUFFER_LENGTH		128
uint8_t INT_USB_VCP_ReceiveBuffer[USB_VCP_RECEIVE_BUFFER_LENGTH];
uint32_t int_usb_vcp_buf_in, int_usb_vcp_buf_out, int_usb_vcp_buf_num;
USB_VCP_Result USB_VCP_INT_Status;
//extern LINE_CODING linecoding;
uint8_t USB_VCP_INT_Init = 0;
USB_OTG_CORE_HANDLE	USB_OTG_dev;

extern uint8_t INT_USB_VCP_ReceiveBuffer[USB_VCP_RECEIVE_BUFFER_LENGTH];

USB_VCP_Result USBVCPInit(void)
{
   USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_FS
	 USB_OTG_FS_CORE_ID,
#else
	 USB_OTG_HS_CORE_ID,
#endif
	 &USR_desc, 
	 &USBD_CDC_cb, 
	 &USR_cb);   
   
   /* Reset buffer counters */
   int_usb_vcp_buf_in = 0;
   int_usb_vcp_buf_out = 0;
   int_usb_vcp_buf_num = 0;
	
   /* Initialized */
   USB_VCP_INT_Init = 1;

   return USB_VCP_OK;
}

USB_VCP_Result USB_VCP_GetStatus(void) {
   if (USB_VCP_INT_Init) {
      return USB_VCP_INT_Status;
   }
   return USB_VCP_ERROR;
}

USB_VCP_Result USB_VCP_Getc(uint8_t* c) {
   /* Any data in buffer */
   if (int_usb_vcp_buf_num > 0) {
      /* Check overflow */
      if (int_usb_vcp_buf_out >= USB_VCP_RECEIVE_BUFFER_LENGTH) {
	 int_usb_vcp_buf_out = 0;
      }
      *c = INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_out];
      INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_out] = 0;
		
      /* Set counters */
      int_usb_vcp_buf_out++;
      int_usb_vcp_buf_num--;
		
      /* Data OK */
      return USB_VCP_DATA_OK;
   }
   *c = 0;
   /* Data not ready */
   return USB_VCP_DATA_EMPTY;
}

USB_VCP_Result USB_VCP_Putc(volatile char c) {
	uint8_t ce = (uint8_t)c;
	
	/* Send data over USB */
	VCP_DataTx(&ce, 1);
	
	/* Return OK */
	return USB_VCP_OK;
}

USB_VCP_Result USB_VCP_Puts(char* str) {
	while (*str) {
		USB_VCP_Putc(*str++);
	}
	
	/* Return OK */
	return USB_VCP_OK;
}

USB_VCP_Result INT_USB_VCP_AddReceived(uint8_t c) {
	/* Still available data in buffer */
	if (int_usb_vcp_buf_num < USB_VCP_RECEIVE_BUFFER_LENGTH) {
		/* Check for overflow */
		if (int_usb_vcp_buf_in >= USB_VCP_RECEIVE_BUFFER_LENGTH) {
			int_usb_vcp_buf_in = 0;
		}
		/* Add character to buffer */
		INT_USB_VCP_ReceiveBuffer[int_usb_vcp_buf_in] = c;
		/* Increase counters */
		int_usb_vcp_buf_in++;
		int_usb_vcp_buf_num++;
		
		/* Return OK */
		return USB_VCP_OK;
	}
	
	/* Return Buffer full */
	return USB_VCP_RECEIVE_BUFFER_FULL;
}

int32_t IRcw = 0;
int32_t REDcw = 0;
int32_t DCRemove(int32_t value,int32_t *cw)
{
   //value *= 100;
   int32_t oldcw = *cw;
   *cw = value + 0.95 * *cw;
   return *cw - oldcw;
}

#define MEAN_FILTER_SIZE        15  
int32_t msum = 0;
int32_t mvalues[MEAN_FILTER_SIZE];
int32_t mindex = 0;
int32_t mcount = 0;

int32_t MeanDiff(int32_t M)
{
  int32_t avg = 0;

  msum -= mvalues[mindex];
  mvalues[mindex] = M;
  msum += mvalues[mindex];

  mindex++;
  mindex = mindex % MEAN_FILTER_SIZE;

  if(mcount < MEAN_FILTER_SIZE){
     mcount++;
  }

  avg = msum / mcount;
  return avg - M;
}

typedef struct{
int32_t value[2];
}Filter_Data;
int32_t LowPassButterworthFilter(int32_t value,Filter_Data *filter_data)
{  
   filter_data->value[0] = filter_data->value[1];
   //Fs = 100Hz and Fc = 10Hz
   filter_data->value[1] = (2.452372752527856026e-1 * value) + (0.50952544949442879485 * filter_data->value[0]);

   //Fs = 100Hz and Fc = 4Hz
   //filter_data->value[1] = (1.367287359973195227e-1 * value) + (0.72654252800536101020 * filter_data->value[0]); //Very precise butterworth filter 

   return filter_data->value[0] + filter_data->value[1];
}

volatile uint32_t lastREDLedCurrentCheck = 0;
LEDCurrent redLEDCurrent = MAX30100_LED_CURRENT_27_1MA;
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500000
void BalanceIntesities()
{
  
  if( micros - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS) 
  {
    //Serial.println( redLedDC - IRLedDC );
    if( IRcw - REDcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA) 
    {
      redLEDCurrent++;
      I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_LED_CONFIG,(redLEDCurrent << 4) | MAX30100_LED_CURRENT_50MA);
    } 
    else if(REDcw - IRcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) 
    {
      redLEDCurrent--;
      I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_LED_CONFIG,(redLEDCurrent << 4) | MAX30100_LED_CURRENT_50MA);
    }

    lastREDLedCurrentCheck = micros;
  }
}

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1
#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size
/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     100
typedef enum PulseStateMachine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;
uint8_t currentPulseDetectorState = PULSE_IDLE;
uint32_t lastBeatThreshold = 0;
uint32_t currentBPM;
uint32_t valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
uint32_t valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
bool detectPulse(uint32_t sensor_value)
{
  static uint32_t prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = micros;
        lastBeatThreshold = sensor_value;
      }
      else
      {
/*
        if(debug == true) 
        {
          Serial.print("Peak reached: ");
          Serial.print(sensor_value);
          Serial.print(" ");
          Serial.println(prev_sensor_value);
        }*/

        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        uint32_t rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000000 / beatDuration;
        /*if(debug == true) 
          Serial.println(rawBPM);*/

        //This method sometimes glitches, it's better to go through whole moving average everytime
        //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
        //valuesBPMSum -= valuesBPM[bpmIndex];
        //valuesBPM[bpmIndex] = rawBPM;
        //valuesBPMSum += valuesBPM[bpmIndex];

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;
        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }
/*
        if(debug == true) 
        {
          Serial.print("CurrentMoving Avg: ");
          for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
          {
            Serial.print(valuesBPM[i]);
            Serial.print(" ");
          }
  
          Serial.println(" ");
        }*/

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;
        /*if(debug == true) 
        {
          Serial.print("AVg. BPM: ");
          Serial.println(currentBPM);
        }*/


        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void Configure_PB12(void) {
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
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
    
    /* PB12 is connected to EXTI_Line12 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

volatile int int_v = 0;
void EXTI15_10_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        /* Do your stuff when PB12 is changed */
        ++int_v;
        
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

int main(void)
{
   if(SysTick_Config(SystemCoreClock / 1000 / 1000)){
      while(1){}
   }

  USBVCPInit();
  init_I2C2();
  Configure_PB12();

  I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_MODE_CONFIG,MAX30100_MODE_SPO2_HR);
  I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_LED_CONFIG,(i27 << 4) | i50);
  I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_SPO2_CONFIG,(sr100<<2)|pw1600);
  Delay(5000);

  I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_INT_ENABLE, ((hr+1) << 4));

  /*
  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Transmitter); 
  I2C_write(I2C2, MAX30100_INT_ENABLE); 
  I2C_stop(I2C2); 
  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Receiver);
  uint8_t int_data;
  int_data = I2C_read_nack(I2C2);
  I2C_stop(I2C2);
  I2C_Write(I2C2,MAX30100_ADDRESS,MAX30100_INT_ENABLE,int_data | 0x10);*/
  
 

  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Transmitter); 
  I2C_write(I2C2, MAX30100_FIFO_DATA); 
  I2C_stop(I2C2); 
  uint8_t received_data[4];
  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Receiver);
  received_data[0] = I2C_read_ack(I2C2); 
  received_data[1] = I2C_read_ack(I2C2);
  received_data[2] = I2C_read_ack(I2C2);
  received_data[3] = I2C_read_nack(I2C2); 
  I2C_stop(I2C2);
  //int16_t data = received_data[0] << 8 | received_data[1];
  uint16_t IR = (received_data[0]<<8) | received_data[1];;      // Last IR reflectance datapoint
  uint16_t RED = (received_data[2]<<8) | received_data[3];;     // Last Red reflectance datapoint

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
  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Transmitter); 
  I2C_write(I2C2, MAX30100_FIFO_DATA); 
  I2C_stop(I2C2); 
  uint8_t received_data[4];
  I2C_start(I2C2, MAX30100_ADDRESS, I2C_Direction_Receiver);
  received_data[0] = I2C_read_ack(I2C2); 
  received_data[1] = I2C_read_ack(I2C2);
  received_data[2] = I2C_read_ack(I2C2);
  received_data[3] = I2C_read_nack(I2C2); 
  I2C_stop(I2C2);
  int32_t IR = 0x00000000 | (received_data[0]<<8) | received_data[1];      // Last IR reflectance datapoint
  int32_t RED = 0x00000000 | (received_data[2]<<8) | received_data[3];     // Last Red reflectance datapoint

  IR = DCRemove(IR,&IRcw);
  RED = DCRemove(RED,&REDcw);

  int32_t IRdc = IR;
  int32_t REDdc = RED;
/*
  IR = MeanDiff(IR);*/
  IR = LowPassButterworthFilter(IR,&irf);
  RED = LowPassButterworthFilter(RED,&redf);
 
  irACValueSqSum += IRdc * IRdc;
  redACValueSqSum += REDdc * REDdc;
  samplesRecorded++;  
  if(/*detectPulse(IR)*/ true){
     pulsesDetected++;
     /*uint32_t ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );
     currentSaO2Value = 110 - 18 * ratioRMS;*/
     if(pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0){
	irACValueSqSum = 0;
	redACValueSqSum = 0;
	samplesRecorded = 0;
     }     
  }

  
  BalanceIntesities();

  unsigned char str[255];
  sprintf(str,"R%d,%d,%d,%d,%d,%d,%d,%d",micros,IR,RED,irACValueSqSum,redACValueSqSum,redLEDCurrent,pulsesDetected,samplesRecorded);
  //sprintf(str,"%d,4431,2341,4432,1233,2131,5453,3436,6456,1233,1553,2344\r\n",count_usb);

  //sprintf(str,"R%d,%d,%d,%d,%d\r\n",micros,adc_value[0],adc_value[1],adc_value[2],adc_value[3]);  
  

     if(USB_VCP_GetStatus() == USB_VCP_CONNECTED) {
	if(sn == 1){
	   uint8_t c;
	   if (USB_VCP_Getc(&c) == USB_VCP_DATA_OK) {
	      /* Return data back */
	      //TM_USB_VCP_Putc(c);
	      if(c == 'G' && id_state == 0){
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
	      //TM_USB_VCP_Putc(c);
	      if(c == 'S' && recheck_state == 0){
		 ++recheck_state;
	      }
	      if(c == 'M' && recheck_state == 1){
		 ++recheck_state;
	      }
	      if(c == 'S' && recheck_state == 2){
		 ++recheck_state;
		 char sms[] = {0x53,0x4d,0x53,0x3a,0x41,0x0d,0x0a};
		 USB_VCP_Puts(sms);
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
