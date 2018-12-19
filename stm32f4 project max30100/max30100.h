#ifndef __MAX30100__
#define __MAX30100__

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
}LEDCurrent;



extern int32_t IRcw;
extern int32_t REDcw;
extern LEDCurrent redLEDCurrent;

#define MEAN_FILTER_SIZE        15  
typedef struct{
   int32_t value[2];
}Filter_Data;

#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500000

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1
#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size
/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     100
typedef enum PulseStateMachine{
   PULSE_IDLE,
   PULSE_TRACE_UP,
   PULSE_TRACE_DOWN
}PulseStateMachine;


extern volatile uint32_t micros;

#endif
