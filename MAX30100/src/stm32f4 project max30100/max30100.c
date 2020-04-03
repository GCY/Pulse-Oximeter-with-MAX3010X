#include "max30100.h"

int32_t IRcw = 0;
int32_t REDcw = 0;

int32_t msum = 0;
int32_t mvalues[MEAN_FILTER_SIZE];
int32_t mindex = 0;
int32_t mcount = 0;

volatile uint32_t lastREDLedCurrentCheck = 0;
LEDCurrent redLEDCurrent = MAX30100_LED_CURRENT_27_1MA;

uint8_t currentPulseDetectorState = PULSE_IDLE;
uint32_t lastBeatThreshold = 0;
uint32_t currentBPM;
uint32_t valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
uint32_t valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;

int32_t DCRemove(int32_t value,int32_t *cw)
{
   int32_t oldcw = *cw;
   *cw = value + 0.94 * *cw;
   return *cw - oldcw;
}

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

int32_t LowPassButterworthFilter(int32_t value,Filter_Data *filter_data)
{  
   filter_data->value[0] = filter_data->value[1];
   //Fs = 100Hz and Fc = 10Hz
   filter_data->value[1] = (2.452372752527856026e-1 * value) + (0.50952544949442879485 * filter_data->value[0]);

   //Fs = 100Hz and Fc = 4Hz
   //filter_data->value[1] = (1.367287359973195227e-1 * value) + (0.72654252800536101020 * filter_data->value[0]); //Very precise butterworth filter 

   return filter_data->value[0] + filter_data->value[1];
}

void BalanceIntesities()
{

   if( micros - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS) 
   {
      //Serial.println( redLedDC - IRLedDC );
      if( IRcw - REDcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA) 
      {
	 redLEDCurrent++;
	 I2C_Write(I2C1,MAX30100_ADDRESS,MAX30100_LED_CONFIG,(redLEDCurrent << 4) | MAX30100_LED_CURRENT_50MA);
      } 
      else if(REDcw - IRcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) 
      {
	 redLEDCurrent--;
	 I2C_Write(I2C1,MAX30100_ADDRESS,MAX30100_LED_CONFIG,(redLEDCurrent << 4) | MAX30100_LED_CURRENT_50MA);
      }

      lastREDLedCurrentCheck = micros;
   }
}

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

