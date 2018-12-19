#include "FIR.h"

#include <string.h>
#include <stdlib.h>
#include <assert.h>

FIR::FIR(double *coefficients, unsigned number_of_taps) :
	coefficients(coefficients),
	buffer(new double[number_of_taps]()),  
	taps(number_of_taps),
	offset(0)
{}

FIR::~FIR()
{
  delete[] buffer;
  //delete[] coefficients;
}

double FIR::filter(double input)
{
	double *coeff     = coefficients;
	double *coeff_end = coefficients + taps;

	double *buf_val = buffer + offset;

	*buf_val = input;
	double output_ = 0;
	
	while(buf_val >= buffer)
		output_ += *buf_val-- * *coeff++;
	
	buf_val = buffer + taps-1;
	
	while(coeff < coeff_end)
		output_ += *buf_val-- * *coeff++;
	
	if(++offset >= taps)
		offset = 0;
	
	return output_;
}

void FIR::reset()
{
	memset(buffer, 0, sizeof(double)*taps);
	offset = 0;
}
