/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Raised Cosine
Roll Off Factor: 0.500000
Sampling Frequency: 100 Hz
Cut Frequency: 1.000000 Hz
Coefficents Quantization: 16-bit
***************************************************************/
#define Ntap 5


#include "stdint.h"
#define __int16 int16_t
#define __int32 int32_t

#define DCgain 131072

__int16 fir(__int16 NewSample) {
    __int16 FIRCoef[Ntap] = { 
        26166,
        26238,
        26261,
        26238,
        26166
    };

    static __int16 x[Ntap]; //input samples
    __int32 y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y / DCgain;
}
