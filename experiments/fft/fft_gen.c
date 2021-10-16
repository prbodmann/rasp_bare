#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fourier.h"
#include <time.h>

void fft_float (
    unsigned  NumSamples,          /* must be a power of 2 */
    int       InverseTransform,    /* 0=forward FFT, 1=inverse FFT */
    float    *RealIn,              /* array of input's real samples */
    float    *ImaginaryIn,         /* array of input's imag samples */
    float    *RealOut,             /* array of output's reals */
    float    *ImaginaryOut );
int main(int argc, char** argv){
    time_t t;
    char name[2000];
    int MAXSIZE=1<<(atoi(argv[1]));
    int MAXWAVES=atoi(argv[2]);    
    int *coeff=(int*)malloc(sizeof(int)*MAXWAVES);
    int *amp=(int*)malloc(sizeof(int)*MAXWAVES);
    float *RealIn=(float*)malloc(sizeof(float)*MAXSIZE);
    float *RealOut=(float*)malloc(sizeof(float)*MAXSIZE);
    float *ImagOut=(float*)malloc(sizeof(float)*MAXSIZE);
    float *ImagIn=(float*)malloc(sizeof(float)*MAXSIZE);
    int i,j;
    FILE* fd,out;

    srand(1);
    for (i=0;i<MAXWAVES;i++){
        coeff[i]=rand() %1000;
        amp[i]=rand() %1000;
    }
    for (i=0;i<MAXSIZE;i++){
        for (j=0;j<MAXWAVES;j++){
            if(rand()%2){
                RealIn[i]+=coeff[j]*cos(amp[j]*i);
            }else{
                RealIn[i]+=coeff[j]*sin(amp[j]*i);
            }

        }
        ImagIn[i]=0;
    }
    sprintf(name,"fft_input_%d.bin",MAXSIZE);
    fd =fopen(name,"wb");
    fwrite(RealIn,sizeof(float),MAXSIZE,fd);
    fclose(fd);
    sprintf(name,"fft_gold_%d.bin",MAXSIZE);
    fft_float (MAXSIZE,0,RealIn,ImagIn,RealOut,ImagOut);
    fd =fopen(name,"wb");
    fwrite(RealOut,sizeof(float),MAXSIZE,fd);
    fwrite(ImagOut,sizeof(float),MAXSIZE,fd);
    fclose(fd);


}
