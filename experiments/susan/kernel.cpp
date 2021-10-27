//
// kernel.cpp
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2014-2015  R. Stange <rsta2@o2online.de>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "kernel.h"
#include <circle/string.h>
#include <circle/util.h>

#include <math.h>
#include <circle/alloc.h>
static const char FromKernel[] = "kernel";
#define PARTITION	"emmc1-1"

/* Single iteration of the transient solver in the grid model.
 * advances the solution of the discretized difference equations
 * by one time step*/





CKernel::CKernel (void):
	m_Screen (m_Options.GetWidth (), m_Options.GetHeight ()),	
	m_Timer (&m_Interrupt),
	m_Logger (m_Options.GetLogLevel (), &m_Timer),
	m_EMMC (&m_Interrupt, &m_Timer, &m_ActLED),
	m_GPIOManager (&m_Interrupt),
	m_SoftSerial (17, 18, &m_GPIOManager)	
{

	m_ActLED.Blink (5);	// show we are alive
}

CKernel::~CKernel (void)
{
}





void CKernel::send_message(int size){
	m_SoftSerial.Write ((char*)buffer, size*4);
}

boolean CKernel::Initialize (void)
{
	boolean bOK = TRUE;


	if (bOK)
	{
		bOK = m_Serial.Initialize (115200);
	}


	if (bOK)
	{
		bOK = m_Interrupt.Initialize ();
	}

	if (bOK)
	{
		bOK = m_Timer.Initialize ();
	}

	if (bOK)
	{
		bOK = m_EMMC.Initialize ();
	}
	if (bOK)
	{
		bOK = m_GPIOManager.Initialize ();
	}

	if (bOK)
	{
		bOK = m_SoftSerial.Initialize ();
	}

	return bOK;
}


#define MATRIX_SIZE 600
#define uint32_t unsigned int
    float mA[MATRIX_SIZE*MATRIX_SIZE];
    float mB[MATRIX_SIZE*MATRIX_SIZE];
    float mCS0[MATRIX_SIZE*MATRIX_SIZE];
    float float_golden[MATRIX_SIZE*MATRIX_SIZE];
void CKernel::initGold(char* gold_file){
    unsigned hFile = m_FileSystem.FileOpen (gold_file);    
    int i,j;
    m_FileSystem.FileRead(hFile,float_golden, sizeof(float) * MATRIX_SIZE*MATRIX_SIZE);
   
    m_FileSystem.FileClose (hFile);
}
void CKernel::initInput(char* input_file){
    unsigned hFile = m_FileSystem.FileOpen (input_file);    
    int i,j;
    m_FileSystem.FileRead(hFile,mA, sizeof(float) * MATRIX_SIZE*MATRIX_SIZE);
   	m_FileSystem.FileRead(hFile,mB, sizeof(float) * MATRIX_SIZE*MATRIX_SIZE);
    m_FileSystem.FileClose (hFile);
}

TShutdownMode CKernel::Run (void)
{

	// Mount file system
	CDevice *pPartition = m_DeviceNameService.GetDevice (PARTITION, TRUE);
	if (pPartition == 0)
	{
		buffer[0]=0xFF100000;
		send_message(1);
	}

	if (!m_FileSystem.Mount (pPartition))
	{
		buffer[0]=0xFF200000;
		send_message(1);
	}
	int i = 0;
    int j = 0;
    int k = 0;
    int p = 0;
    int status_app;
    //int count = 0;


    //XTime tStart, tEnd, endexec;
    
    initGold("matmul_gold_600.bin");
    initInput("matmul_input_600.bin");
    while(1){

        //########### control_dut ###########

        for (i=0; i<MATRIX_SIZE; i++)
        {
            for(j=0; j<MATRIX_SIZE; j++)
            {
                mCS0[i*MATRIX_SIZE+j] = 0.0;
                for (k=0; k<MATRIX_SIZE; k++)
                    mCS0[i*MATRIX_SIZE+j] += mA[i*MATRIX_SIZE+k] * mB[k*MATRIX_SIZE+j];
            }
        }
        //XTime_GetTime(&endexec);
        //if (count == 5)
        //{mCS0[30][47] = 2.35; count=0;}

        // check for errors
        //mCS0[10][20]--;
        //mCS0[30][20]--;
        int flag=0;
        for (i=0; i<MATRIX_SIZE; i++)
        {
            for(j=0; j<MATRIX_SIZE; j++)
            {

                if((mCS0[i*MATRIX_SIZE+j] != float_golden[i*MATRIX_SIZE+j]))
                {
                    if(flag==0){
                        buffer[0] = 0xDD000000;
                        flag=1;
                    }else{
                        buffer[0] = 0xCC000000;
                    }
                    //printf("\ni=%d j=%d \n %20.18f vs %20.18f\n",i,j,mCS0[i][j],float_golden[i][j]);


                    buffer[1] = *((uint32_t*)&i);
                    buffer[2] = *((uint32_t*)&j);
                    buffer[3] = *((uint32_t*)&mCS0[i*MATRIX_SIZE+j]); // u32, float has 32 bits
                    send_message(4);

                }
            }
            //printf("a");
        }
        //printf("end");

        //########### control_dut ###########
        if (flag == 0) // sem erros
        {
            buffer[0] = 0xAA000000; //sem erros
            send_message(1);
        }
    }

	return ShutdownHalt;
}
