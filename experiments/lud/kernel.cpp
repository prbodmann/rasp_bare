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
#include "common.h"
#include <math.h>

static const char FromKernel[] = "kernel";


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
#define N 1024

TShutdownMode CKernel::Run (void)
{

	int opt, option_index=0;
    func_ret_t ret;
    const char *input_file = "input_1024_th_1";
    const char *gold_file = "gold_1024_th_1";

    FP m[N*N], gold[N*N], m_input[N*N];
    stopwatch sw;

    
      unsigned hFile = m_FileSystem.FileOpen (input_file);

	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}
	  m_FileSystem.FileRead(hFile,m_input, sizeof(FP) * N * N);
  
     hFile = m_FileSystem.FileOpen (gold_file);

	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}
     m_FileSystem.FileRead(hFile,gold, sizeof(FP) * N * N);
   
    while(1) {
        memcpy(m,m_input,sizeof(FP)*N*N);
        

        lud_omp(m, N);

        int i, flag = 0;        
        for ( i = 0; i < N; i++) {
            int j;
            for ( j = 0; j < N; j++) {
               
                if(m[i + N * j] != gold[i + N * j])
                {
                     if(flag==0){
                        buffer[0] = 0xDD000000;
                        flag=1;
                    }else{
                        buffer[0] = 0xCC000000;
                    }
                    buffer[1]=*((unsigned int*)&i); 
                    buffer[2]=*((unsigned int*)&j); 
                    unsigned long long aux=*((unsigned long long*)&m[i + N * j]);
                    buffer[3] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                    buffer[4] = (unsigned int)(aux & 0xFFFFFFFFLL);
                    send_message(5); 
                }
                    
            }
        }
         if(flag==0){
            buffer[0] = 0xAA000000;
              send_message(1); 
        }

       

        // read inputs again


    }

	return ShutdownHalt;
}
