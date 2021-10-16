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
#include "fourier.h"


#define uint32_t unsigned int
#define PARTITION	"emmc1-1"
#define INPUT_FILENAME "fft_input.bin"
#define GOLD_FILENAME "fft_gold.bin"
static const char FromKernel[] = "kernel";
#define SIZE_ARRAY 21
float RealIn[1<<SIZE_ARRAY];
float RealOut[1<<SIZE_ARRAY];
float ImagOut[1<<SIZE_ARRAY];
float ImagIn[1<<SIZE_ARRAY];
unsigned int goldReal[1<<SIZE_ARRAY];
unsigned int goldImag[1<<SIZE_ARRAY];

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

void CKernel::init_input(void){
	// Reopen file, read it and display its contents
	unsigned hFile = m_FileSystem.FileOpen (INPUT_FILENAME);
	unsigned nResult;
	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}


	while ((nResult = m_FileSystem.FileRead (hFile, RealIn, 1<<(SIZE_ARRAY+2))) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF10000;
			send_message(1);
			break;
		}


	}
	if (!memset(ImagIn,0,1<<(SIZE_ARRAY+2))){
		buffer[0]=0xFFF30000;
		send_message(1);
		
	}

	if (!m_FileSystem.FileClose (hFile))
	{
		buffer[0]=0xFFF40000;
		send_message(1);	
	}
}


void CKernel::init_gold(void){
	// Reopen file, read it and display its contents
	unsigned hFile = m_FileSystem.FileOpen (GOLD_FILENAME);
	unsigned nResult;

	if (hFile == 0)
	{
		buffer[0]=0xFFF50000;
		send_message(1);
	}


	while ((nResult = m_FileSystem.FileRead (hFile, goldReal, 1<<(SIZE_ARRAY+2))) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF60000;
			send_message(1);
			break;
		}


	}

	while ((nResult = m_FileSystem.FileRead (hFile, goldImag, 1<<(SIZE_ARRAY+2))) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF70000;
			send_message(1);
			break;
		}


	}

	if (!m_FileSystem.FileClose (hFile))
	{
		buffer[0]=0xFFF80000;
			send_message(1);
	}
}



TShutdownMode CKernel::Run (void)
{
 unsigned i;
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
	init_input();
	init_gold();	
	 while(1){          



          /* regular*/
          fft_float (1<<SIZE_ARRAY,0,RealIn,ImagIn,RealOut,ImagOut);

        unsigned status_app=0;
        //printf("unsigned int goldRealOut[]={\n\r");

              for(i=0; i<(1<<SIZE_ARRAY); i++)
              {
                  //printf("0x%lX,\n\r",*((uint32_t*)&RealOut[i]));
  		          if((*((unsigned int*)&RealOut[i]) != goldReal[i]) || (*((unsigned int*)&ImagOut[i]) != goldImag[i]))
                    {
                        if(status_app==0){
                            buffer[0] = 0xDD000000;

                        }else{
                            buffer[0] = 0xCC000000;
                        }

                        buffer[1] = *((uint32_t*)&i);
                        buffer[2] = *((uint32_t*)&RealOut[i]);
                        buffer[3] = *((uint32_t*)&ImagOut[i]); // u32, float has 32 bits

                        send_message(4);
                        status_app=1;
                    }

              }

                //printf("};");
                //return 0;
        

        if(status_app==0){
            buffer[0] = 0xAA000000; //sem erros
            send_message(1);
        }
    }


	return ShutdownHalt;
}
