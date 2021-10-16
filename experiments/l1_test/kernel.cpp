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

#define ARRAY_SIZE 32*1024
volatile unsigned char array[ARRAY_SIZE];
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


TShutdownMode CKernel::Run (void)
{
	int i;
  while(1)
    {
        for(i=0;i<ARRAY_SIZE;i++)
        {
            array[i]=0xA5A5A5A5;
        }

        for(i=0;i<ARRAY_SIZE;i++){
        	__asm__("nop");
        }
        int flag=0;
        for(i=0;i<ARRAY_SIZE;i++)
        {
            if(array[i]!=0xA5A5A5A5){
                 if(flag==0){
                buffer[0] = 0xDD000000;
                flag=1;
                }else{
                    buffer[0] = 0xCC000000;
                }
                buffer[1] = *((unsigned int*)&i);
                unsigned int temp=array[i]^(0xA5A5A5A5);
                buffer[2] = *((unsigned int*)&temp);
                send_message(3);
            }
           
        }       
      if(flag==0){
        buffer[0] = 0xAA000000;
          send_message(1); 
        }


    }

	return ShutdownHalt;
}
