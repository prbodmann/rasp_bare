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
#include "./kernel_cpu.h"
#include <math.h>
#include <circle/alloc.h>
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
	 int i, j, k, l, m, n;

    par_str par_cpu;
    dim_str dim_cpu;
    box_str* box_cpu;
    FOUR_VECTOR* rv_cpu;
    fp* qv_cpu;
    FOUR_VECTOR* fv_cpu;
    FOUR_VECTOR* fv_cpu_GOLD;
    int nh;
    dim_cpu.boxes1d_arg = 5;
	
	 par_cpu.alpha = 0.5;

    dim_cpu.number_boxes = dim_cpu.boxes1d_arg * dim_cpu.boxes1d_arg * dim_cpu.boxes1d_arg;

    dim_cpu.space_elem = dim_cpu.number_boxes * NUMBER_PAR_PER_BOX;
    dim_cpu.space_mem = dim_cpu.space_elem * sizeof(FOUR_VECTOR);
    dim_cpu.space_mem2 = dim_cpu.space_elem * sizeof(fp);

    dim_cpu.box_mem = dim_cpu.number_boxes * sizeof(box_str);

    box_cpu = (box_str*)malloc(dim_cpu.box_mem);

    nh = 0;

    for(i=0; i<dim_cpu.boxes1d_arg; i++) {

        for(j=0; j<dim_cpu.boxes1d_arg; j++) {

            for(k=0; k<dim_cpu.boxes1d_arg; k++) {

                box_cpu[nh].x = k;
                box_cpu[nh].y = j;
                box_cpu[nh].z = i;
                box_cpu[nh].number = nh;
                box_cpu[nh].offset = nh * NUMBER_PAR_PER_BOX;

                box_cpu[nh].nn = 0;

                for(l=-1; l<2; l++) {

                    for(m=-1; m<2; m++) {

                        for(n=-1; n<2; n++) {

                            if((((i+l)>=0 && (j+m)>=0 && (k+n)>=0)==true && ((i+l)<dim_cpu.boxes1d_arg && (j+m)<dim_cpu.boxes1d_arg && (k+n)<dim_cpu.boxes1d_arg)==true) && (l==0 && m==0 && n==0)==false) {

                                box_cpu[nh].nei[box_cpu[nh].nn].x = (k+n);
                                box_cpu[nh].nei[box_cpu[nh].nn].y = (j+m);
                                box_cpu[nh].nei[box_cpu[nh].nn].z = (i+l);
                                box_cpu[nh].nei[box_cpu[nh].nn].number = (box_cpu[nh].nei[box_cpu[nh].nn].z * dim_cpu.boxes1d_arg * dim_cpu.boxes1d_arg) + (box_cpu[nh].nei[box_cpu[nh].nn].y * dim_cpu.boxes1d_arg) + box_cpu[nh].nei[box_cpu[nh].nn].x;
                                box_cpu[nh].nei[box_cpu[nh].nn].offset = box_cpu[nh].nei[box_cpu[nh].nn].number * NUMBER_PAR_PER_BOX;

                                box_cpu[nh].nn = box_cpu[nh].nn + 1;

                            }
                        }
                    }
                }

                nh = nh + 1;
            }
        }
    }



    unsigned hFile = m_FileSystem.FileOpen (INPUT_DISTANCE);

	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}

    rv_cpu = (FOUR_VECTOR*)malloc(dim_cpu.space_mem);
    for(i=0; i<dim_cpu.space_elem; i=i+1) {
        m_FileSystem.FileRead(hFile,&(rv_cpu[i].v), sizeof(double));
        m_FileSystem.FileRead(hFile,&(rv_cpu[i].x), sizeof(double));
        m_FileSystem.FileRead(hFile,&(rv_cpu[i].y), sizeof(double));
        m_FileSystem.FileRead(hFile,&(rv_cpu[i].z), sizeof(double));
    }

    m_FileSystem.FileClose (hFile);
    hFile = m_FileSystem.FileOpen (INPUT_CHARGE);
	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}
    qv_cpu = (fp*)malloc(dim_cpu.space_mem2);
    for(i=0; i<dim_cpu.space_elem; i=i+1) {
         m_FileSystem.FileRead(hFile,&(qv_cpu[i]), sizeof(double));
    }
    m_FileSystem.FileClose (hFile);

    fv_cpu = (FOUR_VECTOR*)malloc(dim_cpu.space_mem);
    fv_cpu_GOLD = (FOUR_VECTOR*)malloc(dim_cpu.space_mem);
    if( (hFile =  m_FileSystem.FileOpen (GOLD)) == 0 ) {
        buffer[0]=0xFFF00000;
		send_message(1);
    }
    for(i=0; i<dim_cpu.space_elem; i=i+1) {
        fv_cpu[i].v = 0;
        fv_cpu[i].x = 0;
        fv_cpu[i].y = 0;
        fv_cpu[i].z = 0;

          m_FileSystem.FileRead(hFile,&(fv_cpu_GOLD[i].v), sizeof(double));
          m_FileSystem.FileRead(hFile,&(fv_cpu_GOLD[i].x), sizeof(double));
          m_FileSystem.FileRead(hFile,&(fv_cpu_GOLD[i].y), sizeof(double));
          m_FileSystem.FileRead(hFile,&(fv_cpu_GOLD[i].z), sizeof(double));
    }

     m_FileSystem.FileClose(hFile);

	 while(1) {

        for(i=0; i<dim_cpu.space_elem; i=i+1) {
            fv_cpu[i].v = 0;
            fv_cpu[i].x = 0;
            fv_cpu[i].y = 0;
            fv_cpu[i].z = 0;
        }


        kernel_cpu(	par_cpu,
                    dim_cpu,
                    box_cpu,
                    rv_cpu,
                    qv_cpu,
                    fv_cpu);


        int flag=0;
        for(i=0; i<dim_cpu.space_elem; i++) {
            int thread_error=0;
            if (fv_cpu[i].v != fv_cpu_GOLD[i].v || fv_cpu[i].x != fv_cpu_GOLD[i].x || fv_cpu[i].y != fv_cpu_GOLD[i].y || fv_cpu[i].z != fv_cpu_GOLD[i].z) {
                //if(fv_cpu_GOLD[i].v != fv_cpu[i].v) {
                if(flag==0){
                    buffer[0] = 0xDD000000;
                    flag=1;
                }else{
                    buffer[0] = 0xCC000000;
                }
                buffer[1]=*((unsigned int*)&i);
                unsigned long long aux=*((unsigned long long*)&fv_cpu[i].v);
                    buffer[2] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                    buffer[3] = (unsigned int)(aux & 0xFFFFFFFFLL);
                     	
                 m=*((unsigned long long*)&fv_cpu[i].x);
                    buffer[4] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                    buffer[5] = (unsigned int)(aux & 0xFFFFFFFFLL);
                      

                 m=*((unsigned long long*)&fv_cpu[i].y);
                    buffer[6] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                    buffer[7] = (unsigned int)(aux & 0xFFFFFFFFLL);
                       
                 m=*((unsigned long long*)&fv_cpu[i].z);
                    buffer[8] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                    buffer[9] = (unsigned int)(aux & 0xFFFFFFFFLL);
                      
                 send_message(10);  
            }
            
                
        }	
        if(flag==0){
        	buffer[0] = 0xAA000000;
        	  send_message(1); 
        }

        
        


    }


	return ShutdownHalt;
}
