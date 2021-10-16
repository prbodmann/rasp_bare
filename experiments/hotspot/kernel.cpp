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


void CKernel::single_iteration(FLOAT *result, FLOAT *temp, FLOAT *power, int row, int col,
                      FLOAT Cap_1, FLOAT Rx_1, FLOAT Ry_1, FLOAT Rz_1,
                      FLOAT step)
{

    FLOAT delta;
    int r, c;
    int chunk;
    int num_chunk = row*col / (BLOCK_SIZE_R * BLOCK_SIZE_C);
    int chunks_in_row = col/BLOCK_SIZE_C;
    int chunks_in_col = row/BLOCK_SIZE_R;

    for ( chunk = 0; chunk < num_chunk; ++chunk )
    {
        int r_start = BLOCK_SIZE_R*(chunk/chunks_in_col);
        int c_start = BLOCK_SIZE_C*(chunk%chunks_in_row);
        int r_end = r_start + BLOCK_SIZE_R > row ? row : r_start + BLOCK_SIZE_R;
        int c_end = c_start + BLOCK_SIZE_C > col ? col : c_start + BLOCK_SIZE_C;

        if ( r_start == 0 || c_start == 0 || r_end == row || c_end == col )
        {
            for ( r = r_start; r < r_start + BLOCK_SIZE_R; ++r ) {
                for ( c = c_start; c < c_start + BLOCK_SIZE_C; ++c ) {
                    /* Corner 1 */
                    if ( (r == 0) && (c == 0) ) {
                        delta = (Cap_1) * (power[0] +
                                           (temp[1] - temp[0]) * Rx_1 +
                                           (temp[col] - temp[0]) * Ry_1 +
                                           (amb_temp - temp[0]) * Rz_1);
                    }	/* Corner 2 */
                    else if ((r == 0) && (c == col-1)) {
                        delta = (Cap_1) * (power[c] +
                                           (temp[c-1] - temp[c]) * Rx_1 +
                                           (temp[c+col] - temp[c]) * Ry_1 +
                                           (   amb_temp - temp[c]) * Rz_1);
                    }	/* Corner 3 */
                    else if ((r == row-1) && (c == col-1)) {
                        delta = (Cap_1) * (power[r*col+c] +
                                           (temp[r*col+c-1] - temp[r*col+c]) * Rx_1 +
                                           (temp[(r-1)*col+c] - temp[r*col+c]) * Ry_1 +
                                           (   amb_temp - temp[r*col+c]) * Rz_1);
                    }	/* Corner 4	*/
                    else if ((r == row-1) && (c == 0)) {
                        delta = (Cap_1) * (power[r*col] +
                                           (temp[r*col+1] - temp[r*col]) * Rx_1 +
                                           (temp[(r-1)*col] - temp[r*col]) * Ry_1 +
                                           (amb_temp - temp[r*col]) * Rz_1);
                    }	/* Edge 1 */
                    else if (r == 0) {
                        delta = (Cap_1) * (power[c] +
                                           (temp[c+1] + temp[c-1] - 2.0*temp[c]) * Rx_1 +
                                           (temp[col+c] - temp[c]) * Ry_1 +
                                           (amb_temp - temp[c]) * Rz_1);
                    }	/* Edge 2 */
                    else if (c == col-1) {
                        delta = (Cap_1) * (power[r*col+c] +
                                           (temp[(r+1)*col+c] + temp[(r-1)*col+c] - 2.0*temp[r*col+c]) * Ry_1 +
                                           (temp[r*col+c-1] - temp[r*col+c]) * Rx_1 +
                                           (amb_temp - temp[r*col+c]) * Rz_1);
                    }	/* Edge 3 */
                    else if (r == row-1) {
                        delta = (Cap_1) * (power[r*col+c] +
                                           (temp[r*col+c+1] + temp[r*col+c-1] - 2.0*temp[r*col+c]) * Rx_1 +
                                           (temp[(r-1)*col+c] - temp[r*col+c]) * Ry_1 +
                                           (amb_temp - temp[r*col+c]) * Rz_1);
                    }	/* Edge 4 */
                    else if (c == 0) {
                        delta = (Cap_1) * (power[r*col] +
                                           (temp[(r+1)*col] + temp[(r-1)*col] - 2.0*temp[r*col]) * Ry_1 +
                                           (temp[r*col+1] - temp[r*col]) * Rx_1 +
                                           (amb_temp - temp[r*col]) * Rz_1);
                    }
                    result[r*col+c] =temp[r*col+c]+ delta;
                }
            }
            continue;
        }

        for ( r = r_start; r < r_start + BLOCK_SIZE_R; ++r ) {
         
            for ( c = c_start; c < c_start + BLOCK_SIZE_C; ++c ) {
                /* Update Temperatures */
                result[r*col+c] =temp[r*col+c]+
                                 ( Cap_1 * (power[r*col+c] +
                                            (temp[(r+1)*col+c] + temp[(r-1)*col+c] - 2.f*temp[r*col+c]) * Ry_1 +
                                            (temp[r*col+c+1] + temp[r*col+c-1] - 2.f*temp[r*col+c]) * Rx_1 +
                                            (amb_temp - temp[r*col+c]) * Rz_1));
            }
        }
    }
}

void CKernel::init_input(void){

unsigned hFile = m_FileSystem.FileOpen (TEMP_INPUT_FILENAME);
	unsigned nResult;
	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}


	while ((nResult = m_FileSystem.FileRead (hFile, temp, 4*MAX_SIZE*MAX_SIZE)) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF10000;
			send_message(1);
			break;
		}


	}


	if (!m_FileSystem.FileClose (hFile))
	{
		buffer[0]=0xFFF40000;
		send_message(1);	
	}

	hFile = m_FileSystem.FileOpen (POWER_INPUT_FILENAME);

	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}


	while ((nResult = m_FileSystem.FileRead (hFile, power, 4*MAX_SIZE*MAX_SIZE)) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF10000;
			send_message(1);
			break;
		}

	}


	if (!m_FileSystem.FileClose (hFile))
	{
		buffer[0]=0xFFF40000;
		send_message(1);	
	}
	hFile = m_FileSystem.FileOpen (GOLD_FILENAME);

	if (hFile == 0)
	{
		buffer[0]=0xFFF00000;
		send_message(1);
	}


	while ((nResult = m_FileSystem.FileRead (hFile, gold, 4*MAX_SIZE*MAX_SIZE)) > 0)
	{
		if (nResult == FS_ERROR)
		{
			buffer[0]=0xFFF10000;
			send_message(1);
			break;
		}

	}


	if (!m_FileSystem.FileClose (hFile))
	{
		buffer[0]=0xFFF40000;
		send_message(1);	
	}
}

void CKernel::compute_tran_temp(FLOAT *result, int num_iterations, FLOAT *temp, FLOAT *power, int row, int col)
{

    FLOAT grid_height = chip_height / row;
    FLOAT grid_width = chip_width / col;

    FLOAT Cap = FACTOR_CHIP * SPEC_HEAT_SI * t_chip * grid_width * grid_height;
    FLOAT Rx = grid_width / (2.0 * K_SI * t_chip * grid_height);
    FLOAT Ry = grid_height / (2.0 * K_SI * t_chip * grid_width);
    FLOAT Rz = t_chip / (K_SI * grid_height * grid_width);

    FLOAT max_slope = MAX_PD / (FACTOR_CHIP * t_chip * SPEC_HEAT_SI);
    FLOAT step = PRECISION / max_slope / 1000.0;

    FLOAT Rx_1=1.f/Rx;
    FLOAT Ry_1=1.f/Ry;
    FLOAT Rz_1=1.f/Rz;
    FLOAT Cap_1 = step/Cap;

    FLOAT* r = result;
    FLOAT* t = temp;
    int i = 0;
    for (i = 0; i < num_iterations ; i++)
    {
        single_iteration(r, t, power, row, col, Cap_1, Rx_1, Ry_1, Rz_1, step);
        FLOAT* tmp = t;
        t = r;
        r = tmp;
    }
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
	 while(1){      



          /* regular*/
        compute_tran_temp(result,sim_time, temp, power, MAX_SIZE, MAX_SIZE);

        int flag=0;

        for (i=0; i < MAX_SIZE; i++) {
            int j;
            for (j=0; j < MAX_SIZE; j++) {
                if (result[i*MAX_SIZE+j] != gold[i*MAX_SIZE+j]) {
                //if(final_result[i*MAX_SIZE+j] != gold[i*MAX_SIZE+j] ) {
                    if(flag==0){
                            buffer[0] = 0xDD000000;
                            flag=1;
                        }else{
                            buffer[0] = 0xCC000000;
                        }                        
                        buffer[1] = *((unsigned int*)&i);
                        buffer[2] = *((unsigned int*)&j);
                        unsigned long long aux=*((unsigned long long*)&result[i*MAX_SIZE+j]);
                        buffer[3] = (unsigned int)((aux & 0xFFFFFFFF00000000LL) >> 32);                      
                        buffer[3] = (unsigned int)(aux & 0xFFFFFFFFLL);
                        send_message(5);
                }
            }
        }
        if(flag==0){
            buffer[0] = 0xAA000000;
              send_message(1); 
        }
    }


	return ShutdownHalt;
}
