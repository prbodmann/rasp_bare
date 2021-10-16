//
// kernel.h
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
#ifndef _kernel_h
#define _kernel_h

#include <circle/actled.h>
#include <circle/koptions.h>
#include <circle/devicenameservice.h>
#include <circle/screen.h>
#include <circle/serial.h>
#include <circle/exceptionhandler.h>
#include <circle/interrupt.h>
#include <circle/timer.h>
#include <circle/logger.h>
#include <emmc.h>
#include <circle/fs/fat/fatfs.h>
#include <circle/types.h>
#include <circle/gpiomanager.h>
#include <softserial.h>


#define uint32_t unsigned int
#define PARTITION	"emmc1-1"
#define TEMP_INPUT_FILENAME "temp_1024"
#define POWER_INPUT_FILENAME "power_1024"
#define GOLD_FILENAME "hotspot_gold.bin"



#define MAX_ERR_ITER_LOG 500

#define BLOCK_SIZE 16
#define BLOCK_SIZE_C BLOCK_SIZE
#define BLOCK_SIZE_R BLOCK_SIZE

#define STR_SIZE	256

/* maximum power density possible (say 300W for a 10mm x 10mm chip)	*/
#define MAX_PD	(3.0e6)
/* required precision in degrees	*/
#define PRECISION	0.001
#define SPEC_HEAT_SI 1.75e6
#define K_SI 100
/* capacitance fitting factor	*/
#define FACTOR_CHIP	0.5

//#define NUM_THREAD 4

/* Define the precision to float or double depending on compiling flags */

#define FLOAT double 
#define MAX_SIZE 256

/* chip parameters	*/
const FLOAT t_chip = 0.0005;
const FLOAT chip_height = 0.016;
const FLOAT chip_width = 0.016;

/* ambient temperature, assuming no package at all	*/
const FLOAT amb_temp = 80.0;




enum TShutdownMode
{
	ShutdownNone,
	ShutdownHalt,
	ShutdownReboot
};

class CKernel
{
public:
	CKernel (void);
	~CKernel (void);

	boolean Initialize (void);
	void send_message(int size);
	void init_input(void);
void single_iteration(FLOAT *result, FLOAT *temp, FLOAT *power, int row, int col,
                      FLOAT Cap_1, FLOAT Rx_1, FLOAT Ry_1, FLOAT Rz_1,
                      FLOAT step);
 void compute_tran_temp(FLOAT *result, int num_iterations, FLOAT *temp, FLOAT *power, int row, int col);

	TShutdownMode Run (void);
	
private:
	// do not change this order
	CActLED			m_ActLED;
	CKernelOptions		m_Options;
	CDeviceNameService	m_DeviceNameService;
	CScreenDevice		m_Screen;
	CSerialDevice		m_Serial;
	CExceptionHandler	m_ExceptionHandler;
	CInterruptSystem	m_Interrupt;
	CTimer			m_Timer;
	CLogger			m_Logger;
	CGPIOManager		m_GPIOManager;	
	CSoftSerialDevice	m_SoftSerial;
	CEMMCDevice		m_EMMC;
	CFATFileSystem		m_FileSystem;
	unsigned int buffer[10];
	FLOAT temp[MAX_SIZE*MAX_SIZE];
	FLOAT power[MAX_SIZE*MAX_SIZE];
	FLOAT result[MAX_SIZE*MAX_SIZE];
	FLOAT gold[MAX_SIZE*MAX_SIZE];
	int sim_time =100;

};

#endif
