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
#define INPUT_DISTANCE "input_distance_1_5"
#define INPUT_CHARGE "input_charge_1_5"
#define GOLD "output_gold_1_5"
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------200
//	DEFINE / INCLUDE
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------200

#define fp double

#define NUMBER_PAR_PER_BOX 100							// keep this low to allow more blocks that share shared memory to run concurrently, code does not work for larger than 110, more speedup can be achieved with larger number and no shared memory used

//===============================================================================================================================================================================================================200
//	STRUCTURES
//===============================================================================================================================================================================================================200

typedef struct
{
    fp x, y, z;

} THREE_VECTOR;

typedef struct
{
    fp v, x, y, z;

} FOUR_VECTOR;

typedef struct nei_str
{

    // neighbor box
    int x, y, z;
    int number;
    long offset;

} nei_str;

typedef struct box_str
{

    // home box
    int x, y, z;
    int number;
    long offset;

    // neighbor boxes
    int nn;
    nei_str nei[26];

} box_str;

typedef struct par_str
{

    fp alpha;

} par_str;

typedef struct dim_str
{

    // input arguments
    int cur_arg;
    int arch_arg;
    int cores_arg;
    int boxes1d_arg;

    // system memory
    long number_boxes;
    long box_mem;
    long space_elem;
    long space_mem;
    long space_mem2;

} dim_str;



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
};

#endif
