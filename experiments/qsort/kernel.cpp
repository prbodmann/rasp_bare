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
#include <alloca.h>
#include <limits.h>
#include <math.h>
#include <circle/alloc.h>
static const char FromKernel[] = "kernel";
#define PARTITION	"emmc1-1"

/* Single iteration of the transient solver in the grid model.
 * advances the solution of the discretized difference equations
 * by one time step*/


/* Byte-wise swap two items of size SIZE. */
#define SWAP(a, b, size)                                                      \
  do                                                                              \
    {                                                                              \
      size_t __size = (size);                                                      \
      char *__a = (a), *__b = (b);                                              \
      do                                                                      \
        {                                                                      \
          char __tmp = *__a;                                                      \
          *__a++ = *__b;                                                      \
          *__b++ = __tmp;                                                      \
        } while (--__size > 0);                                                      \
    } while (0)
/* Discontinue quicksort algorithm when partition gets below this size.
   This particular magic number was chosen to work best on a Sun 4/260. */
#define MAX_THRESH 4
/* Stack node declarations used to store unfulfilled partition obligations. */
typedef struct
  {
    char *lo;
    char *hi;
  } stack_node;
/* The next 4 #defines implement a very fast in-line stack abstraction. */
/* The stack needs log (total_elements) entries (we could even subtract
   log(MAX_THRESH)).  Since total_elements has type size_t, we get as
   upper bound for log (total_elements):
   bits per byte (CHAR_BIT) * sizeof(size_t).  */
#define STACK_SIZE        (CHAR_BIT * sizeof (size_t))
#define PUSH(low, high)        ((void) ((top->lo = (low)), (top->hi = (high)), ++top))
#define        POP(low, high)        ((void) (--top, (low = top->lo), (high = top->hi)))
#define        STACK_NOT_EMPTY        (stack < top)
/* Order size using quicksort.  This implementation incorporates
   four optimizations discussed in Sedgewick:
   1. Non-recursive, using an explicit stack of pointer that store the
      next array partition to sort.  To save time, this maximum amount
      of space required to store an array of SIZE_MAX is allocated on the
      stack.  Assuming a 32-bit (64 bit) integer for size_t, this needs
      only 32 * sizeof(stack_node) == 256 bytes (for 64 bit: 1024 bytes).
      Pretty cheap, actually.
   2. Chose the pivot element using a median-of-three decision tree.
      This reduces the probability of selecting a bad pivot value and
      eliminates certain extraneous comparisons.
   3. Only quicksorts TOTAL_ELEMS / MAX_THRESH partitions, leaving
      insertion sort to order the MAX_THRESH items within each partition.
      This is a big win, since insertion sort is faster for small, mostly
      sorted array segments.
   4. The larger of the two sub-partitions is always pushed onto the
      stack first, with the algorithm then concentrating on the
      smaller partition.  This *guarantees* no more than log (total_elems)
      stack size is needed (actually O(1) in this case)!  */

//void qsort(void *base, size_t nitems, size_t size, int (*compar)(const void *, const void*));
int compare(const void *elem1, const void *elem2);
int compare(const void *elem1, const void *elem2)
{
  /* D = [(x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2]^(1/2) */
  /* sort based on distances from the origin... */
 // printf("hello\n\r");
  double distance1, distance2;

  distance1 = *((double*)elem1);
  distance2 = *((double*)elem2);
//printf("%f %f %d",distance1,distance2,(distance1 > distance2) ? 1 : ((distance1 < distance2) ? -1 : 0));
  return (distance1 > distance2) ? 1 : ((distance1 < distance2) ? -1 : 0);
}

void
qsort (void *const pbase, size_t total_elems, size_t size)
{
  char *base_ptr = (char *) pbase;
  const size_t max_thresh = MAX_THRESH * size;
  if (total_elems == 0)
    /* Avoid lossage with unsigned arithmetic below.  */
    return;
  if (total_elems > MAX_THRESH)
    {
      char *lo = base_ptr;
      char *hi = &lo[size * (total_elems - 1)];
      stack_node stack[STACK_SIZE];
      stack_node *top = stack;
      PUSH (NULL, NULL);
      while (STACK_NOT_EMPTY)
        {
          char *left_ptr;
          char *right_ptr;
          /* Select median value from among LO, MID, and HI. Rearrange
             LO and HI so the three values are sorted. This lowers the
             probability of picking a pathological pivot value and
             skips a comparison for both the LEFT_PTR and RIGHT_PTR in
             the while loops. */
          char *mid = lo + size * ((hi - lo) / size >> 1);
          if (compare ((void *) mid, (void *) lo) < 0)
            SWAP (mid, lo, size);
          if (compare ((void *) hi, (void *) mid) < 0)
            SWAP (mid, hi, size);
          else
            goto jump_over;
          if (compare ((void *) mid, (void *) lo) < 0)
            SWAP (mid, lo, size);
        jump_over:;
          left_ptr  = lo + size;
          right_ptr = hi - size;
          /* Here's the famous ``collapse the walls'' section of quicksort.
             Gotta like those tight inner loops!  They are the main reason
             that this algorithm runs much faster than others. */
          do
            {
              while (compare ((void *) left_ptr, (void *) mid) < 0)
                left_ptr += size;
              while (compare ((void *) mid, (void *) right_ptr) < 0)
                right_ptr -= size;
              if (left_ptr < right_ptr)
                {
                  SWAP (left_ptr, right_ptr, size);
                  if (mid == left_ptr)
                    mid = right_ptr;
                  else if (mid == right_ptr)
                    mid = left_ptr;
                  left_ptr += size;
                  right_ptr -= size;
                }
              else if (left_ptr == right_ptr)
                {
                  left_ptr += size;
                  right_ptr -= size;
                  break;
                }
            }
          while (left_ptr <= right_ptr);
          /* Set up pointers for next iteration.  First determine whether
             left and right partitions are below the threshold size.  If so,
             ignore one or both.  Otherwise, push the larger partition's
             bounds on the stack and continue sorting the smaller one. */
          if ((size_t) (right_ptr - lo) <= max_thresh)
            {
              if ((size_t) (hi - left_ptr) <= max_thresh)
                /* Ignore both small partitions. */
                POP (lo, hi);
              else
                /* Ignore small left partition. */
                lo = left_ptr;
            }
          else if ((size_t) (hi - left_ptr) <= max_thresh)
            /* Ignore small right partition. */
            hi = right_ptr;
          else if ((right_ptr - lo) > (hi - left_ptr))
            {
              /* Push larger left partition indices. */
              PUSH (lo, right_ptr);
              lo = left_ptr;
            }
          else
            {
              /* Push larger right partition indices. */
              PUSH (left_ptr, hi);
              hi = right_ptr;
            }
        }
    }
  /* Once the BASE_PTR array is partially sorted by quicksort the rest
     is completely sorted using insertion sort, since this is efficient
     for partitions below MAX_THRESH size. BASE_PTR points to the beginning
     of the array to sort, and END_PTR points at the very last element in
     the array (*not* one beyond it!). */
#define min(x, y) ((x) < (y) ? (x) : (y))
  {
    char *const end_ptr = &base_ptr[size * (total_elems - 1)];
    char *tmp_ptr = base_ptr;
    char *thresh = min(end_ptr, base_ptr + max_thresh);
    char *run_ptr;
    /* Find smallest element in first threshold and place it at the
       array's beginning.  This is the smallest array element,
       and the operation speeds up insertion sort's inner loop. */
    for (run_ptr = tmp_ptr + size; run_ptr <= thresh; run_ptr += size)
      if (compare ((void *) run_ptr, (void *) tmp_ptr) < 0)
        tmp_ptr = run_ptr;
    if (tmp_ptr != base_ptr)
      SWAP (tmp_ptr, base_ptr, size);
    /* Insertion sort, running from left-hand-side up to right-hand-side.  */
    run_ptr = base_ptr + size;
    while ((run_ptr += size) <= end_ptr)
      {
        tmp_ptr = run_ptr - size;
        while (compare ((void *) run_ptr, (void *) tmp_ptr) < 0)
          tmp_ptr -= size;
        tmp_ptr += size;
        if (tmp_ptr != run_ptr)
          {
            char *trav;
            trav = run_ptr + size;
            while (--trav >= run_ptr)
              {
                char c = *trav;
                char *hi, *lo;
                for (hi = lo = trav; (lo -= size) >= tmp_ptr; hi = lo)
                  *hi = *lo;
                *hi = c;
              }
          }
      }
  }
}


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


#define MAXARRAY 2000000
double distance[MAXARRAY],distance_temp[MAXARRAY];
double temp_gold[MAXARRAY];
void CKernel::initGold(char* gold_file){
    unsigned hFile = m_FileSystem.FileOpen (gold_file);    
    int i,j;
    m_FileSystem.FileRead(hFile,temp_gold, sizeof(double) * MAXARRAY);
   
    m_FileSystem.FileClose (hFile);
}
void CKernel::initInput(char* input_file){
    unsigned hFile = m_FileSystem.FileOpen (input_file);    
    int i,j;
    m_FileSystem.FileRead(hFile,distance, sizeof(double) *MAXARRAY);
    m_FileSystem.FileClose (hFile);
}

void qsort(void *base, size_t nitems, size_t size, int (*compar)(const void *, const void*));
//---------------------------------------------------------------------------


TShutdownMode CKernel::Run (void)
{
    int i;
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
	
    initGold("qsort_gold_2000000.bin");
    initInput("qsort_input_2000000.bin");
    while(1)
    {
        
        memcpy(distance_temp,distance,sizeof(double)*MAXARRAY);
        
        //printf("0\n");

        //status_app    = 0x00000000;
        //########### control_dut ###########
     
        qsort(distance_temp,MAXARRAY,sizeof(double));
        int num_SDCs = 0;
        for (i=0;i<MAXARRAY-1;i++)
        {
            /*char test[200];
            long *test1=(long*)&distance[i];
            long *test2=(long*)&temp_gold[i];
            sprintf(test,"gold[%d]=0x%08lx%08lx;\n\r",i,test1[1],test1[0]);
            //sprintf(test,"gold[%d]=0x%lx%lx;\n\r",i,test1[1],test1[0]);
            printf (test);*/
            //printf("gold[%d]=0x%llx;\n",i,*((long long *)&distance[i]));
            if (*((long long *)&distance_temp[i]) != *((long long *)&temp_gold[i]))
            {

                num_SDCs++;


            }
            //printf("a");
        }
        if (num_SDCs!=0)
        {
            buffer[0] = 0xDD000000;
            buffer[1] = num_SDCs;
            send_message(2);
        }else{
            buffer[0] = 0xAA000000;
            send_message(1);
        }

    }    
	return ShutdownHalt;
}
