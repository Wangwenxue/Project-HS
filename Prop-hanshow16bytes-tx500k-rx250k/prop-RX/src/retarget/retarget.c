/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from ARM Limited.
 *
 *            (C) COPYRIGHT 2010-2011 ARM Limited.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from ARM Limited.
 *
 *      SVN Information
 *
 *      Checked In          : $Date: 2011-03-17 11:30:08 +0000 (Thu, 17 Mar 2011) $
 *
 *      Revision            : $Revision: 164920 $
 *
 *      Release Information : BP200-r0p0-00rel0
 *-----------------------------------------------------------------------------
 */

#include "driver_config.h"
#if defined (CFG_DBG_PRINT) && defined (CFG_STD_PRINTF)

#if defined ( __CC_ARM )
/******************************************************************************/
/* Retarget functions for ARM RVDS / Keil MDK                                 */
/******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <rt_misc.h>
#pragma import(__use_no_semihosting_swi)

extern unsigned char UartPutc(unsigned char my_ch);
extern unsigned char UartGetc(void);
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  return (UartPutc(ch));
}

int fgetc(FILE *f) {
  return (UartPutc(UartGetc()));
}

void _ttywrch(int ch) {
  UartPutc (ch);
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}

#elif defined  ( __ICCARM__ )
/******************************************************************************/
/* Retarget functions for IAR                                                 */
/******************************************************************************/

#include <stdio.h>

extern unsigned char UartPutc(unsigned char my_ch);
extern unsigned char UartGetc(void);

size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  /* Remove the #if #endif pair to enable the implementation */
  size_t nChars = 0;

  if (buffer == 0)
  {
    /*
     * This means that we should flush internal buffers.  Since we
     * don't we just return.  (Remember, "handle" == -1 means that all
     * handles should be flushed.)
     */
    return 0;
  }

  /* This template only writes to "standard out" and "standard err",
   * for all other file handles it returns failure. */

  for (/* Empty */; size != 0; --size)
  {
    UartPutc(*buffer++);

    ++nChars;
  }

  return nChars;

}

size_t __read(int handle, unsigned char * buffer, size_t size)
{
  /* Remove the #if #endif pair to enable the implementation */

  int nChars = 0;

  for (/* Empty */; size > 0; --size)
  {
    int c = UartGetc();
    if (c < 0)
      break;

    *buffer++ = c;
    ++nChars;
  }

  return nChars;
}

#elif defined ( __GNUC__ )
/******************************************************************************/
/* Retarget functions for gcc                                                 */
/******************************************************************************/
#include <stdio.h>
#include <sys/stat.h>

int i;
extern unsigned char UartPutc(unsigned char my_ch);

int _write_r(void *reent, int fd, char *ptr, size_t len)
{
  size_t i;
  for (i=0; i<len;i++) {
    UartPutc(ptr[i]); // call character output function in uart_stdout.c
    }
  return len;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _read(int file, char *ptr, int len)
{
    return 0;
}

#endif

#endif // defined (CFG_DBG_PRINT) && defined (CFG_STD_PRINTF)
