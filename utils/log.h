/************************************************************************
 *
* FILENAME: 	log.h
* DATE: 		17.03.2022
* DESCRIPTION: 	Log module
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
#ifndef LOG_H
#define LOG_H
/************************************************************************
 *
 * INCLUDES
 *
************************************************************************/
#include "SEGGER_RTT.h"
/************************************************************************
 *
 * DEFINES AND MACROS
 *
************************************************************************/
#define LOG_SEGGER_CONSOLE_IDX 0

#define LOG_INFO(x, ...)    SEGGER_RTT_printf(LOG_SEGGER_CONSOLE_IDX, "I: " x, ##__VA_ARGS__)
#define LOG_DEBUG(x, ...)   SEGGER_RTT_printf(LOG_SEGGER_CONSOLE_IDX, "DEBUG: " x, ##__VA_ARGS__)
#define LOG_ERROR(x, ...)   SEGGER_RTT_printf(LOG_SEGGER_CONSOLE_IDX, "ERROR: " x, ##__VA_ARGS__)
#define LOG_RAW(x, ...)     SEGGER_RTT_printf(LOG_SEGGER_CONSOLE_IDX, x, ##__VA_ARGS__)
/************************************************************************
 *
 * ENUMS AND STRUCTS
 *
************************************************************************/

/************************************************************************
 *
 * FUNCTIONS
 *
************************************************************************/
/************************************************************************
*
* FUNCTION: 	  Log_PrintByteArray
* PARAMS: 		  [in] buf - byte array to be printed out
* PARAMS: 		  [in] len - length of byte array to be printed out
* RETVAL: 		  none
* DESCRIPTION:	  Prints out the byte array into log console
* AUTHOR: 		  Embedded Systems Adam Bodurka
*
************************************************************************/
void Log_PrintByteArray(uint8_t *buf, uint32_t len);

#endif // LOG_H
