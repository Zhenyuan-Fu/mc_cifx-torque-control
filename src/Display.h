#pragma once

#include <stdint.h>

/*****************************************************************************/
/*! Displays a hex dump on the debug console (16 bytes per line)
 *   \param pbData     Pointer to dump data
 *   \param ulDataLen  Length of data dump                                    */
/*****************************************************************************/
void DumpData(unsigned char * pbData, unsigned long ulDataLen);

/*****************************************************************************
 *! Displays cifX error
 *   \param lError     Error code
 *****************************************************************************/
void ShowError(int32_t lError);
