#include "cifXErrors.h"
#include "cifXUser.h"

#include <stdio.h>

void DumpData(unsigned char * pbData, unsigned long ulDataLen)
{
  unsigned long ulIdx;
#ifdef DEBUG
  printf("%s() called\n", __FUNCTION__);
#endif
  for(ulIdx = 0; ulIdx < ulDataLen; ++ulIdx)
  {
    if(0 == (ulIdx % 16)) printf("\r\n");

    printf("%02X ", pbData[ulIdx]);
  }
  printf("\r\n");
}

void ShowError(int32_t lError)
{
  if(lError != CIFX_NO_ERROR)
  {
    char szError[1024] = {0};
    xDriverGetErrorDescription(lError, szError, sizeof(szError));
    printf("Error: 0x%X, <%s>\n", (unsigned int)lError, szError);
  }
}
