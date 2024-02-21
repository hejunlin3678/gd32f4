#include "systick.h"
#include "ymodem/common.h"


#if 0
extern uint8_t file_name[FILE_NAME_LENGTH];
uint8_t tab_1024[1024] =
  {
    0
  };


/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
void SerialDownload(void)
{
  uint8_t Number[10] = "          ";
  int32_t Size = 0;

  SerialPutString("Waiting for the file to be sent ... (press 'a' to abort)\n\r");
  Size = Ymodem_Receive(&tab_1024[0]);
	
  delay_1ms(100);	

  if (Size > 0)
  {
    SerialPutString("\n\n\r Programming Completed Successfully!\n\r\r\n[ Name: ");
    SerialPutString(file_name);
    Int2Str(Number, Size);
    SerialPutString(",Size: ");
    SerialPutString(Number);
    SerialPutString(" Bytes]\r\n");
  }
  else if (Size == -1)
  {
    SerialPutString("\n\n\rThe image size is higher than the allowed space memory!\n\r");
  }
  else if (Size == -2)
  {
    SerialPutString("\n\n\rVerification failed!\n\r");
  }
  else if (Size == -3)
  {
    SerialPutString("\r\n\nAborted by user.\n\r");
  }
  else
  {
    SerialPutString("\n\rFailed to receive the file!\n\r");
  }
}
#endif

/*************************************END OF FILE***************************************/
