/********************************************************************************/
/********************************************************************************/
/*  File Purpose : The main file to configure the MCU Internal Resources		*/
/*	735280																		*/
/*  Source File : main.c														*/
/*																				*/
/*  Description : The HandHeld Sprey 		*/
/*																				*/
/*  Revision    : Ver 0.0														*/
/*																				*/
/*  Date Begin  : 08 August 2020												*/
/*																				*/
/*  Date End    : 																*/
/*																				*/
/*  CPU         : STM8S003F3P6													*/
/*																				*/
/*  Compiler    : ST Visual Develop (Version 4.3.1)								*/
/*																				*/
/*  Author      : Chong Chee Han												*/
/*																				*/
/*  CheckSum    : 																*/
/*																				*/
/*  (C) Copyright 2018 VS-Industry												*/
/********************************************************************************/
/********************************************************************************/
 
// https://blogs.yahoo.co.jp/hobbyele/65191789.html
// https://github.com/HomeSmartMesh/STM8_IoT_Boards
//https://github.com/tenbaht/sduino
// https://github.com/tenbaht/sduino/tree/master/sduino/hardware/sduino/stm8/libraries/Servo


#include "main.h"
#include "stm8s.h"
#include "delay.h"
//#include <stdio.h>
//#include <math.h>


/* Private typedef -----------------------------------------------------------*/
#undef USE_FULL_ASSERT
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO TestStatus OperationStatus;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

//bool ReadIO(void);
//void TestDelay(void);

union
{
    struct
    {
        uint8_t high;
        uint8_t low;
    }byte;
    uint16_t word;
}ADC_Value,VR_Value,NTC_Value;

volatile bool Key_status = FALSE;


uint8_t Rs232_RxBuf[RS232_RXSIZE]={ 0 };
uint8_t RxHead_Idx;	//RxWr_Idx;		// Received Link Buffer Write Index
uint8_t RxTail_Idx; //RxRd_Idx;		// Received Link Buffer Read Index

uint8_t Rs232_TxBuf[RS232_TXSIZE] = { 0 };
uint8_t TxHead_Idx;	//TxWr_Idx	// Transmit Link Buffer Write Index
uint8_t TxTail_Idx;	//TxRd_Idx	// Transmit Link Buffer Read Index

uint8_t getkeybuff[RS232_TXSIZE/2];
uint8_t length;
uint8_t lenghtSub;

uint8_t Flag;
uint8_t	keyDebounce; 
uint8_t _50mSec;
uint8_t _100mSec;
uint8_t _500mSec;
uint8_t _150mSec;  
uint8_t _300mSec;  
uint8_t _250mSec;
uint8_t _PC6Sec;

uint8_t Second;
uint8_t _1Sec;

uint8_t RxLength;
uint8_t RxTimer;
uint8_t UARTFLG ;
uint8_t uTimeCnt;
uint8_t uTimeCnt2;
uint8_t uTimeCnt3;
uint8_t m_uLEDStus;

uint16_t ADC_LDR[BUFFER_SIZE] = {0};

const char* ssid     = "VSoffice";
const char* password = "0000100001efg";
//fuad
void main(void)
{
	m_uLEDStus = 0;
	//---feature 1
	//comit 2
	//MyCLK_DeInit();
	
	CLK_Configuration();	// Configures clocks
	GPIO_Configuration();	// Configures GPIOs
	//TIM1_Configuration();	// Set Timer-1 as PWM output on CH3,CH4
	TIM2_Configuration();	// Configure Timer-2 as 10mS TICK Timer
	TIM4_Configuration();	// Not Use.
	UART1_Configuration();	// UART Configuration 115200 baud Rate
	//SPI_Configuration();
	ADC1_Configuration();
 	EXTI_Configuration();
	enableInterrupts();		//
	Clear_Memory();			//
	Rs232_InitCold();
	
//	Rs232_PutStr("\r");

//	Rs232_PutStr("<= Leminator Temperature Controller =>\r\n");

	ADC1->CR1 |= ADC1_CR1_ADON;
	//GPIO->ODR &= ~pin_mask;
	//GPIOB->ODR ^= (uint8_t)GPIO_PIN_5;
	//GPIOB->ODR &= ~((uint8_t)GPIO_PIN_5);
	//GPIOB->ODR = 0x00;
	//GPIO_WriteHigh(PIN_LEDBUILTIN);
	//GPIO_WriteLow(PIN_LEDBUILTIN);
	//GPIO_WriteLow(GPIOB,GPIO_PIN_4);
	while (1)
	{
		//Delay_uS(DELAY_10mS);
		softTimer();
		LEDC();
	}
}

void LEDC(void)
{
	//PC3
	if(_100mSecFlag == TRUE)
	{
		GPIO_WriteReverse(PIN_C3);  
		_100mSecFlag = FALSE;
	}
	//PC4
	if(_150mSecFlag == TRUE)
	{
		GPIO_WriteReverse(PIN_C4); 
		_150mSecFlag = FALSE;
	}
	//PC5
	if(_250mSecFlag == TRUE) //50ms
	{
		_250mSecFlag = FALSE;
		uTimeCnt3++;
		if(uTimeCnt3 >= 5)//500ms
		{
			GPIO_WriteReverse(PIN_C5); 
			uTimeCnt3 = 0;
		}
	}
	//PC6
	if(_PC6flag == TRUE) //100ms flg
	{
		_PC6flag = FALSE;
		uTimeCnt++;
		if(uTimeCnt >= 3)//300ms
		{
			GPIO_WriteLow(PIN_C6);
		}
		if(uTimeCnt >= 8)//500ms
		{
			GPIO_WriteHigh(PIN_C6);
			uTimeCnt = 0;
		}
	}
	//PC7 
	if(_PC7flag == TRUE) //50ms flg
	{
		_PC7flag = FALSE;
		uTimeCnt2++;
		if(uTimeCnt2 >= 10)//500ms
		{
			GPIO_WriteLow(PIN_C7);
		}
		if(uTimeCnt2 >= 15)//250ms
		{			
			GPIO_WriteHigh(PIN_C7);
			uTimeCnt2 = 0;
		}
	}
}

void ReadWriteEEPROM(void)
{
    uint8_t val = 0x00, val_comp = 0x00;
    uint32_t add = 0x00;

    /* Define FLASH programming time */
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
 //   FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);
 //   FLASH->CR1 |= (uint8_t)FLASH_PROGRAMTIME_STANDARD;

    /* Unlock Data memory */
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
//	FLASH->DUKR = FLASH_RASS_KEY2; /* Warning: keys are reversed on data memory !!! */
//	FLASH->DUKR = FLASH_RASS_KEY1;

    /* Read a byte at a specified address */
    add = 0x4000;
    val = FLASH_ReadByte(add);

    /* Program complement value (of previous read byte) at previous address + 1 */
    val_comp = (uint8_t)(~val);
    FLASH_ProgramByte((add + 1), val_comp);

    /* Check program action */
    val = FLASH_ReadByte((add + 1));
    if (val != val_comp)
    {
        /* Error */
        OperationStatus = FAILED;
        /* OperationStatus = PASSED, if the data written/read to/from DATA EEPROM memory is correct */
        /* OperationStatus = FAILED, if the data written/read to/from DATA EEPROM memory is corrupted */
        while (1)
        {
        }
    }

    /* Erase byte at a specified address & address + 1 */
    FLASH_EraseByte(add);
    FLASH_EraseByte((add + 1));
    /* Erase action */
    val = FLASH_ReadByte(add);
    val_comp = FLASH_ReadByte((add + 1));
    if ((val != 0x00) & (val_comp != 0x00))
    {
        /* Error */
  //      OperationStatus = FAILED;
        /* OperationStatus = PASSED, if the data written/read to/from DATA EEPROM memory is correct */
        /* OperationStatus = FAILED, if the data written/read to/from DATA EEPROM memory is corrupted */
        while (1)
        {

		
        }
    }

    /* Pass */
 //   OperationStatus = PASSED;
    /* OperationStatus = PASSED, if the data written/read to/from DATA EEPROM memory is correct */
    /* OperationStatus = FAILED, if the data written/read to/from DATA EEPROM memory is corrupted */
    while (1)
    {
    
    }
}

/*
********************************************************************************
** Function Name	: softTimer(void)
** Function Desc	: Written specificall for use with the DS18B20 temperature sensor.
** Input Parameter	: None
** Output Parameter : None
** Return Parameter : None
********************************************************************************
*/
void softTimer(void)
{
	if(_10mSecFlag)
	{
		_10mSecFlag = FALSE;
		
		_50mSec = (uint8_t)((_50mSec + 1) % _50MSEC);
		if(_50mSec == 0)
		{
			_50mSecFlag = TRUE; //50mSec
			_PC7flag = TRUE;
			_250mSecFlag = TRUE; //250mSec
			
			_150mSec = (uint8_t)((_150mSec + 1) % _150MSEC);
			if(_150mSec == 0)
			{
				_150mSecFlag = TRUE; //150mSec
				//GPIO_WriteReverse(PIN_LEDBUILTIN);
				
				_300mSec = (uint8_t)((_300mSec + 1) % _300MSEC);
				if(_300mSec == 0)
				{
					_300mSecFlag = TRUE; //300mSec
				}
			}
			_250mSec = (uint8_t)((_250mSec + 1) % _250MSEC);
			if(_250mSec == 0)
			{
				//_250mSecFlag = TRUE; //250mSec
				
			}
			
		}
		
		//-----------------------------
		_100mSec = (uint8_t)((_100mSec + 1) % _100MSEC);
		if(_100mSec == 0)
		{	
			_100mSecFlag = TRUE; //100mSec
			_PC6flag = TRUE;
			//GPIO_WriteReverse(PIN_LEDBUILTIN);
			_500mSec = (uint8_t)((_500mSec + 1) % _500MSEC);
			if(_500mSec == 0)
			{	
				_500mSecFlag = TRUE; //500mSec
				//GPIOB->ODR ^= (uint8_t)GPIO_PIN_5;
				//GPIO_WriteReverse(PIN_LEDBUILTIN);
				//_500mSDoneflag = FALSE;
				
				_1Sec = (uint8_t)((_1Sec + 1) % _1SEC);
				if(_1Sec == 0)
				{
					SecondFlag = TRUE; //1000mSec
					//printf("Hello World");
					//GPIO_WriteReverse(PIN_LEDBUILTIN);
					//Rs232_PutStr("\n Temperature is : ");
					//if (UART_FLAG == TRUE)
					//{
					//	GPIO_WriteReverse(PIN_LEDBUILTIN);
					//	UART_FLAG = FALSE;
					//}
				}
			}
		}
	}
}

/********************************************************************************
** Function Name	: updateSerial(void)
** Function Desc	: 
** Input Parameter   : None
** Output Parameter : None
** Return Parameter : None
*********************************************************************************/
void updateSerial(void)
{
	if(updagteFlag)
	{
		Rs232_PutStr("\n Temperature is : ");
		ByteToStr('0x00');  //, txt);bin_dec(temp_msb);
		Rs232_PutStr(" degC");
		updagteFlag = FALSE;
	}
}

/********************************************************************************
** Function Name	: hex2Ascii(void)
** Function Desc	: Convert the Hex Number to String without using library
** Input Parameter   : Temperature in Hex Number
** Output Parameter : None
** Return Parameter : None
*********************************************************************************/

void hex2Ascii(uint8_t *cp, uint8_t len) //convert value range 0 ~ FFFE
{
	uint8_t i;
	uint8_t buffer_1;
	uint8_t data;
	uint8_t retval = 0;
	
	for (i=0; i<len; i++)
	{
		data = *cp;
		buffer_1 = (uint8_t)((data >> 4) & 0x0f);
		if(buffer_1 >= 0x09 )
			buffer_1 += 0x57;
		else
			buffer_1 |= 0x30;
		
		Rs232_PutChar(buffer_1);
		buffer_1 = (uint8_t)(data & 0x0f);
		if(buffer_1 >= 0x09 )
			buffer_1 += 0x57;
		else
			buffer_1 |= 0x30;
		Rs232_PutChar(buffer_1);
		Rs232_PutChar(EMTY);
		cp++;
	}
	Rs232_PutStr("\r\n");	
}

/********************************************************************************
** Function Name	: ByteToStr(void)
** Function Desc	: Convert the Hex Number to String without using library
** Input Parameter   : Temperature in Hex Number
** Output Parameter : None
** Return Parameter : None
*********************************************************************************/
void ByteToStr(uint16_t temperature) //, uint8_t txt)
{
	uint16_t n = temperature; //INT_MIN;
	char buffer[5];
	uint8_t i = 0 , t = 0;

	bool isNeg = (bool)(n < 0);

	unsigned int n1 = isNeg ? -n : n;

	while(n1!=0)
	{
	    buffer[i++] = (uint8_t)(n1%10+'0');
	    n1=n1/10;
	}

	if(isNeg)
	    buffer[i++] = '-';

	buffer[i] = '\0';

	for( t = 0; t < i/2; t++)
	{
	    buffer[t] ^= buffer[i-t-1];
	    buffer[i-t-1] ^= buffer[t];
	    buffer[t] ^= buffer[i-t-1];
	}

	if(n == 0)
	{
	    buffer[0] = '0';
	    buffer[1] = '\0';
	}   
	Rs232_PutStr(buffer);
	
}

/********************************************************************************
** Function Name	: bin_dec(void)
** Function Desc	: 
** Input Parameter   : None
** Output Parameter : None
** Return Parameter : None
*********************************************************************************/
void bin_dec(uint8_t BinNum)   // Function Definition
{
	uint8_t remainder , DecNum = 0;
	uint8_t pow = 1;
	
	while(BinNum != 0)
	{
		remainder = (uint8_t)(BinNum % 10);
		DecNum = (uint8_t)(DecNum + remainder * pow);
		pow = (uint8_t)(pow * 2);
		BinNum = (uint8_t)(BinNum / 10);
//		sum = sum + rem * pow(2,power);
//		data = data / 10;
//		power++;
	}
}

/*
********************************************************************************
** Function Name	: BuzzerSound(void)
** Function Desc	: Setting the Buzzer Generator
** Input Parameter   : None
** Output Parameter : None
** Return Parameter : None
********************************************************************************
*/
void BuzzerSound(void)
{
    BEEP->CSR = 0x6E;	// 2kHz
}

/*
********************************************************************************
** Function Name	: Reload_Timer4(void)
** Function Desc	: Written specificall for use with the DS18B20 temperature sensor.
** Input Parameter	: None
** Output Parameter : None
** Return Parameter : None
********************************************************************************
void Reload_Timer4(uint8_t NewValue)
{
	TIM4_TimeBaseInit(TIM4_PRESCALER_8, NewValue);
	TIM4_Cmd(ENABLE);
	while(TIM4_GetFlagStatus(!TIM4_FLAG_UPDATE));
	TIM4_Cmd(DISABLE);

	TIM4_ClearITPendingBit(TIM4_IT_UPDATE);

}
*/
/*
********************************************************************************
** Function Name	: Reload_Timer4(void)
** Function Desc	: Written specificall for use with the DS18B20 temperature sensor.
** Input Parameter	: None
** Output Parameter : None
** Return Parameter : None
********************************************************************************/
uint16_t ADC1_samplingValue(uint8_t ADC_PORT)
{
    uint16_t temph = 0;
    uint8_t templ = 0;

	ADC1->CSR &= 0xF0;		// Clear 
	ADC1->CSR |= ADC_PORT;

	// Triggers the start of conversion, after ADC1 configuration.
	ADC1->CR1 |= ADC1_CR1_ADON;

	Delay_uS(DELAY_10US)
	
	while((ADC1->CSR & ADC1_FLAG_EOC )!= RESET)
	{
		// Clear ADC1 flag
        ADC1->CSR &= (uint8_t) (~ADC1_FLAG_EOC);
	    // Read LSB first 
	    ADC_low = ADC1->DRL;
	    // Then read MSB 
	    ADC_high = ADC1->DRH;
    }
	return (ADC_10BitValue);
}

/*********************************************************************************
** Function Name	: Rs232_PutChar(void)
** Function Desc	: Store data to buffer and transmit data .
** Input Parameter   : Data
** Output Parameter : None
** Return Parameter : Data
*********************************************************************************/
uint8_t Rs232_PutChar(uint8_t Data)
{
	if(((TxHead_Idx + 1) % RS232_TXSIZE) != TxTail_Idx)
	{	// Check Buffer if not full, continue the storage
		Rs232_TxBuf[TxHead_Idx] = Data;	// Store Data
		TxHead_Idx = (uint8_t)((TxHead_Idx + 1) % RS232_TXSIZE);

		if (!TxSbufFull) 
		UART1->CR2 |= UART_CR2_TCIEN;	// Enable Transmission complete interrupt enable
		return 0;
	}
	return 1;
}

/*********************************************************************************
** Function Name	: Rs232_PutStr(void)
** Function Desc	:Write String Data to transmit buffer.
** Input Parameter   : Pointer
** Output Parameter : None
** Return Parameter : None
*********************************************************************************/
uint8_t Rs232_PutStr(uint8_t *cp)
{
	uint8_t Wr_Idx = TxHead_Idx;	// Prevent error occur will not update the TxHead_Idx.
	
	for (; *cp; cp++)
	{
		if (((Wr_Idx + 1) % RS232_TXSIZE) != TxTail_Idx) 
		{	// Check Buffer if not full, continue the storage
			Rs232_TxBuf[Wr_Idx] = *cp;
			Wr_Idx = (uint8_t)((Wr_Idx + 1) % RS232_TXSIZE);
		}
		else 
		{
			return ERROR;			
		}
	}
	TxHead_Idx = Wr_Idx;	// To prevent if error occur, it will not update TxHead_Idx
	if (!TxSbufFull)
		UART1->CR2 |= UART_CR2_TCIEN;	// Enable Transmission complete interrupt enable
		
	return SUCCESS;
}

/*
********************************************************************************
** Function Name	: Rs232_Enable(void)
** Function Desc	: Enable the Transmission and Receiving
** Input Parameter   : None
** Output Parameter : None
** Return Parameter : None
********************************************************************************
*/
void Rs232_InitCold(void)
{
//	RxWr_Idx = 0;	// Received Write Index
//	RxRd_Idx = 0;
	
	RxHead_Idx = 0;	// Received Write Index
	RxTail_Idx = 0;

	TxHead_Idx = 0;
	TxTail_Idx = 0;	// Transmit Read Index

	TxSbufFull = FALSE;

//	TR1 = 1;	// Start timer 1(TCON)
//	ES0 = 1;	//(IE.4)
}

void Clear_Memory(void)
{
	/* Enable time counter display*/
	Flag = 0;
	RxHead_Idx = 0;
	RxTail_Idx = 0;
	TxHead_Idx = 0;
	TxTail_Idx = 0;
	keyDebounce = 0;
	RxLength = 0;
	TxSbufFull = FALSE;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


