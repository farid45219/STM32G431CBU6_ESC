
/*
 * File:   debug.c
 * Author: MD. Faridul Islam
 * faridmdislam@gmail.com
 * Created on December 12, 2024, 9:44 PM
 */


/*
 * Uses USART2 with Interrupt Priority 3
 * Uses TIM17  with Interrupt Priority 4
 */
 
 

#include "stm32g431xx.h"
//must be included
#include "debug.h"


#define  DEBUG_DOUBLE_SPEED
#define  DEBUG_ENABLE_TX    
//#define  DEBUG_ENABLE_RX    
//#define  DEBUG_ENABLE_RX_INT

#define  DEBUG_BUFFER_SIZE            128U
#define  DEBUG_RX_PCKT_CMPLT_DELAY    20U



#define  DEBUG_CRC_ENABLE     //Uncomment if packet validation by CRC is needed
#define  DEBUG_CRC_XMODEM     //Uncomment if CRC is by X-MODEM Protocol
//#define  DEBUG_CRC_ALTERNATE     //Uncomment if CRC is by Alternative Protocol




//Define Software Error Codes
#define  DEBUG_RX_ERR_NO_ERR          0x00U
#define  DEBUG_RX_ERR_FRAMING         0x01U
#define  DEBUG_RX_ERR_OVERRUN         0x02U
#define  DEBUG_RX_ERR_READ_INCOMPLETE 0x10U


typedef struct debug_timer_t{
  volatile uint8_t   Enabled;
  volatile uint8_t   ResetVal;
}debug_timer_t;

typedef struct debug_rx_packet_t{
  volatile uint16_t  CalculatedCRC;
  volatile uint16_t  ReceivedCRC;
	volatile uint8_t   CalculatedChecksum;
	volatile uint8_t   ReceivedChecksum;
  volatile uint8_t   CRCStatus;
  volatile uint8_t   DataAvailable;
  volatile uint8_t   DataReadComplete;
	volatile uint8_t   Header;
	volatile uint8_t   Length;
	volatile uint8_t   Command;
	volatile uint8_t   Register;
	volatile uint8_t   Reserved0;
	volatile uint16_t  Reserved1;
	volatile uint32_t  AccuCRCError;
}debug_rx_packet_t;

typedef struct debug_extensive_t{
  volatile int16_t   CaS1;
	volatile int16_t   DeS1;
	volatile int16_t   CaS2;
	volatile int16_t   DeS2;
	volatile int16_t   AmS;
	volatile int16_t   DoS1;
	volatile int16_t   DoS2;
}debug_extensive_t;

typedef struct debug_override_t{
  volatile uint8_t   InS;
	volatile uint8_t   EvF1;
	volatile uint8_t   EvF2;
	volatile uint8_t   CoF;
	volatile uint8_t   EvH;
	volatile uint8_t   VaS;
	volatile uint8_t   ReadOp;
	volatile uint8_t   NackRet;
	volatile uint8_t   AckRet;
	volatile uint8_t   Error;
}debug_override_t;

typedef struct debug_t{
  volatile uint8_t   Error;
  uint8_t            Digits[8];
  uint8_t            InputNumDigits;
  volatile uint8_t   LastRxByte;
  volatile uint8_t   Buf[DEBUG_BUFFER_SIZE];
  volatile uint8_t   Mode;
  volatile uint16_t  BufSize;
  volatile uint16_t  BufIndex;
  volatile uint16_t  Reserved3;
  debug_timer_t      Timer;
  debug_rx_packet_t  RxPacket;
	debug_extensive_t  Extensive;
	debug_override_t   Override;
}debug_t;





static debug_t Debug;






/*******************Debug Structure Functions Start****************/

void Debug_Struct_Init(void){
  Debug.Error = 0;
  for(uint8_t i = 0; i < 8; i++){
    Debug.Digits[i] = DEBUG_NULL;
  }
  Debug.InputNumDigits = DEBUG_NULL;
  Debug.LastRxByte = DEBUG_NULL;
  Debug.BufSize = DEBUG_BUFFER_SIZE;
  Debug.BufIndex = 0;
  for(uint8_t i = 0; i < Debug.BufSize; i++){
    Debug.Buf[i] = DEBUG_NULL;
  }
	
	//Select Debug Mode, Default Mode:: Autonomous
	Debug.Mode = DEBUG_MODE_SYSTEM_AUTONOMOUS;
	
	//Extensive Mode Parameters
	Debug.Extensive.CaS1  = DEBUG_NULL;
	Debug.Extensive.DeS1  = DEBUG_NULL;
	Debug.Extensive.CaS2  = DEBUG_NULL;
	Debug.Extensive.DeS2  = DEBUG_NULL;
	Debug.Extensive.AmS   = DEBUG_NULL;
	Debug.Extensive.DoS1  = DEBUG_NULL;
	Debug.Extensive.DoS2  = DEBUG_NULL;
	
	//Override Mode Parameters
	Debug.Override.InS    = DEBUG_NULL;
	Debug.Override.EvF1   = DEBUG_NULL;
	Debug.Override.EvF2   = DEBUG_NULL;
	Debug.Override.CoF    = DEBUG_NULL;
	Debug.Override.EvH    = DEBUG_NULL;
	Debug.Override.VaS    = DEBUG_NULL;
	Debug.Override.ReadOp = DEBUG_READ_NO_READ_REQ;
	Debug.Override.NackRet = DEBUG_FALSE;
	Debug.Override.AckRet  = DEBUG_FALSE;
	Debug.Override.Error   = DEBUG_NULL;
}

void Debug_RX_Packet_Struct_Init(void){
  Debug.RxPacket.CalculatedCRC    = DEBUG_NULL;
  Debug.RxPacket.ReceivedCRC      = DEBUG_NULL;
  Debug.RxPacket.CRCStatus        = DEBUG_FALSE;
  Debug.RxPacket.DataAvailable    = DEBUG_FALSE;
  Debug.RxPacket.DataReadComplete = DEBUG_FALSE;
	Debug.RxPacket.AccuCRCError     = 0;
}

/********************UART Structure Functions End*****************/









/*********************Debug Init Functions Start******************/

void Debug_Config_GPIO(void){
	//PB3->TX->AF7, PB4->RX->AF7
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	//Config PB3 as alternate Func
	GPIOB->MODER &=~ GPIO_MODER_MODE3_Msk;
	GPIOB->MODER |=  GPIO_MODER_MODE3_1;
	
	//Enable Pull-up in PB3
	GPIOB->PUPDR &=~ GPIO_PUPDR_PUPD3_Msk;
	GPIOB->PUPDR |=  GPIO_PUPDR_PUPD3_0;
	
	//Select TX in AFR
	GPIOB->AFR[0]&=~ GPIO_AFRL_AFSEL3_Msk;
	GPIOB->AFR[0]|=  (7<<GPIO_AFRL_AFSEL3_Pos);

	//Config PB4 as alternate Func
	GPIOB->MODER &=~ GPIO_MODER_MODE4_Msk;
	GPIOB->MODER |=  GPIO_MODER_MODE4_1;
	
	//Enable Pull-up in PB4
	GPIOB->PUPDR &=~ GPIO_PUPDR_PUPD4_Msk;
	GPIOB->PUPDR |=  GPIO_PUPDR_PUPD4_0;
	
	//Select RX in AFR
	GPIOB->AFR[0]&=~ GPIO_AFRL_AFSEL4_Msk;
	GPIOB->AFR[0]|=  (7<<GPIO_AFRL_AFSEL4_Pos);
}

void Debug_Config_Clock(void){
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
}

void Debug_Config_BAUD_Rate(uint32_t baud_rate){
	if(USART2->CR1 & USART_CR1_UE){
    USART2->CR1 &=~USART_CR1_UE;
	}
  USART2->BRR   = (uint16_t)(16000000/baud_rate);
}


void Debug_Config_Tx(void){
	USART2->CR1   |= USART_CR1_TE;
	if((USART2->CR1 & USART_CR1_UE) != USART_CR1_UE){
	  USART2->CR1 |= USART_CR1_UE;
	}
}


void Debug_Config_Rx(void){
  USART2->CR1   |= USART_CR1_RE;
	if((USART2->CR1 & USART_CR1_UE) != USART_CR1_UE){
	  USART2->CR1 |= USART_CR1_UE;
	}
}

void Debug_Config_Rx_Interrupt(void){
  USART2->CR1  |= USART_CR1_RXNEIE_RXFNEIE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 3);
}

void Debug_Clear_Interrupt_Flag(void){
  //Not necessary, auto cleared when RXDR read
}

void Debug_Tx_Byte(uint8_t val){
  USART2->TDR=val;
	while((USART2->ISR & USART_ISR_TC)!=USART_ISR_TC);
	USART2->ICR|=USART_ICR_TCCF;	                                                                                                 
}

uint8_t Debug_Rx_Byte(void){
  volatile uint8_t val = 0;
  val = (uint8_t)USART2->RDR;
  return val;
}

//add Debug int handler vector
//call Debug_ISR_Handler() inside ISR

void USART2_IRQHandler(void){
	Debug_ISR_Handler();
}

/**********************Debug Init Functions End*******************/









/********************Debug Timer Functions Start*****************/

void Debug_Timer_Struct_Init(void){
  Debug.Timer.Enabled = DEBUG_FALSE;
  Debug.Timer.ResetVal = DEBUG_NULL;
}

void Debug_Timer_Init(void){
  //config Debug timer for auto packet validation
	//calculate Debug.Timer.ResetVal if overflow int is used
	//timer resolution should be 1ms
	//if other than 1ms is used, adjust DEBUG_RX_PCKT_CMPLT_DELAY
	//TIM14 for Timebase
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	TIM7->PSC    = 15999;
	TIM7->ARR    = DEBUG_RX_PCKT_CMPLT_DELAY;
	TIM7->DIER  |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 4);
	
	/*
  #if DEBUG_RX_PCKT_CMPLT_DELAY<20U
    #warning DEBUG_RX_PCKT_CMPLT_DELAY value < 20
  #endif
	*/
}

void Debug_Timer_Enable(void){
  TIM7->CR1   |= TIM_CR1_CEN;
}

void Debug_Timer_Disable(void){
  TIM7->CR1   &=~TIM_CR1_CEN;
}

uint8_t Debug_Timer_Get_Status(void){
  return Debug.Timer.Enabled;
}

uint16_t Debug_Timer_Get_Val(void){
  //return current timer val
	return (uint16_t)TIM7->CNT;
}


void Debug_Timer_Value_Reset(void){
	//reset timer val if compare mode selected
	TIM7->CNT = 0;
	//set timer val to Debug.Timer.ResetVal if Overflow int is used
}

void Debug_Timer_Clear_Interrupt_Flag(void){
  //Clear flag if 
	TIM7->SR &=~ TIM_SR_CC1IF;
	TIM7->SR &=~ TIM_SR_UIF;
}

//add Debug timer int handler vector
//call Debug_Timer_ISR_Handler() inside ISR

void TIM7_IRQHandler(void){
	Debug_Timer_ISR_Handler();
}


/*********************Debug Timer Functions End******************/









/********************Buffer Tx Functions Start*******************/

void Debug_Tx_Buf(uint8_t *data, uint8_t len){
  for(uint16_t i = 0; i < len; i++){
	  Debug_Tx_Byte( data[i] );
  }
}

/*********************Buffer Tx Functions End********************/









/*******************End Char Functions Start******************/

void Debug_Tx_NL(void){
  Debug_Tx_Byte('\r');
  Debug_Tx_Byte('\n');
}

void Debug_Tx_SP(void){
  Debug_Tx_Byte(' ');
}

void Debug_Tx_CM(void){
  Debug_Tx_Byte(',');
}

/*******************End Char Functions End*******************/









/*********************Text Functions Start*******************/

void Debug_Tx_Text(char *str){
  uint8_t i = 0;
  while(str[i] != '\0'){
    Debug_Tx_Byte(str[i]);
    i++;
  }
}

void Debug_Tx_Text_NL(char *str){
  Debug_Tx_Text(str);
  Debug_Tx_NL();
}

void Debug_Tx_Text_SP(char *str){
  Debug_Tx_Text(str);
  Debug_Tx_SP();
}

void Debug_Tx_Text_CM(char *str){
  Debug_Tx_Text(str);
  Debug_Tx_CM();
}

/*********************Text Functions End********************/









/*********************Number Functions Start********************/

void Debug_Determine_Digit_Numbers(uint32_t num){
  uint8_t i = 0;
  if(num == 0){
    Debug.Digits[0] = 0;
    Debug.InputNumDigits = 1;
  }else{
    while(num != 0){
      Debug.Digits[i] = num%10;
      num /= 10;
      i++;
    }
	Debug.InputNumDigits = i;
  }
}

void Debug_Tx_Number_Digits(void){
  for(uint8_t i = Debug.InputNumDigits; i > 0; i--){
    uint8_t temp = i;
    temp -= 1;
    temp  = Debug.Digits[temp];
    temp += 48;
    Debug_Tx_Byte(temp);
  }
}

void Debug_Tx_Number(int32_t num){
  if(num < 0){
    Debug_Tx_Byte('-');
	  num = -num;
  }
  Debug_Determine_Digit_Numbers((uint32_t)num);
  Debug_Tx_Number_Digits();
}

void Debug_Tx_Number_Hex(uint32_t val){
  uint16_t hex_digit, index = 0, loop_counter = 0;
  if(val <= 0xFF){
    index = 8;
    loop_counter = 2;
  }else if(val <= 0xFFFF){
    index = 16;
    loop_counter = 4;     
  }else{
    index = 32;
    loop_counter = 8;
  }
  Debug_Tx_Byte('0');
  Debug_Tx_Byte('x');
  for(uint8_t i = 0; i < loop_counter; i++){
	index -= 4;
	hex_digit = (uint8_t)((val>>index) & 0x0F);
	if(hex_digit > 9){
	  hex_digit += 55;
	}
	else{
	  hex_digit += 48;
	}
	Debug_Tx_Byte((uint8_t)hex_digit);
  }
}

void Debug_Tx_Number_Bin(uint32_t val){
  uint8_t loop_counter = 0;
  if(val <= 0xFF){
    loop_counter = 7;
  }else if(val <= 0xFFFF){
    loop_counter = 15;     
  }else{
    loop_counter = 31;
  }
  
  Debug_Tx_Byte('0');
  Debug_Tx_Byte('b');
  for(int i = loop_counter; i >= 0; i--){
    if( (val>>i) & 1){
      Debug_Tx_Byte( 49 );   
    }else{
      Debug_Tx_Byte( 48 );         
    }
  }
}

/*********************Number Functions End*********************/









/************Number with End Char Functions Start**************/

void Debug_Tx_Number_NL(int32_t num){
  Debug_Tx_Number(num);
  Debug_Tx_NL();
}

void Debug_Tx_Number_SP(int32_t num){
  Debug_Tx_Number(num);
  Debug_Tx_SP();
}

void Debug_Tx_Number_CM(int32_t num){
  Debug_Tx_Number(num);
  Debug_Tx_CM();
}

/*************Number with End Char Functions End***************/









/**********Hex Number with End Char Functions Start************/

void Debug_Tx_Number_Hex_NL(uint32_t num){
  Debug_Tx_Number_Hex(num);
  Debug_Tx_NL();
}

void Debug_Tx_Number_Hex_SP(uint32_t num){
  Debug_Tx_Number_Hex(num);
  Debug_Tx_SP();
}

void Debug_Tx_Number_Hex_CM(uint32_t num){
  Debug_Tx_Number_Hex(num);
  Debug_Tx_CM();
}

/***********Hex Number with End Char Functions End*************/









/**********Bin Number with End Char Functions Start************/

void Debug_Tx_Number_Bin_NL(uint32_t num){
  Debug_Tx_Number_Bin(num);
  Debug_Tx_NL();
}

void Debug_Tx_Number_Bin_SP(uint32_t num){
  Debug_Tx_Number_Bin(num);
  Debug_Tx_SP();
}

void Debug_Tx_Number_Bin_CM(uint32_t num){
  Debug_Tx_Number_Bin(num);
  Debug_Tx_CM();
}

/***********Bin Number with End Char Functions End*************/







/************Number with Parameter Functions Start*************/

void Debug_Tx_Parameter_NL(char *name, int32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_NL(num);
}

void Debug_Tx_Parameter_SP(char *name, int32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_SP(num);
}

void Debug_Tx_Parameter_CM(char *name, int32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_CM(num);
}

/*************Number with Parameter Functions End**************/









/**********Hex Number with Parameter Functions Start***********/

void Debug_Tx_Parameter_Hex_NL(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Hex_NL(num);
}

void Debug_Tx_Parameter_Hex_SP(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Hex_SP(num);
}

void Debug_Tx_Parameter_Hex_CM(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Hex_CM(num);
}

/***********Hex Number with Parameter Functions End************/









/**********Bin Number with Parameter Functions Start***********/

void Debug_Tx_Parameter_Bin_NL(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Bin_NL(num);
}

void Debug_Tx_Parameter_Bin_SP(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Bin_SP(num);
}

void Debug_Tx_Parameter_Bin_CM(char *name, uint32_t num){
  Debug_Tx_Text(name);
  Debug_Tx_SP();
  Debug_Tx_Number_Bin_CM(num);
}

/***********Bin Number with Parameter Functions End************/









/*******************Debug Buffer Functions Start***************/

void Debug_Buf_Flush(void){
  for(uint8_t i = 0; i < DEBUG_BUFFER_SIZE; i++){
	Debug.Buf[i] = 0;
  }
  Debug.BufIndex = 0;
}

void Debug_Buf_Set(uint16_t index, uint8_t val){
  Debug.Buf[index] = val;
}

uint8_t Debug_Buf_Get(uint16_t index){
  return Debug.Buf[index];
}

uint16_t Debug_Buf_Get_Index(void){
  return Debug.BufIndex;
}

/********************Debug Buffer Functions End****************/









/*******************Debug Data Functions Start****************/

uint8_t Debug_Data_Available(void){
  return Debug.RxPacket.DataAvailable;
}

uint16_t Debug_Data_Len_Get(void){
  return Debug_Buf_Get_Index();
}

uint16_t Debug_Data_Calculated_CRC_Get(void){
  return Debug.RxPacket.CalculatedCRC;
}

uint16_t Debug_Data_Received_CRC_Get(void){
  return Debug.RxPacket.ReceivedCRC;
}

uint8_t Debug_Data_CRC_Status_Get(void){
  return Debug.RxPacket.CRCStatus;
}

uint8_t Debug_Data_Read_Complete_Status(void){
  return Debug.RxPacket.DataReadComplete;
}

void Debug_Data_Clear_Available_Flag(void){
  Debug.RxPacket.DataAvailable = DEBUG_FALSE;
}

void Debug_Data_Clear_Read_Complete_Flag(void){
  Debug_Buf_Flush();
  Debug.RxPacket.DataReadComplete = DEBUG_TRUE;
}


void Debug_Data_Copy_Buf(uint8_t *buf){
  for(uint16_t i = 0; i < Debug_Data_Len_Get(); i++){
	  buf[i] = Debug_Buf_Get(i);
  }
}


void Debug_Data_Print_Buf(void){
  for(uint16_t i = 0; i < Debug_Data_Len_Get(); i++){
	  //Debug_Tx_Byte( Debug_Buf_Get(i) );
		Debug_Tx_Byte( Debug_Buf_Get(i) );
  }
  //Debug_Tx_NL();
	Debug_Tx_NL();
}

/********************Debug Data Functions End*****************/









/******************Error Code Functions Start****************/

uint8_t Debug_Error_Code_Get(void){
  return Debug.Error;
}

void Debug_Error_Code_Clear(void){
  Debug.Error = 0;
}

/******************Error Code Functions End******************/









/**********Accumulated CRC Error Functions Start************/

uint32_t Debug_Accumulated_CRC_Error_Get(void){
  return Debug.RxPacket.AccuCRCError;
}

/**********Accumulated CRC Error Functions Stop*************/









/***************Debug ISR Handler Functions Start************/

void Debug_ISR_Handler(void){
  Debug_Clear_Interrupt_Flag();
  Debug.LastRxByte = (uint8_t)Debug_Rx_Byte();
  if(Debug.Error == 0x00){
    Debug.Buf[Debug.BufIndex] = Debug.LastRxByte;
    Debug.BufIndex++;
    if(Debug.BufIndex >= Debug.BufSize){
      Debug.BufIndex = 0;
    }
  }
  else{
    Debug.LastRxByte = DEBUG_NULL;
  }
  
  Debug_Timer_Value_Reset();
  if(Debug.Timer.Enabled == DEBUG_FALSE){
	  Debug_Timer_Enable();
	  Debug.Timer.Enabled = DEBUG_TRUE;
  }
  
}

void Debug_Timer_ISR_Handler(void){
  Debug_Timer_Clear_Interrupt_Flag();
  if(Debug.Timer.Enabled == DEBUG_TRUE){
    Debug_Timer_Disable();
	  Debug.Timer.Enabled = DEBUG_FALSE;
  }
  
  if(Debug_Buf_Get_Index() != DEBUG_NULL){
    
	  if(Debug.RxPacket.DataReadComplete == DEBUG_FALSE){
	    Debug.Error = DEBUG_RX_ERR_READ_INCOMPLETE;
	  }
    Debug_RX_Packet_CRC_Check();
    #ifdef DEBUG_CRC_ENABLE
	  if(Debug.RxPacket.CRCStatus == DEBUG_TRUE){
	    Debug.RxPacket.DataAvailable = DEBUG_TRUE;
	  }
	  else{
	    Debug_Buf_Flush();
	    Debug.RxPacket.DataAvailable = DEBUG_FALSE;
	  }
	  #else
	  Debug.RxPacket.DataAvailable = DEBUG_TRUE;
	  #endif
	
	  Debug.RxPacket.DataReadComplete = DEBUG_FALSE;
  }
}

/****************Debug ISR Handler Functions End*************/









/******************Debug CRC Functions Start****************/

#ifdef   DEBUG_CRC_XMODEM

uint16_t Debug_CRC_Calculate_Byte(uint16_t crc, uint8_t data){
	uint16_t temp = data;
	temp <<= 8;
  crc = crc ^ temp;
  for(uint8_t i = 0; i < 8; i++){
    if(crc & 0x8000){
			temp   = crc;
			temp <<= 0x01;
			temp  ^= 0x1021;
	    crc = temp;
	  }
    else{
	    crc <<= 1;
	  }
  }
  return crc;
}

uint16_t Debug_CRC_Calculate_Block(volatile uint8_t *buf, uint8_t len){
  uint16_t crc = 0;
  for(uint8_t i = 0; i < len; i++){
    crc = Debug_CRC_Calculate_Byte(crc,buf[i]);
  }
  return crc;
}
#endif

#ifdef   DEBUG_CRC_ALTERNATE

static uint16_t CRCTable[16] = {
 0x0000, 0xCC01, 0xD801, 0x1400,
 0xF001, 0x3C00, 0x2800, 0xE401,
 0xA001, 0x6C00, 0x7800, 0xB401,
 0x5000, 0x9C01, 0x8801, 0x4400
};


uint16_t Debug_CRC_Calculate_Block(volatile uint8_t *buf, uint8_t len){
 uint16_t crc = 0xFFFF, i;
 uint8_t  Data;
 for (i = 0; i < len; i++) {
  Data = *buf++;
  crc = CRCTable[(Data ^ crc) & 0x0f] ^ (crc >> 4);
  crc = CRCTable[((Data >> 4) ^ crc) & 0x0f] ^ (crc >> 4);
 }
 crc = (uint16_t)(((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF));
 return crc;
}

#endif

/*******************Debug CRC Functions End*****************/









/*************Debug RX Packet Functions Start***************/

void Debug_RX_Packet_CRC_Check(void){
	uint8_t  temp = 0;
  uint16_t crc_calc = 0, crc_recv = 0;
  if( Debug_Data_Len_Get() > 2){
		temp  = (uint8_t)Debug_Data_Len_Get();
		temp -= 2;
    crc_calc   =  Debug_CRC_Calculate_Block(Debug.Buf, temp);
    crc_recv   =  Debug_Buf_Get(Debug_Data_Len_Get() - 2);
    crc_recv <<= 8;
    crc_recv  |=  Debug_Buf_Get(Debug_Data_Len_Get() - 1);
  }
  Debug.RxPacket.CalculatedCRC = crc_calc;
  Debug.RxPacket.ReceivedCRC = crc_recv;
  if( Debug.RxPacket.CalculatedCRC == Debug.RxPacket.ReceivedCRC ){
    Debug.RxPacket.CRCStatus = DEBUG_TRUE;
	}
	else{
		Debug.RxPacket.CRCStatus = DEBUG_FALSE;
		Debug.RxPacket.AccuCRCError++;
	}
}




void Debug_RX_Packet_Read_Complete(void){
  Debug_Buf_Flush();
  Debug_Data_Clear_Available_Flag();
  Debug_Data_Clear_Read_Complete_Flag();
	Debug_Error_Code_Clear();
}

/**************Debug RX Packet Functions End****************/










/*****************Debug Init Functions Start****************/

void Debug_Init(uint32_t baud){
  Debug_Struct_Init();
  Debug_RX_Packet_Struct_Init();
  Debug_Timer_Struct_Init();
  
  Debug_Config_GPIO();
  Debug_Config_Clock();
  Debug_Config_BAUD_Rate(baud);
  
  #ifdef DEBUG_ENABLE_TX  
  Debug_Config_Tx();
  #endif
  
  #ifdef DEBUG_ENABLE_RX
  Debug_Config_Rx();
  #endif
  
  #ifdef DEBUG_ENABLE_RX_INT
  Debug_Config_Rx_Interrupt();
  #endif
  
  Debug_Timer_Init();
  Debug_Buf_Flush();
}

/******************Debug Init Functions End*****************/


