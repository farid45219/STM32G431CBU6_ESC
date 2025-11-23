

#ifndef  _DEBUG_H_
#define  _DEBUG_H_

#include "stm32g431xx.h"

enum{
  DEBUG_FALSE                  = 0x00,
  DEBUG_TRUE                   = 0x01,
  DEBUG_NULL                   = 0x00,
	
	DEBUG_CMD_WRITE              = 0x00,
	DEBUG_CMD_READ               = 0x01,
	
	//Autonomous mode operates itself, no dependency on data input
	DEBUG_MODE_SYSTEM_AUTONOMOUS = 0x02,
	
	//Extensive mode can control many variabels via data input
	DEBUG_MODE_SYSTEM_EXTENSIVE  = 0x03,
	
	//Override mode can control only few variabels via data input
	DEBUG_MODE_SYSTEM_OVERRIDE   = 0x04,
	
	DEBUG_READ_NO_READ_REQ       = 0x00,
	DEBUG_READ_STATUS_REG        = 0x01,
	DEBUG_READ_BURST_REG         = 0x02,
	DEBUG_READ_INS_REG           = 0x03,
	DEBUG_READ_FCEVF_REG         = 0x04,
	DEBUG_READ_PCEVF_REG         = 0x05,
	DEBUG_READ_COF_REG           = 0x06,
	DEBUG_READ_EVH_REG           = 0x07,
	DEBUG_READ_VAS_REG           = 0x08,
	
	DEBUG_ERROR_NO_ERROR           = 0x00,
	DEBUG_ERROR_CRC_MISMATCH       = 0x01,
	DEBUG_ERROR_INVALID_CMD        = 0x02,
	DEBUG_ERROR_LEN_MISMATCH       = 0x04,
	DEBUG_ERROR_HEADER_MISMATCH    = 0x08,
	DEBUG_ERROR_DATA_OUT_OF_RANGE  = 0x10,
	DEBUG_ERROR_INVALID_PARAMETER  = 0x20
};

void     Debug_Struct_Init(void);
void     Debug_RX_Packet_Struct_Init(void);

void     Debug_Config_GPIO(void);
void     Debug_Config_Clock(void);
void     Debug_Config_BAUD_Rate(uint32_t baud_rate);
void     Debug_Config_Tx(void);
void     Debug_Config_Rx(void);
void     Debug_Config_Rx_Interrupt(void);
void     Debug_Clear_Interrupt_Flag(void);
void     Debug_Tx_Byte(uint8_t val);
uint8_t  Debug_Rx_Byte(void);
void     USART2_IRQHandler(void);


void     Debug_Timer_Struct_Init(void);
void     Debug_Timer_Init(void);
void     Debug_Timer_Enable(void);
void     Debug_Timer_Disable(void);
uint8_t  Debug_Timer_Get_Status(void);
uint16_t Debug_Timer_Get_Val(void);
void     Debug_Timer_Value_Reset(void);
void     Debug_Timer_Clear_Interrupt_Flag(void);
void     TIM17_IRQHandler(void);

void     Debug_Tx_Buf(uint8_t *data, uint8_t len);

void     Debug_Tx_NL(void);
void     Debug_Tx_SP(void);
void     Debug_Tx_CM(void);


void     Debug_Tx_Text(char *str);
void     Debug_Tx_Text_NL(char *str);
void     Debug_Tx_Text_SP(char *str);
void     Debug_Tx_Text_CM(char *str);


void     Debug_Determine_Digit_Numbers(uint32_t num);
void     Debug_Tx_Number_Digits(void);
void     Debug_Tx_Number(int32_t num);
void     Debug_Tx_Number_Hex(uint32_t val);
void     Debug_Tx_Number_Bin(uint32_t val);


void     Debug_Tx_Number_NL(int32_t num);
void     Debug_Tx_Number_SP(int32_t num);
void     Debug_Tx_Number_CM(int32_t num);


void     Debug_Tx_Number_Hex_NL(uint32_t num);
void     Debug_Tx_Number_Hex_SP(uint32_t num);
void     Debug_Tx_Number_Hex_CM(uint32_t num);


void     Debug_Tx_Number_Bin_NL(uint32_t num);
void     Debug_Tx_Number_Bin_SP(uint32_t num);
void     Debug_Tx_Number_Bin_CM(uint32_t num);


void     Debug_Tx_Parameter_NL(char *name, int32_t num);
void     Debug_Tx_Parameter_SP(char *name, int32_t num);
void     Debug_Tx_Parameter_CM(char *name, int32_t num);


void     Debug_Tx_Parameter_Hex_NL(char *name, uint32_t num);
void     Debug_Tx_Parameter_Hex_SP(char *name, uint32_t num);
void     Debug_Tx_Parameter_Hex_CM(char *name, uint32_t num);


void     Debug_Tx_Parameter_Bin_NL(char *name, uint32_t num);
void     Debug_Tx_Parameter_Bin_SP(char *name, uint32_t num);
void     Debug_Tx_Parameter_Bin_CM(char *name, uint32_t num);

//Receiver Functions
void     Debug_Buf_Flush(void);
void     Debug_Buf_Set(uint16_t index, uint8_t val);
uint8_t  Debug_Buf_Get(uint16_t index);
uint16_t Debug_Buf_Get_Index(void);


//Debug Data Functions
uint8_t  Debug_Data_Available(void);
uint16_t Debug_Data_Len_Get(void);

uint16_t Debug_Data_Calculated_CRC_Get(void);
uint16_t Debug_Data_Received_CRC_Get(void);
uint8_t  Debug_Data_CRC_Status_Get(void);
uint8_t  Debug_Data_Read_Complete_Status(void);

void     Debug_Data_Clear_Available_Flag(void);
void     Debug_Data_Clear_Read_Complete_Flag(void);

void     Debug_Data_Copy_Buf(uint8_t *buf);
void     Debug_Data_Print_Buf(void);

uint8_t  Debug_Error_Code_Get(void);
void     Debug_Error_Code_Clear(void);

uint32_t Debug_Accumulated_CRC_Error_Get(void);

void     Debug_ISR_Handler(void);
void     Debug_Timer_ISR_Handler(void);


uint16_t Debug_CRC_Calculate_Byte(uint16_t crc, uint8_t data);
uint16_t Debug_CRC_Calculate_Block(volatile uint8_t *buf, uint8_t len);

void     Debug_RX_Packet_CRC_Check(void);
void     Debug_RX_Packet_Rough_Check_Disassemble(void);
void     Debug_Error_Check(void);
void     Debug_Extensive_Response(void);
void     Debug_Override_Response(void);

void     Debug_Override_Status(void);
void     Debug_Override_Burst(void);
void     Debug_Override_InS(void);
void     Debug_Override_FCEvF(void);
void     Debug_Override_PCEvF(void);
void     Debug_Override_CoF(void);
void     Debug_Override_EvH(void);
void     Debug_Override_VaS(void);


void     Debug_RX_Packet_Read_Complete(void);

uint8_t  Debug_Mode_Get(void);
int16_t  Debug_Mode_Extensive_CaS1_Get(void);
int16_t  Debug_Mode_Extensive_DeS1_Get(void);
int16_t  Debug_Mode_Extensive_CaS2_Get(void);
int16_t  Debug_Mode_Extensive_DeS2_Get(void);
int16_t  Debug_Mode_Extensive_AmS_Get(void);
int16_t  Debug_Mode_Extensive_DoS1_Get(void);
int16_t  Debug_Mode_Extensive_DoS2_Get(void);
uint8_t  Debug_Mode_Override_InS_Get(void);
uint8_t  Debug_Mode_Override_EvF1_Get(void);
uint8_t  Debug_Mode_Override_EvF2_Get(void);
uint8_t  Debug_Mode_Override_CoF_Get(void);
uint8_t  Debug_Mode_Override_EvH_Get(void);
uint8_t  Debug_Mode_Override_VaS_Get(void);

void     Debug_Init(uint32_t baud);

#endif

