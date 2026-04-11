/*
* LIB By HungDK
*/
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H


#include "main.h"
#include "usart.h"
#include "Modbus_Config.h"

/* Enable DMA RX callback */
#define NUMBER_OF_MODBUS

/*
* Define table CRC16 Modbus
*/

extern const uint16_t crc16_modbus_table[256];

/*
 * Define Function code
 * Use for check fucntion
 */
typedef enum {
    // ========== READ Functions ==========
    MODBUS_FC_READ_COILS                    = 0x01,  // Đọc Coils (DO)
    MODBUS_FC_READ_DISCRETE_INPUTS          = 0x02,  // Đọc Discrete Inputs (DI)
    MODBUS_FC_READ_HOLDING_REGISTERS        = 0x03,  // Đọc Holding Registers
    MODBUS_FC_READ_INPUT_REGISTERS          = 0x04,  // Đọc Input Registers
    // ========== WRITE Functions ==========
    MODBUS_FC_WRITE_SINGLE_COIL             = 0x05,  // Ghi 1 Coil
    MODBUS_FC_WRITE_SINGLE_REGISTER         = 0x06,  // Ghi 1 Register
    MODBUS_FC_WRITE_MULTIPLE_COILS          = 0x0F,  // Ghi nhiều Coils
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS      = 0x10,  // Ghi nhiều Registers
    // ========== DIAGNOSTIC Functions ==========
    MODBUS_FC_READ_EXCEPTION_STATUS         = 0x07,  // Đọc Exception Status
    MODBUS_FC_DIAGNOSTIC                    = 0x08,  // Diagnostic
    MODBUS_FC_GET_COMM_EVENT_COUNTER        = 0x0B,  // Get Comm Event Counter
    MODBUS_FC_GET_COMM_EVENT_LOG            = 0x0C,  // Get Comm Event Log
    // ========== OTHER Functions ==========
    MODBUS_FC_READ_WRITE_MULTIPLE_REGISTERS = 0x17,  // Đọc/Ghi nhiều Registers
    MODBUS_FC_MASK_WRITE_REGISTER           = 0x16,  // Mask Write Register
    MODBUS_FC_READ_FIFO_QUEUE               = 0x18,  // Read FIFO Queue
    MODBUS_FC_READ_FILE_RECORD              = 0x14,  // Read File Record
    MODBUS_FC_WRITE_FILE_RECORD             = 0x15,  // Write File Record
    // ========== CUSTOM Functions (User-defined) ==========
    MODBUS_FC_CUSTOM_FUNCTION_65            = 0x41,  // Custom function
    MODBUS_FC_CUSTOM_FUNCTION_66            = 0x42,  // Custom function
}FC_Modbus;


/*
 * Struct for Object Modbus Slave Serial
 */
typedef struct{
	uint8_t BuffRxRaw[100];
	uint8_t BuffRx[100];
	uint8_t BuffTx[100];

	uint16_t StartAddr;
	uint16_t DataLenRx;
	uint16_t DataLenTx;
	uint16_t CrcRx;
	uint16_t CrcTx;
	uint16_t Quantity;

	uint8_t BuffRxSize;
	uint8_t BuffTxSize;
	uint8_t FlagRx;
	uint8_t FlagTx;
	uint8_t SlaveID;
	uint8_t PossStartData;

	FC_Modbus FunctionCode;
	UART_HandleTypeDef *huart;
	
	uint16_t BuffRxIndex;
	uint32_t LastTickRx;

}Modbus_Slave_Serial;

/*
 * Struct for Object Modbus Master Serial
 */
typedef struct{
	uint8_t BuffRxRaw[100];
	uint8_t BuffRx[100];
	uint8_t BuffTx[100];

	uint16_t StartAddr;
	uint16_t DataLenRx;
	uint16_t DataLenTx;
	uint16_t CrcRx;
	uint16_t CrcTx;
	uint16_t Quantity;

	uint8_t BuffRxSize;
	uint8_t BuffTxSize;
	uint8_t FlagRx;
	uint8_t FlagTx;
	uint8_t SlaveID;
	uint8_t PossStartData;

	FC_Modbus FunctionCode;
	UART_HandleTypeDef *huart;

}Modbus_Master_Serial;

/*
 * Define for Modbus
 */

 extern Modbus_Slave_Serial UsbCom;     // USB -> Uart 7 use for connect to master is Computer.
 extern Modbus_Slave_Serial Port1_485;  // Uart 3 -> RS485 Slave (PC Master)
 extern Modbus_Slave_Serial Port2_485; // Uart 4 -> Connect to PC Master

/*
 * Function prototypes for Modbus RTU operations
 */

void Modbus_init(void);
void Modbus_Service(void);
void Modbus_CheckHealth(void);
void Modbus_Detach_Frame(Modbus_Slave_Serial *Modbus);
void Modbus_Process_Frame(Modbus_Slave_Serial *Modbus);
uint16_t Modbus_CRC16(uint8_t *buff, uint16_t len); 

void Modbus_Resspon_Read_Holding_REG(Modbus_Slave_Serial* Slave, uint8_t *data);
void Modbus_Resspon_Write_Multiple_REG(Modbus_Slave_Serial* Slave);
void Modbus_Response_Write_Single_REG(Modbus_Slave_Serial* Slave);

void Modbus_Master_Send_Read_Holding_REG(Modbus_Master_Serial* Master, uint8_t SlaveID, uint16_t StartAddr, uint16_t Quantity);
void Modbus_Master_Send_Write_Multiple_REG(Modbus_Master_Serial* Master, uint8_t SlaveID, uint16_t StartAddr, uint16_t Quantity, uint8_t* Data);

#endif // MODBUS_RTU_H

