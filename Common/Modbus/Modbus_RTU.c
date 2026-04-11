/*
* LIB By HungDK - Cleaned & Optimized for Port 2 (UART4)
*/

#include "Modbus_RTU.h"
#include "Modbus_RegMap.h"
#include "car.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════
 *  CRC TABLE & HELPER
 * ═══════════════════════════════════════════════════════════════════ */
const uint16_t crc16_modbus_table[256] = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
    0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
    0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
    0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
    0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
    0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
    0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
    0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
    0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
    0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
    0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
    0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
    0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
    0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
    0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
    0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
    0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
    0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
    0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
    0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
    0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
    0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
    0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
    0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
    0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
    0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
    0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
    0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
    0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
    0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};

uint16_t Modbus_CRC16(uint8_t *buff, uint16_t len){
    uint16_t crc = 0xFFFF;
    while (len--) {
        uint8_t idx = (crc ^ *buff++) & 0xFF;
        crc = (crc >> 8) ^ crc16_modbus_table[idx];
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════
 *  SLAVE INSTANCES
 * ═══════════════════════════════════════════════════════════════════ */
Modbus_Slave_Serial UsbCom = {
    .huart = &huart7,
    .SlaveID = 0x89,
};

// Port 1 (Uart 3) -> Not used for PC main control right now
Modbus_Slave_Serial Port1_485 = {
    .huart = &huart3,
    .SlaveID = MODBUS_SLAVE_ID,
};

// Port 2 (Uart 4) -> PRIMARY PC INTERFACE
Modbus_Slave_Serial Port2_485 = {
    .huart = &huart4,
    .SlaveID = MODBUS_SLAVE_ID,
};

volatile Modbus_Diag_Typedef modbus_diag;
static const uint32_t MODBUS_FRAME_GAP_MS = 5U;

static uint8_t Modbus_Diag_PortId(UART_HandleTypeDef *huart) {
    if (huart == NULL) {
        return 0;
    }
    if (huart->Instance == USART3) {
        return 3;
    }
    if (huart->Instance == UART4) {
        return 4;
    }
    if (huart->Instance == UART7) {
        return 7;
    }
    return 0;
}

static Modbus_Slave_Serial *Modbus_FindSlave(UART_HandleTypeDef *huart) {
    if (huart == NULL) {
        return NULL;
    }
    if (huart->Instance == UART4) {
        return &Port2_485;
    }
    if (huart->Instance == USART3) {
        return &Port1_485;
    }
    if (huart->Instance == UART7) {
        return &UsbCom;
    }
    return NULL;
}

static void Modbus_ClearDiagState(void) {
    memset((void *)&modbus_diag, 0, sizeof(modbus_diag));
    modbus_regs.Array[REG_CMD_DIAG_LOOPBACK] = 0;
    modbus_regs.Array[REG_CMD_DIAG_RESET] = 0;
}

static void Modbus_ResetFrame(Modbus_Slave_Serial *slave) {
    if (slave == NULL) {
        return;
    }

    slave->BuffRxIndex = 0;
    slave->BuffRxSize = 0;
    slave->FlagRx = 0;
    slave->LastTickRx = 0;
}

static HAL_StatusTypeDef Modbus_StartRxDMA(Modbus_Slave_Serial *slave, uint8_t force_abort) {
    HAL_StatusTypeDef status;

    if (slave == NULL || slave->huart == NULL || slave->huart->hdmarx == NULL) {
        return HAL_ERROR;
    }

    if (force_abort != 0U) {
        (void)HAL_UART_AbortReceive(slave->huart);
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(slave->huart, slave->BuffRxRaw, sizeof(slave->BuffRxRaw));
    if (status == HAL_BUSY) {
        (void)HAL_UART_AbortReceive(slave->huart);
        status = HAL_UARTEx_ReceiveToIdle_DMA(slave->huart, slave->BuffRxRaw, sizeof(slave->BuffRxRaw));
    }

    __HAL_DMA_DISABLE_IT(slave->huart->hdmarx, DMA_IT_HT);

    if (status != HAL_OK) {
        modbus_diag.uart_error_count++;
        modbus_diag.last_port = Modbus_Diag_PortId(slave->huart);
        modbus_diag.last_status = 7;
    }

    return status;
}

static void Modbus_RestartRx(Modbus_Slave_Serial *slave) {
    if (slave == NULL) {
        return;
    }

    Modbus_ResetFrame(slave);
    (void)Modbus_StartRxDMA(slave, 1U);
}

/* ═══════════════════════════════════════════════════════════════════
 *  SYSTEM CALLBACKS
 * ═══════════════════════════════════════════════════════════════════ */
#ifdef NUMBER_OF_MODBUS
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    system_debug.brake_debug++; // ABSOLUTE RX HIT COUNT
    
    Modbus_Slave_Serial* slave = Modbus_FindSlave(huart);

    if(slave != NULL){
        modbus_diag.rx_event_count++;
        modbus_diag.last_port = Modbus_Diag_PortId(huart);
        modbus_diag.last_rx_size = Size;
        modbus_diag.last_status = 1;

        // Modbus RTU inter-frame gap at 115200 is sub-millisecond.
        if (HAL_GetTick() - slave->LastTickRx > MODBUS_FRAME_GAP_MS) {
            slave->BuffRxIndex = 0;
        }
        
        // Append data to accumulator
        uint16_t copy_size = (Size > (100 - slave->BuffRxIndex)) ? (100 - slave->BuffRxIndex) : Size;
        for(uint8_t i = 0; i < copy_size; i++) {
            slave->BuffRx[slave->BuffRxIndex++] = slave->BuffRxRaw[i];
        }
        
        slave->LastTickRx = HAL_GetTick();
        slave->BuffRxSize = slave->BuffRxIndex; 
        slave->FlagRx = 1U;
        
        // Activity LED Toggle
        HAL_GPIO_TogglePin(Led_W_GPIO_Port, Led_W_Pin);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    Modbus_Slave_Serial *slave = Modbus_FindSlave(huart);
    if (slave != NULL) {
        modbus_diag.uart_error_count++;
        modbus_diag.last_port = Modbus_Diag_PortId(huart);
        modbus_diag.last_status = 4;
        system_debug.brake_cmd_dbg++; // Reusing as general error count
        // Try to clear errors and restart DMA
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        Modbus_RestartRx(slave);
    }
}
#endif

void Modbus_init(){
    Modbus_ClearDiagState();
    Modbus_RestartRx(&Port1_485);
    Modbus_RestartRx(&Port2_485);
    Modbus_RestartRx(&UsbCom);
}

void Modbus_Service(void) {
    Modbus_Slave_Serial *slaves[] = { &Port1_485, &Port2_485, &UsbCom };

    for (uint32_t i = 0; i < (sizeof(slaves) / sizeof(slaves[0])); i++) {
        Modbus_Slave_Serial *slave = slaves[i];

        if (slave == NULL || slave->FlagRx == 0U) {
            continue;
        }

        slave->FlagRx = 0U;
        Modbus_Detach_Frame(slave);
        Modbus_RestartRx(slave);
    }
}

void Modbus_CheckHealth(void) {
    if (Port2_485.FlagRx != 0U || huart4.hdmarx == NULL) {
        return;
    }

    // Keep UART4 armed for the next master request.
    if (huart4.RxState != HAL_UART_STATE_BUSY_RX || huart4.hdmarx->State != HAL_DMA_STATE_BUSY) {
        Modbus_RestartRx(&Port2_485);
    }
}

static void Modbus_UART_Send(Modbus_Slave_Serial* Slave) {
    if (Slave->huart->Instance == USART3) {
        HAL_GPIO_WritePin(DIR3_UART3_GPIO_Port, DIR3_UART3_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(Slave->huart, Slave->BuffTx, Slave->BuffTxSize, 50);
        HAL_GPIO_WritePin(DIR3_UART3_GPIO_Port, DIR3_UART3_Pin, GPIO_PIN_RESET);
    } else if (Slave->huart->Instance == UART4) {
        HAL_UART_Transmit(Slave->huart, Slave->BuffTx, Slave->BuffTxSize, 50);
    } else {
        // UART7 (USB)
        HAL_UART_Transmit(Slave->huart, Slave->BuffTx, Slave->BuffTxSize, 50);
    }
    modbus_diag.tx_response_count++;
    modbus_diag.last_port = Modbus_Diag_PortId(Slave->huart);
    modbus_diag.last_tx_size = Slave->BuffTxSize;
    modbus_diag.last_status = 5;
    system_debug.rs485_tx_count++; // Count actual responses sent
}

/* ═══════════════════════════════════════════════════════════════════
 *  RESPONSE HANDLERS
 * ═══════════════════════════════════════════════════════════════════ */
void Modbus_Resspon_Read_Holding_REG(Modbus_Slave_Serial* Slave, uint8_t *data){
    Slave->BuffTxSize = 5 + Slave->Quantity * 2;
    Slave->BuffTx[0] = Slave->BuffRx[0];
    Slave->BuffTx[1] = Slave->BuffRx[1];
    Slave->BuffTx[2] = Slave->Quantity * 2;
    for(uint8_t i = 0; i < Slave->Quantity * 2; i++) Slave->BuffTx[3 + i] = data[i];
    uint16_t crc = Modbus_CRC16(Slave->BuffTx, Slave->BuffTxSize - 2);
    Slave->BuffTx[Slave->BuffTxSize - 2] = crc & 0xFF;
    Slave->BuffTx[Slave->BuffTxSize - 1] = (crc >> 8) & 0xFF;
    Modbus_UART_Send(Slave);
}

void Modbus_Response_Write_Single_REG(Modbus_Slave_Serial* Slave) {
    Slave->BuffTxSize = 8;
    for (uint8_t i = 0; i < 6; i++) Slave->BuffTx[i] = Slave->BuffRx[i];
    uint16_t crc = Modbus_CRC16(Slave->BuffTx, 6);
    Slave->BuffTx[6] = crc & 0xFF;
    Slave->BuffTx[7] = (crc >> 8) & 0xFF;
    Modbus_UART_Send(Slave);
}

void Modbus_Resspon_Write_Multiple_REG(Modbus_Slave_Serial* Slave){
    Slave->BuffTxSize = 8;
    for(uint8_t i = 0; i < 6; i++) Slave->BuffTx[i] = Slave->BuffRx[i];
    uint16_t crc = Modbus_CRC16(Slave->BuffTx, 6);
    Slave->BuffTx[6] = crc & 0xFF;
    Slave->BuffTx[7] = (crc >> 8) & 0xFF;
    Modbus_UART_Send(Slave);
}

/* ═══════════════════════════════════════════════════════════════════
 *  PACKET DECODER
 * ═══════════════════════════════════════════════════════════════════ */
void Modbus_Detach_Frame(Modbus_Slave_Serial* Slave){
    if (Slave->BuffRxSize < 4) return;
    
    // SCANNER: Search for the first byte that matches our Slave ID (or Broadcast 0x00)
    uint16_t start_idx = 0;
    while (start_idx < (Slave->BuffRxSize - 3)) {
        if (Slave->BuffRx[start_idx] == Slave->SlaveID || Slave->BuffRx[start_idx] == 0x00) {
            break;
        }
        start_idx++;
    }
    
    uint8_t *pFrame = &Slave->BuffRx[start_idx];
    uint16_t frameLen = Slave->BuffRxSize - start_idx;
    
    if (frameLen < 4) return;

    // Determine expected length BEFORE CRC check
    uint8_t expected_len = 0;
    uint8_t fc = pFrame[1];
    if (fc == 0x01 || fc == 0x02 || fc == 0x03 || fc == 0x04 || fc == 0x05 || fc == 0x06) {
        expected_len = 8;
    } else if (fc == 0x0F || fc == 0x10) {
        expected_len = (frameLen > 6) ? (9 + pFrame[6]) : 0;
    } else {
        // Unknown FC, drop frame
        modbus_diag.last_slave_id = pFrame[0];
        modbus_diag.last_function = fc;
        modbus_diag.last_status = 6;
        Slave->BuffRxIndex = 0;
        system_debug.brake_cmd_dbg++;
        return;
    }
    
    if (expected_len == 0 || frameLen < expected_len) {
        // Fragmented packet, wait for next chunk (keep current BuffRxIndex)
        return;
    }

    // We have at least expected_len bytes. Calculate CRC on EXACTLY expected_len bytes.
    uint16_t excrc = Modbus_CRC16(pFrame, expected_len - 2);
    uint16_t recrc = (pFrame[expected_len - 1] << 8) | pFrame[expected_len - 2];
    
    if (excrc != recrc) {
        modbus_diag.crc_error_count++;
        modbus_diag.last_slave_id = pFrame[0];
        modbus_diag.last_function = fc;
        modbus_diag.last_crc_calc = excrc;
        modbus_diag.last_crc_recv = recrc;
        modbus_diag.last_status = 3;
        system_debug.brake_cmd_dbg++; // CRC Error
        
        // Log the exact bytes that failed CRC to system_debug for deep hardware debugging
        system_debug.brake_limit_dbg = pFrame[0];
        system_debug.brake_phase_dbg = pFrame[1];
        system_debug.brake_test_stage_dbg = pFrame[expected_len - 2]; // CRCLo
        system_debug.brake_release_req = pFrame[expected_len - 1]; // CRCHi
        
        Slave->BuffRxIndex = 0; // Clear buffer and wait for fresh start
        return;
    }

    // CRC OK!
    Slave->FunctionCode = fc;
    system_debug.brake_step_dbg = fc; 
    
    // Shift data to front of BuffRx if it was offset by scanner, so handlers can use fixed indices
    if (start_idx > 0) {
        for(uint16_t i = 0; i < expected_len; i++) Slave->BuffRx[i] = pFrame[i];
    }

    switch(Slave->FunctionCode){
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
            Slave->StartAddr = (Slave->BuffRx[2] << 8) | Slave->BuffRx[3];
            Slave->Quantity  = (Slave->BuffRx[4] << 8) | Slave->BuffRx[5];
            break;
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            Slave->StartAddr = (Slave->BuffRx[2] << 8) | Slave->BuffRx[3];
            Slave->Quantity  = 1;
            break;
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            Slave->StartAddr = (Slave->BuffRx[2] << 8) | Slave->BuffRx[3];
            Slave->Quantity  = (Slave->BuffRx[4] << 8) | Slave->BuffRx[5];
            break;
        default: return;
    }
    modbus_diag.frame_ok_count++;
    modbus_diag.last_slave_id = Slave->BuffRx[0];
    modbus_diag.last_function = Slave->FunctionCode;
    modbus_diag.last_start_addr = Slave->StartAddr;
    modbus_diag.last_quantity = Slave->Quantity;
    modbus_diag.last_crc_calc = excrc;
    modbus_diag.last_crc_recv = recrc;
    modbus_diag.last_status = 2;
    Modbus_Process_Frame(Slave);
}

/* ═══════════════════════════════════════════════════════════════════
 *  PROCESS HANDLER: Link to Register Map
 * ═══════════════════════════════════════════════════════════════════ */
void Modbus_Process_Frame(Modbus_Slave_Serial* Slave) {
    // Validate Slave ID
    if (Slave->BuffRx[0] != Slave->SlaveID && Slave->BuffRx[0] != 0x00) {
        // Not for us, but maybe we can log this mismatch if needed
        // system_debug.brake_debug++; // Already incremented in RxEvent
        return;
    }
    
    // Toggle LED on valid Slave ID frame
    HAL_GPIO_TogglePin(Led_W_GPIO_Port, Led_W_Pin);
    
    switch (Slave->FunctionCode) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS: {
            uint16_t qty = Slave->Quantity;
            if (qty > 50) qty = 50; // Safety cap
            uint8_t dat[qty * 2];
            for (uint16_t i = 0; i < qty; i++) {
                uint16_t val = ModbusRegMap_Read(Slave->StartAddr + i);
                dat[i * 2]     = (val >> 8) & 0xFF;
                dat[i * 2 + 1] = val & 0xFF;
            }
            Modbus_Resspon_Read_Holding_REG(Slave, dat);
            break;
        }
        case MODBUS_FC_WRITE_SINGLE_REGISTER: {
            uint16_t val = (Slave->BuffRx[4] << 8) | Slave->BuffRx[5];
            ModbusRegMap_Write(Slave->StartAddr, val);
            Modbus_Response_Write_Single_REG(Slave);
            if (Slave->StartAddr == REG_CMD_DIAG_RESET && val == 1) {
                Modbus_ClearDiagState();
            }
            break;
        }
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
            uint8_t start = 7;
            for (uint16_t i = 0; i < Slave->Quantity; i++) {
                uint16_t val = (Slave->BuffRx[start+i*2] << 8) | Slave->BuffRx[start+i*2+1];
                ModbusRegMap_Write(Slave->StartAddr + i, val);
            }
            Modbus_Resspon_Write_Multiple_REG(Slave);
            if (modbus_regs.Array[REG_CMD_DIAG_RESET] == 1) {
                Modbus_ClearDiagState();
            }
            break;
        }
        default: break;
    }
    Slave->BuffRxIndex = 0; // Clear buffer after successful processing
    Slave->BuffRxSize = 0;
}
