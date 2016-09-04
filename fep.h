#ifndef _FEP_H
#define _FEP_H

#include <stdio.h>
#include "avr-uart/uart.h"

/*
** FEP responses
*/
#define FEP_NO_RESPONSE  (0x10) /* レスポンスが得られなかった */
#define FEP_P0  (0x11)  /* 正常終了 */
#define FEP_P1  (0x12)  /* コマンド受理、データ送信中 */
#define FEP_N0  (0x13)  /* コマンドエラー */
#define FEP_N1  (0x14)  /* データ送信失敗(宛先の無線モデムの応答なし、キャリアセンスで送信出来なかった)  */
#define FEP_N2  (0x15)  /* 割り当てなし */
#define FEP_N3  (0x16)  /* データ送信失敗(宛先の無線モデムのバッファがフルで受信できない) */

#define FEP_DT_ERR 0
#define FEP_DT_STR 1
#define FEP_DT_BIN 2

/*
 * global variables
 */
extern FILE fepio;

/*
** function prototypes
*/

/******************************************************************************
Function: FEP_init()
Purpose:  Initializing UART and FEP.
Params:   module - UART module's number (0~3)
          addr - address of FEP
          ch1 - channel1 band of FEP
          ch2 - channel2 band of FEP
          ch3 - channel3 band of FEP
          id - id of FEP
Return:   none
******************************************************************************/
void FEP_init(
    uint8_t module,
    uint8_t addr,
    uint8_t ch1,
    uint8_t ch2,
    uint8_t ch3,
    uint16_t id
    );

/******************************************************************************
Function: FEP_puts()
Purpose:  Sending string.
Params:   str - string for sending
          addr - receiver's address
Return:   response from FEP
******************************************************************************/
uint8_t FEP_puts(char *str, uint8_t addr);

/******************************************************************************
Function: FEP_putbin()
Purpose:  Sending binary array.
Params:   ary - head address of array
          len - size of array
          addr - receiver's address
Return:   response from FEP
******************************************************************************/
uint8_t FEP_putbin(char *ary, size_t len, uint8_t addr);

/******************************************************************************
Function: FEP_gets()
Purpose:  get string or binary data from transmitter.
Params:   str - buffer for storing string
Return:   If data is string, return constant FEP_DT_STR.
          If data is binary, return constant FEP_DT_BIN.
******************************************************************************/
uint8_t FEP_gets(char *str, size_t len);

/******************************************************************************
Function: FEP_getTransmitterAddr()
Purpose:  get the address of transmitter. You can call this function
          after you call FEP_gets().
Params:   none
Return:   Address of transmitter
******************************************************************************/
uint8_t FEP_getTransmitterAddr(void);

/******************************************************************************
Function: FEP_flushFEP()
Purpose:  flush buffer of FEP
Params:   none
Return:   response from FEP
******************************************************************************/
uint8_t FEP_flushFEP(void);

/******************************************************************************
Function: FEP_getReg()
Purpose:  get register value
Params:   reg_num - register number
Return:   register value
******************************************************************************/
uint8_t FEP_getReg(uint8_t reg_num);

/******************************************************************************
Function: FEP_setReg()
Purpose:  set register value
Params:   reg_num - register number
          val - value to set
Return:   response from FEP
******************************************************************************/
uint8_t FEP_setReg(uint8_t reg_num, uint8_t val);

/******************************************************************************
Function: FEP_getIntensity()
Purpose:  get electric field intensity
Params:   none
Return:   intensity
******************************************************************************/
int16_t FEP_getIntensity(void);

/******************************************************************************
Function: FEP_getMyAddr()
Purpose:  get own address of FEP
Params:   none
Return:   addres or response from FEP(If the instruction fails)
******************************************************************************/
uint8_t FEP_getMyAddr(void);

/******************************************************************************
Function: FEP_setMyAddr()
Purpose:  set own address to FEP
Params:   addr - address
Return:   response from FEP
******************************************************************************/
uint8_t FEP_setMyAddr(uint8_t addr);

/******************************************************************************
Function: FEP_getFrq()
Purpose:  get frequency band setting
Params:   ch1 - variable for storing channel1 value
          ch2 - variable for storing channel2 value
          ch3 - variable for storing channel3 value
Return:   none
******************************************************************************/
void FEP_getFrq(uint8_t *ch1, uint8_t *ch2, uint8_t *ch3);

/******************************************************************************
Function: FEP_setFrq()
Purpose:  set frequency band
Params:   ch1 - channel1 value
          ch2 - channel2 value (In the single band mode, this value is ignored)
          ch3 - channel3 value (In the single or double band mode, this value is ignored)
Return:   response from FEP
******************************************************************************/
uint8_t FEP_setFrq(uint8_t ch1, uint8_t ch2, uint8_t ch3);

/******************************************************************************
Function: FEP_getID()
Purpose:  get ID code
Params:   none
Return:   ID or response from FEP(If the instruction fails)
******************************************************************************/
uint16_t FEP_getID(void);

/******************************************************************************
Function: FEP_setID()
Purpose:  set ID code
Params:   id - id value
Return:   response from FEP
******************************************************************************/
uint8_t FEP_setID(uint16_t id);

/******************************************************************************
Function: FEP_reset()
Purpose:  reset FEP.(Activate register setting.)
Params:   none
Return:   response from FEP
******************************************************************************/
uint8_t FEP_reset(void);

/******************************************************************************
Function: FEP_available()
Purpose:  Determine if a line waiting in the receive buffer or not
Params:   none
Return:   When available: 1
          When unavailable: 0
******************************************************************************/
uint16_t FEP_available(void);

/******************************************************************************
Function: FEP_rxHandler()
Purpose:  Called in uart receive interrupt.
Params:   data - newest data from uart
          error - error of uart
Return:   none
******************************************************************************/
void FEP_rxHandler(uint8_t data, uint8_t error);

#endif /* _FEP_H */
