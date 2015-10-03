#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "avr-uart/uart.h"
#include "fep.h"

/*
 * Macros and constants
 */
#define FEP_LINE_LEN 150
#define FEP_TIMEOUT_MS 4000
#define FEP_RETRY 10

/*
 * private function prototypes
 */
/******************************************************************************
Function: FEP_getFrq1()
Purpose:  get single frequency band setting
Params:   channel number you want to read
Return:   band number
******************************************************************************/
uint8_t FEP_getFrq1(uint8_t ch);

/******************************************************************************
Function: FEP_setFrq1()
Purpose:  set single frequency band
Params:   ch - channel number
          band - band
Return:   response from FEP
******************************************************************************/
uint8_t FEP_setFrq1(uint8_t ch, uint8_t band);

/******************************************************************************
Function: FEP_waitResponse()
Purpose:  Loop until receive response from FEP
Params:   none
Return:   response from FEP
******************************************************************************/
static uint8_t FEP_waitResponse(void);

/******************************************************************************
Function: FEP_waitResponseStr()
Purpose:  Loop until receive response from FEP and store response string to buffer
Params:   buf - buffer for storing the response from FEP
          len - size of response you want to read(< size of buf - 3).
Return:   number of character that has been stored
******************************************************************************/
static char *FEP_waitResponseStr(char *buf, size_t len);

/******************************************************************************
Function: FEP_io_getchar()
Purpose:  get a character from UART
Params:   stream
Return:   received character from UART
******************************************************************************/
int FEP_io_getchar(FILE *stream);

/******************************************************************************
Function: FEP_io_putchar()
Purpose:  send a character via UART
Params:   c - character to be sent
          stream
Return:
******************************************************************************/
int FEP_io_putchar(char c, FILE *stream);

/*
 *  Module global variables
 */
static volatile int16_t FEP_intensity;
static volatile uint8_t FEP_busyFlag;
static volatile uint8_t FEP_lastRxData;
static volatile uint8_t FEP_availablePacket;
static volatile uint8_t FEP_currentByte;
static volatile uint8_t FEP_prevByte;
static volatile uint16_t FEP_discardableByte;
void (*FEP_uart_init)(uint16_t baudrate);
void (*FEP_uart_setRxHandler)(void (*func)(uint8_t data, uint8_t lastRxError));
uint16_t (*FEP_uart_getc)(void);
uint16_t (*FEP_uart_peek)(void);
void (*FEP_uart_putc)(uint8_t data);
uint16_t (*FEP_uart_available)(void);
void (*FEP_uart_flush)(uint16_t n);

FILE fepio = FDEV_SETUP_STREAM(FEP_io_putchar, FEP_io_getchar, _FDEV_SETUP_RW);

/*
 * functions
 */
void FEP_init(
    uint8_t module,
    uint8_t addr,
    uint8_t ch1,
    uint8_t ch2,
    uint8_t ch3,
    uint16_t id)
{
    /* initialize global variables */
    FEP_intensity = 0;
    FEP_busyFlag = 0;
    FEP_availablePacket = 0;
    FEP_currentByte = 0;
    FEP_prevByte = 0;
    FEP_discardableByte = 0;

    /* set uart functions */
    switch (module) {
#if defined( USART0_ENABLED )
        case 0:
            FEP_uart_init = &uart0_init;
            FEP_uart_setRxHandler = &uart0_setRxHandler;
            FEP_uart_getc = &uart0_getc;
            FEP_uart_peek = &uart0_peek;
            FEP_uart_putc = &uart0_putc;
            FEP_uart_available = &uart0_available;
            FEP_uart_flush = &uart0_flush;
            break;
#endif
#if defined( USART1_ENABLED )
        case 1:
            FEP_uart_init = &uart1_init;
            FEP_uart_setRxHandler = &uart1_setRxHandler;
            FEP_uart_getc = &uart1_getc;
            FEP_uart_peek = &uart1_peek;
            FEP_uart_putc = &uart1_putc;
            FEP_uart_available = &uart1_available;
            FEP_uart_flush = &uart1_flush;
            break;
#endif
#if defined( USART2_ENABLED )
        case 2:
            FEP_uart_init = &uart2_init;
            FEP_uart_setRxHandler = &uart2_setRxHandler;
            FEP_uart_getc = &uart2_getc;
            FEP_uart_peek = &uart2_peek;
            FEP_uart_putc = &uart2_putc;
            FEP_uart_available = &uart2_available;
            FEP_uart_flush = &uart2_flush;
            break;
#endif
#if defined( USART3_ENABLED )
        case 3:
            FEP_uart_init = &uart3_init;
            FEP_uart_setRxHandler = &uart3_setRxHandler;
            FEP_uart_getc = &uart3_getc;
            FEP_uart_peek = &uart3_peek;
            FEP_uart_putc = &uart3_putc;
            FEP_uart_available = &uart3_available;
            FEP_uart_flush = &uart3_flush;
            break;
#endif
        default:
            /* error! The module # is wrong! */
            return;
    }

    /* Initialize uart module */
    (*FEP_uart_init)(UART_BAUD_SELECT(38400, F_CPU));

    /* Set additional rx interrupt handler */
    (*FEP_uart_setRxHandler)(FEP_rxHandler);

    /* Add intensity to response */
    FEP_setReg(13, (1 << 7));

    FEP_setMyAddr(addr);
    FEP_setFrq(ch1, ch2, ch3);
    FEP_setID(id);
}

uint8_t FEP_puts(char *str, uint8_t addr) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@TXT%03d%s\r\n"), addr, str);

        response = FEP_waitResponse();
        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

uint8_t FEP_putbin(char *ary, size_t len, uint8_t addr) {
    uint8_t response, i, j;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@TBN%03d%03d"), addr, len);
        for (j = 0; j < len; j++) {
            fputc(*(ary + j), &fepio);
        }
        fputc('\r', &fepio);
        fputc('\n', &fepio);

        response = FEP_waitResponse();
        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

uint8_t FEP_gets(char *str, size_t len) {
    uint16_t intensity, i;
    uint8_t transmitter_addr, data_mode;
    size_t data_len;
    char buf[FEP_LINE_LEN], sub[FEP_LINE_LEN];

    /* skip over old datas */
    FEP_flushAVR(FEP_discardableByte);
    fgets(buf, FEP_LINE_LEN, &fepio);
    FEP_availablePacket = 0;

    /* Parse command */
    if (strlen(buf) >= 3) {
        /* get command part */
        strncpy(sub, buf, 3);
        sub[3] = '\0';
        if (!strcmp(sub, "RXT")) {
            /* Subtract string from a part before the intensity  */
            strncpy(sub, buf, strlen(buf) - 3 - 2);
            sub[strlen(buf) - 3 - 2 + 1] = '\0';

            /* Get transmitter's address */
            sscanf_P(sub, PSTR("RXT%03d"), &transmitter_addr);
            /* Store message to str */
            strncpy(str, buf + 6, strlen(buf) - 6 - 3 - 2);

            /* Get intensity part of string */
            strncpy(sub, buf + strlen(buf) - 3 - 2, 3);
            sub[3] = '\0';

            data_mode = FEP_DT_STR;
        } else if (!strcmp(sub, "RBN")) {
            /* addr and length part */
            strncpy(sub, buf + 3, 6);
            sub[6] = '\0';

            /* Get transmitter's address & data length */
            sscanf_P(sub, PSTR("%03d%03d"), &transmitter_addr, &data_len);
            /* Store binary data to array */
            if (data_len > len) {
                /* received data is too long! */
                return 0;
            }
            /* memcpy(str, buf + 9, data_len); */
            for (i = 0; i < data_len; i++) {
                str[i] = buf[i + 9];
            }

            /* Get intensity part of string */
            strncpy(sub, buf + 9 + data_len, 3);
            sub[3] = '\0';

            data_mode = FEP_DT_BIN;
        }

        /* Import intensity */
        sscanf_P(sub, PSTR("%03d"), &intensity);
        FEP_intensity = intensity;
    }

    return data_mode;
}

uint8_t FEP_flushFEP(void) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@BCL\r\n"));

        response = FEP_waitResponse();

        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

void FEP_flushAVR(uint16_t n) {
    (*FEP_uart_flush)(n);
}

uint8_t FEP_getReg(uint8_t reg_num) {
    char buf[6];
    uint8_t val;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    fprintf_P(&fepio, PSTR("@REG%02d\r\n"), reg_num);

    FEP_waitResponseStr(buf, 3);

    sscanf_P(buf, PSTR("%XH"), &val);

    FEP_busyFlag = 0;

    return val;
}

uint8_t FEP_setReg(uint8_t reg_num, uint8_t val) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@REG%02d:%03d\r\n"), reg_num, val);

        response = FEP_waitResponse();

        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    FEP_reset();
    return response;
}

int16_t FEP_getIntensity(void) {
    return FEP_intensity;
}

uint8_t FEP_getMyAddr(void) {
    return FEP_getReg(0);
}

uint8_t FEP_setMyAddr(uint8_t addr) {
    return FEP_setReg(0, addr);
}

void FEP_getFrq(uint8_t *ch1, uint8_t *ch2, uint8_t *ch3) {
    *ch1 = FEP_getFrq1(1);
    *ch2 = FEP_getFrq1(2);
    *ch3 = FEP_getFrq1(3);
}

uint8_t FEP_getFrq1(uint8_t ch) {
    char buf[5];
    uint8_t band;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    fprintf_P(&fepio, PSTR("@FRQ%d\r\n"), ch);

    FEP_waitResponseStr(buf, 2);

    sscanf_P(buf, PSTR("%d"), &band);

    FEP_busyFlag = 0;

    return band;
}

uint8_t FEP_setFrq(uint8_t ch1, uint8_t ch2, uint8_t ch3) {
    if (FEP_setFrq1(1, ch1) == FEP_N0) return FEP_N0;
    if (FEP_setFrq1(2, ch2) == FEP_N0) return FEP_N0;
    if (FEP_setFrq1(3, ch3) == FEP_N0) return FEP_N0;

    return FEP_P0;
}

uint8_t FEP_setFrq1(uint8_t ch, uint8_t band) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@FRQ%1d:%02d\r\n"), ch, band);

        response = FEP_waitResponse();

        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

uint16_t FEP_getID(void) {
    char buf[8];
    uint16_t id;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    fprintf_P(&fepio, PSTR("@IDR\r\n"));

    FEP_waitResponseStr(buf, 5);

    sscanf_P(buf, PSTR("%XH"), &id);

    FEP_busyFlag = 0;

    return id;
}

uint8_t FEP_setID(uint16_t id) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@IDW%4XH\r\n"), id);

        response = FEP_waitResponse();

        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

uint8_t FEP_reset(void) {
    uint8_t response, i;

    FEP_busyFlag = 1;
    FEP_flushAVR(0);
    for (i = 0; i < FEP_RETRY; i++) {
        fprintf_P(&fepio, PSTR("@RST\r\n"));

        response = FEP_waitResponse();

        if (response == FEP_P0) break;
    }
    FEP_busyFlag = 0;

    return response;
}

uint8_t FEP_waitResponse(void) {
    uint16_t i;
    char buf[16];

    for(i = 0; i < FEP_TIMEOUT_MS; i++) {
        if ((*FEP_uart_available)() >= 4) {
            /* get response */
            fgets(buf, sizeof(buf), &fepio);

            if (strcmp(buf, "P1\r\n") == 0) {
                return FEP_waitResponse();
            } else {
                if (strcmp(buf, "P0\r\n") == 0) {
                    return FEP_P0;
                } else if (strcmp(buf, "N0\r\n") == 0) {
                    return FEP_N0;
                } else if (strcmp(buf, "N1\r\n") == 0) {
                    return FEP_N1;
                } else if (strcmp(buf, "N2\r\n") == 0) {
                    return FEP_N2;
                } else if (strcmp(buf, "N3\r\n") == 0) {
                    return FEP_N3;
                } else {
                    continue;
                }
            }
        } else {
            _delay_ms(1);
        }
    }
    return FEP_NO_RESPONSE;
}

char *FEP_waitResponseStr(char *buf, size_t len) {
    size_t i;

    for(i = 0; i < FEP_TIMEOUT_MS && (*FEP_uart_available)() < len; i++) {
        _delay_ms(1);
    }

    return fgets(buf, len + 2 + 1, &fepio);
}

int FEP_io_getchar(FILE *stream) {
    uint16_t status, data;

    while ((*FEP_uart_available)() <= 0) ;

    data = (*FEP_uart_getc)();
    status = data >> 8;
    data = (data & 0xff);

    return data;
}

int FEP_io_putchar(char c, FILE *stream) {
    (*FEP_uart_putc)(c);
    return 0;
}

uint16_t FEP_available(void) {
	return FEP_availablePacket;
}

void FEP_rxHandler(uint8_t data, uint8_t error) {
    FEP_currentByte++;

    if (!FEP_busyFlag && FEP_lastRxData == '\r' && data == '\n') {
        FEP_discardableByte += FEP_prevByte;
        FEP_prevByte = FEP_currentByte;
        FEP_currentByte = 0;
        FEP_availablePacket++;
    }

    FEP_lastRxData = data;

    return;
}
