#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "../fep.h"

#define DEBUG_TX
//#define DEBUG_RX

typedef struct _packet{
	unsigned char up;
	unsigned char point;
	unsigned char shot;
	unsigned char manual;//新規追加
	signed short drivingA;
	signed short drivingB;
}packet;

int main(void) {
    #ifdef DEBUG_TX
    //char buf[5] = {1, 2, 3, 4, 5};
    packet buf = {0};
    #endif

    #ifdef DEBUG_RX
    char buf[20] = {0};
    #endif

    int i = 0;

    #ifdef DEBUG_TX
    DDRA = 0x00;
    PORTA = 0x01;
    #endif
    #ifdef DEBUG_RX
    DDRA = 0xFF;
    #endif

    sei();

    #ifdef DEBUG_TX
    FEP_init(0, 1, 26, 40, 58, 0x1234);
    #endif
    #ifdef DEBUG_RX
    FEP_init(0, 2, 26, 40, 58, 0x1234);
    #endif

    while (1) {
        #ifdef DEBUG_TX
		buf.drivingA = -0xF0;
		buf.drivingB = -0xF0;
        FEP_putbin((uint8_t *)&buf, sizeof(buf), 2);
        //FEP_puts("hello", 2);
        //_delay_ms(100);
        #endif
        #ifdef DEBUG_RX
        if (FEP_available() > 0) {
            FEP_gets(buf, sizeof(buf));
            PORTA = buf[0];
            //fprintf(&fepio, "%x %x %x %x %x\r", buf[0], buf[1], buf[2], buf[3], buf[4]);
            //fprintf(&fepio, "%s\r", buf);
        }
        #endif
        _delay_ms(20);
    }

    return 0;
}
