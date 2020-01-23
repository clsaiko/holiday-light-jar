/**********************************
File:   ws2812_asmLib.h
Author: csaiko

Created: Dec 4, 2019
**********************************/

#ifndef WS2812_ASMLIB_H
#define	WS2812_ASMLIB_H

#ifdef	__cplusplus
extern "C" {
#endif

    void wait_50us(void);
    void write_0(void);
    void write_1(void);
    void clock_test(void);

#ifdef	__cplusplus
}
#endif

#endif	// WS2812_ASMLIB_H

