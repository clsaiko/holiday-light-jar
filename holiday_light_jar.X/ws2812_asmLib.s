.equ __P24FJ64GA002,1
.include "p24Fxxxx.inc"

#include "xc.inc" ;BP

.text
.global _clock_test, _write_0, _write_1, _wait_50us


_clock_test:
    btg LATA, #0    ;toggle RA0, one cycle
    bra _clock_test ;repeat

_write_0:	    ; 0.35us high, 0.8us low, 1.25us total
    inc	    LATA    ; set pin RA0 high = 1 cycle
    repeat  #3	    ; 1 cycle to load and prep
    nop		    ; 3+1 cycles to execute NOP 4 times
		    ; 6 cycles total (0.375us)
		    
    clr	    LATA    ; set pin RA0 low = 1 cycle
    repeat  #5	    ; 1 cycle to load and prep
    nop		    ; 5+1 cycles to execute NOP 6 times
    return	    ; 3 cycles for return
		    ; 2 cycles for function call
		    ; 13 cycles total (0.8125us)
    nop		    ; extra 1 cycles (0.0625us)
		    
_write_1:	    ; 0.7us high, 0.6us low, 1.25us total
    inc	    LATA    ; set pin RA0 high = 1 cycle
    repeat  #8	    ; 1 cycle to load and prep
    nop		    ; 8+1 cycles to execute NOP 9 times
		    ; 11 cycles total (0.6875us)
		    
    clr	    LATA    ; set pin RA0 low = 1 cycle
    repeat  #1	    ; 1 cycle to load and prep
    nop		    ; 1+1 cycles to execute NOP 2 times
    return	    ; 3 cycles for return
		    ; 2 cycles for function call
		    ; 9 cycles total (0.5625us)
	
_wait_50us:	    ; >50us low
    clr LATA	    ; set pin RA0 low = 1 cycle
    repeat  #800    ; 1 cycle to load and prep
    nop		    ; 800+1 cycles to execute NOP 801 times
    return	    ; 3 cycles for return
		    ; 806 cycles total (50.375us)    

	