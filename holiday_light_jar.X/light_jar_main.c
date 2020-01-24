/**********************************
File:   light_jar_main.c
Author: csaiko

Created: Dec 4, 2019
**********************************/


#include "xc.h"
#include "ws2812_asmLib.h"
#include <stdbool.h>
#include <p24Fxxxx.h>

// CW1: FLASH CONFIG WORD 1 (see PIC24 Family Ref Manual 24.1)
#pragma config ICS = PGx1       // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF     // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF       // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF        // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF     // JTAG Port Enable (JTAG port is disabled)

// CW2: FLASH CONFIG WORD 2 (see PIC24 Family Ref Manual 24.1)
#pragma config I2C1SEL = PRI    // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF    // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = OFF   // Primary Oscillator I/O Function (CLK0/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME   // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL   // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))


//function prototypes KNOWN NEEDED
void setup(void);
void delay(int time);
uint16_t lfsr(uint16_t start);
void writeColor(int r, int g, int b);
unsigned long int packColor(int red, int blu, int grn);
int getR(unsigned long int RGBval);
int getG(unsigned long int RGBval);
int getB(unsigned long int RGBval);
void writePacCol(unsigned long int PackedColor);

//function prototypes UNKNOWN NEEDED
void drawFrame(int frame);
void primFrame(int frame,int color);
unsigned long int Wheel(unsigned char WheelPos);
void cycler1(void);
void cycler2(void);
void wheeldemo(void);

//global variables
volatile int themeNumber = 0;
volatile int themeMode = 0;
volatile int minLEDs = 5;   //minimum number of LEDs that must be lit
volatile int led;

// Contains the valid colors available for the user's chosen theme. The first
//  integer contains the number of theme colors, the rest are the abstract packColor
//  format as the colors.
//  Default theme: Winter - White, Gray, Light Blue
volatile unsigned long int colorList[] = {0x3, 0x00ffffff, 0x00666666, 0x0066ffff, 0x0, 0x0, 0x0};

// The lastColorState array contains the last color state of the LEDs.  Similar
//  to the targetColorState, though it is trivial when the LEDs are simply
//  blinking to a new color, it will be important to know the starting state
//  the when executing a smooth gradient between colors. Defaults to all off.
volatile unsigned long int lastColorState[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// The currentColorState array contains the current or soon to be displayed
//  colors of the LEDs. It will be more useful when gradients are executed.
unsigned long int currentColorState[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// The targetColorState array contains "packedColor" 32 bit integers, which
//  contain the RGB values for each of the six LEDs. It is set when new colors
//  are selected at random to populate the LEDs. Although it is trivial when
//  the LEDs are simply blinking to a new color, it will be important to know
//  the end result wanted when executing a smooth gradient between colors.
unsigned long int targetColorState[] = {0x00ff0000, 0x0000ff00, 0x000000ff, 0x00ff00ff, 0x0000ffff, 0x00ffff00};

// INT0 Capture Interrupt Service Routine
//  Precondition:   A rising edge on the PIC INT0 pin has been received.
//  Postcondition:
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(void){

    delay(50);    //simple 50 ms wait for de-bouncing

    themeNumber++;  //increment to the next themeNumber
    //bounds check
    if (themeNumber > 13) themeNumber = 0;

    //Winter - White, Gray, Light Blue, White, Gray, Light Blue
    if (themeNumber == 0){
        colorList[0] = 0x6;
        colorList[1] = 0x00ffffff;
        colorList[2] = 0x00666666;
        colorList[3] = 0x0066ffff;
        colorList[4] = 0x00ffffff;
        colorList[5] = 0x00666666;
        colorList[6] = 0x0066ffff;
        minLEDs = 5;
    }

    //Christmas - Red, Green, White, Red, Green, White
    else if (themeNumber == 1){
        colorList[0] = 0x6;
        colorList[1] = 0x00ff0000;
        colorList[2] = 0x0000ff00;
        colorList[3] = 0x00ffffff;
        colorList[4] = 0x00ff0000;
        colorList[5] = 0x0000ff00;
        colorList[6] = 0x00ffffff;
        minLEDs = 5;
    }

    //New Years' - Purple, Gold, White, Purple, Gold, White
    else if (themeNumber == 2){
        colorList[0] = 0x6;
        colorList[1] = 0x00cc00cc;
        colorList[2] = 0x00ffff33;
        colorList[3] = 0x00ffffff;
        colorList[4] = 0x00cc00cc;
        colorList[5] = 0x00ffff33;
        colorList[6] = 0x00ffffff;
        minLEDs = 5;
    }

    //Valentines - Red, Pink, White, Red, Pink, White
    else if (themeNumber == 3){
        colorList[0] = 0x6;
        colorList[1] = 0x00ff0000;
        colorList[2] = 0x00ff007f;
        colorList[3] = 0x00ffffff;
        colorList[4] = 0x00ff0000;
        colorList[5] = 0x00ff007f;
        colorList[6] = 0x00ffffff;
        minLEDs = 5;
    }

    //Easter - Light Green, Light Pink, Light Yellow , Light Blue
    else if (themeNumber == 4){
        colorList[0] = 0x4;
        colorList[1] = 0x0066ff66;
        colorList[2] = 0x00ff66b2;
        colorList[3] = 0x00ffff66;
        colorList[4] = 0x0066ffff;
        colorList[5] = 0x0;
        colorList[6] = 0x0;
        minLEDs = 5;
    }

    //St. Patrick's Day - White, Green, White, Green, White, Green
    else if (themeNumber == 5){
        colorList[0] = 0x6;
        colorList[1] = 0x00ffffff;
        colorList[2] = 0x0000ff00;
        colorList[3] = 0x00ffffff;
        colorList[4] = 0x0000ff00;
        colorList[5] = 0x00ffffff;
        colorList[6] = 0x0000ff00;
        minLEDs = 5;
    }

    //Spring - Light Green, Yellow, Light Green, Yellow, Light Green, Yellow
    else if (themeNumber == 6){
        colorList[0] = 0x6;
        colorList[1] = 0x0066ff66;
        colorList[2] = 0x00ffff00;
        colorList[3] = 0x0066ff66;
        colorList[4] = 0x00ffff00;
        colorList[5] = 0x0066ff66;
        colorList[6] = 0x00ffff00;
        minLEDs = 5;
    }

    //Summer - Dark Green, Yellow, Dark Green, Yellow, Dark Green, Yellow
    else if (themeNumber == 7){
        colorList[0] = 0x6;
        colorList[1] = 0x00006600;
        colorList[2] = 0x00ffff00;
        colorList[3] = 0x00006600;
        colorList[4] = 0x00ffff00;
        colorList[5] = 0x00006600;
        colorList[6] = 0x00ffff00;
        minLEDs = 5;
    }

    //USA - Red, White, Blue, Red, White, Blue
    else if (themeNumber == 8){
        colorList[0] = 0x6;
        colorList[1] = 0x00ff0000;
        colorList[2] = 0x00ffffff;
        colorList[3] = 0x000000ff;
        colorList[4] = 0x00ff0000;
        colorList[5] = 0x00ffffff;
        colorList[6] = 0x000000ff;
        minLEDs = 5;
    }

    //Fall - Orange, Yellow, Brown
    else if (themeNumber == 9){
        colorList[0] = 0x6;
        colorList[1] = 0x00cc6600;
        colorList[2] = 0x00ffff00;
        colorList[3] = 0x00663300;
        colorList[4] = 0x00cc6600;
        colorList[5] = 0x00ffff00;
        colorList[6] = 0x00663300;
        minLEDs = 5;
    }

    //Halloween - Orange, Gray, Brown, Dark Green
    else if (themeNumber == 10){
        colorList[0] = 0x4;
        colorList[1] = 0x00cc6600;
        colorList[2] = 0x00808080;
        colorList[3] = 0x00331900;
        colorList[4] = 0x00006600;
        colorList[5] = 0x0;
        colorList[6] = 0x0;
        minLEDs = 5;
    }

    //Lantern - All White
    else if (themeNumber == 11){
        colorList[0] = 0x6;
        colorList[1] = 0x00ffffff;
        colorList[2] = 0x00ffffff;
        colorList[3] = 0x00ffffff;
        colorList[4] = 0x00ffffff;
        colorList[5] = 0x00ffffff;
        colorList[6] = 0x00ffffff;
        minLEDs = 6;
    }

    //Lantern - All Yellow
    else if (themeNumber == 12){
        colorList[0] = 0x6;
        colorList[1] = 0x00ffff00;
        colorList[2] = 0x00ffff00;
        colorList[3] = 0x00ffff00;
        colorList[4] = 0x00ffff00;
        colorList[5] = 0x00ffff00;
        colorList[6] = 0x00ffff00;
        minLEDs = 6;
    }

    //Party - Red, Green, Blue, Teal, Magenta, Yellow
    else if (themeNumber == 13){
        colorList[0] = 0x6;
        colorList[1] = 0x00ff0000;
        colorList[2] = 0x0000ff00;
        colorList[3] = 0x000000ff;
        colorList[4] = 0x0000ffff;
        colorList[5] = 0x00ff00ff;
        colorList[6] = 0x00ffff00;
        minLEDs = 6;
    }

    //write the colors to the LEDs
    writePacCol(colorList[1]);
    writePacCol(colorList[2]);
    writePacCol(colorList[3]);
    writePacCol(colorList[4]);
    writePacCol(colorList[5]);
    writePacCol(colorList[6]);
    wait_50us();

    //update color states
    for (led = 0; led < 6; led++){
        lastColorState[led] = colorList[led + 1];
        currentColorState[led] = colorList[led + 1];
        targetColorState[led] = colorList[led + 1];
    }

    delay(50);    //wait for 50ms so the user can see the selection

    _INT0IF = 0;    // reset INT0 interrupt here before exit

}//end INT0 ISR


int main(void) {

    setup();    //setup registers

    // The LEDState array contains the on/off state of the six LEDs, after the
    //  LFSR algorithm to determine their status has run. True and False
    //  correspond to On and Off, respectively. This might potentially be eliminated
    //  for performance if a bit mask on the
    bool LEDState[6];

    int tempRval, tempGval, tempBval;
    int targetRval, targetGval, targetBval;

    uint16_t currentState = 0x00BF;  //must be non-zero for lfsr cycle to work
    uint16_t tempState = 0x0;        //for validating a working state
    uint16_t ADCValue;
    int colorPick = 0;
    int LEDtotal = 0;
    int gradStep, ledNum;
    int i, j, v;  //TODO: change these to be more descriptive

    while(1){   //forever loop

        //set LEDState array values from state
        LEDState[0] = ((currentState & 0x01) == 1);
        LEDState[1] = ((currentState & 0x02) == 2);
        LEDState[2] = ((currentState & 0x04) == 4);
        LEDState[3] = ((currentState & 0x08) == 8);
        LEDState[4] = ((currentState & 0x10) == 16);
        LEDState[5] = ((currentState & 0x20) == 32);


        //set targetColorState array from lastColorState
        for (i = 0; i < 6; i++){

            //This color picker does not have a check to ensure a new color is picked
            //lastColorState could be used to force a color change, but for now it
            // is just random.
            if (LEDState[i]){       //LED is on, need a color
                colorPick = 0;
                //pick random color
                while (colorPick == 0){

                    AD1CON1bits.SAMP = 1;       //start sampling process
                    while(!AD1CON1bits.DONE);   //wait until conversion is done
                    ADCValue = ADC1BUF0;        //get the ADC value

                    //mask off all but least sig three bits of ADC value
                    ADCValue = (ADCValue & 0x07);

                    if ((ADCValue > colorList[0]) || (ADCValue == 0)) {
                        continue;   //ADC value out of bounds, get a new number
                    }

                    //use this hopefully random number to select a random color
                    targetColorState[i] = colorList[ADCValue];

                    //targetColorState[i] = colorList[(i % colorList[0]) + 1];
                    colorPick = 1;

                }//end colorPick while



            }//end if
            else {
                targetColorState[i] = 0x0;    //LED off
                //wait_50us();
            }


        }//end for loop to set targetColorState


//        //BLINKY CODE
//        for (k = 0; k < 6; k++) {
//            currentColorState[k] = targetColorState[k];
//            //if (!LEDState[k]) currentColorState[k] = 0x0;   //turn off
//        }
//
//        //write the colors
//        for (ledNum = 0; ledNum < 6; ledNum++){
//            writePacCol(currentColorState[ledNum]);
//        }
//
//        //data stream written, wait to complete
//        wait_50us();


        //GRADIENT CODE

        //set the current color to what the last color was
        for (j = 0; j < 6; j++){
            currentColorState[j] = lastColorState[j];
        }

        //the gradient is carried out over 255 maximum steps
        for (gradStep = 0; gradStep < 255; gradStep++) {

            //within each gradient step, set up each color step
            for (v = 0; v < 6; v++){

                //get current values for the working color
                tempRval = getR(currentColorState[v]);
                tempGval = getG(currentColorState[v]);
                tempBval = getB(currentColorState[v]);

                targetRval = getR(targetColorState[v]);
                targetGval = getG(targetColorState[v]);
                targetBval = getB(targetColorState[v]);

                //compare Red values
                if (tempRval > targetRval){
                    tempRval--;
                }
                else if (tempRval < targetRval){
                    tempRval++;
                }

                //compare Green values
                if (tempGval > targetGval){
                    tempGval--;
                }
                else if (tempGval < targetGval){
                    tempGval++;
                }

                //compare Blue values
                if (tempBval > targetBval){
                    tempBval--;
                }
                else if (tempBval < targetBval){
                    tempBval++;
                }

                //repack the color
                currentColorState[v] = packColor(tempRval, tempGval, tempBval);

            }//end color setup for loop

            //write the colors
            for (ledNum = 0; ledNum < 6; ledNum++){
                writePacCol(currentColorState[ledNum]);
            }

            //data stream written, wait to complete
            wait_50us();

            //insert 4ms delay
            delay(4);

        }//end gradient while loop


        //reset the number of LEDs that are on
        LEDtotal = 0;

        //need at least four LEDs on out of the six
        while (LEDtotal < minLEDs) {

            LEDtotal = 0;

            if (minLEDs == 6){
                currentState = 0x003f;  //all LEDs on, just set the state
            }
            else {
                currentState = lfsr(currentState);  //get new state
            }

            //total up the sum of digits, representing the number of active LEDs
            if ((currentState & 0x01) == 1) LEDtotal++;
            if ((currentState & 0x02) == 2) LEDtotal++;
            if ((currentState & 0x04) == 4) LEDtotal++;
            if ((currentState & 0x08) == 8) LEDtotal++;
            if ((currentState & 0x10) == 16) LEDtotal++;
            if ((currentState & 0x20) == 32) LEDtotal++;

            if (minLEDs == 6);  //all LEDs on, don't worry about duplicates
            //if a next duplicate state is encountered when using a mask of six bits
            else if ((currentState & 0x3F) == (tempState & 0x3F)) LEDtotal = 0;

        }//end LFSR while loop

        //found a good state, save it in case it gets duplicated in the next one
        tempState = currentState;

        //save a new lastColorState and update currentColorState
        for (j = 0; j < 6; j++){
            lastColorState[j] = targetColorState[j];
            currentColorState[j] = targetColorState[j]; //possibly redundant
        }

        //this could be adjustable with a pot in the future (0.05s - 2s)
        //this could possibly be a sleep function to go to low power mode
        delay(1000);    //wait 1.0 seconds

        /*
        //hardcoded writeColor
        //red
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        //green
        write_0();
        write_0();
        write_0();
        write_1();
        write_0();
        write_0();
        write_0();
        write_0();
        //blue
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        write_0();
        //reset
        wait_50us();
        */

    }//end while loop

    return 0;

}//end main

//FUNCTIONS

// setup function
// Initializes PIC24 configuration settings, such as the clock, I/O, and pin
//  states. This is run only once on microcontroller startup. Other settings may
//  be added or changed here later as needed by the device.
// Arguments: None
// Returns:   None
void setup(void){
    // Execute once code here
    CLKDIVbits.RCDIV = 0;   //set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M

    AD1PCFG = 0x9ffd;       //control analog(0) or digital(1) operation,
                            //set RA0 digital, set AN1 analog for ADC
                            //0b 1001 1111 1111 1101

    TRISA = 0xfffe;         //set pin modes to output(0) or input(1)
                            //set RA0 to output, set AN1 to input
                            //0b 1111 1111 1111 1110

    //TRISB = 0xffff;         //set pin modes to output(0) or input(1)
    TRISBbits.TRISB7 = 1;   //RB7/INT0 is input
    TRISBbits.TRISB9 = 0;   //RB9 is output (LED testing)

    LATA = 0x0101;          //set RA0 to HIGH(1)
                            //0b 0000 0001 0000 0001

    LATB = 0x0000;          //set all outputs to LOW(0)

    //set up ADC stuff
    AD1CON1 = 0x0000;        //when SAMP bit = 0, end sample, start conversion
    AD1CON2 = 0x0000;
    AD1CON3 = 0x0000;
    AD1CHS = 0x0001;        //using AN1

    AD1CON1bits.SSRC = 0b111;   //use auto-convert
    AD1CON3bits.SAMC = 10;      //Auto-Sample Time, Tad = 10
    AD1CON3bits.ADCS = 2;       //A/D Conv Clock Select Bits, clock = 3 * Tcy

    _AD1IF = 0;             //clear flag
    _AD1IE = 0;             //disable interrupt

    AD1CON1bits.ADON = 1;   //start ADC

    //set up INT0
    _INT0EP = 0;            //positive edge trigger
    _INT0IP = 6;            //interrupt priority
    _INT0IF = 0;            //reset INT0 flag
    _INT0IE = 1;            //enable INT0

}//end setup


// delay function
// Arguments: An integer representing the number of milliseconds to delay.
// Returns:   None
void delay(int ms){
    int i;
    int iters = 2*ms;
    for (i = 0; i < iters; i++ ){

        //For some undetermined reason, a single wait_50us() in a for loop
        //  that simply repeats the needed amount results in timing issues,
        //  where calling 20 in a row does not.

        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles
        wait_50us();                //806 cycles

    }//end for
}//end delay


// lfsr function
// This function takes a 16 bit number and runs a series of linear feedback shift
//  operations on it. The next state it finds is guaranteed to be different than
//  the start state it was given.
// Arguments: unsigned 16bit integer representing an input start state
// Returns:   unsigned 16bit integer representing the next state
uint16_t lfsr(uint16_t start){

    uint16_t state = start;	// Initially set the current state to the start state

	//example set of shift operations
	//state ^= state >> 7;	// 0b10110001 XOR with 0b00000001
	//state ^= state << 3;	// 0b10110000 XOR with 0b10000000
	//state ^= state >> 2;	// 0b00110000 XOR with 0b00001100
							// end result state = 0b00111100

    // These shift operations result in 65535 states before returning to the start state.

	state ^= state >> 7;
	state ^= state << 9;
	state ^= state >> 13;

	return state;	// Return the next state found

}//end lfsr function


// writeColor function
// Writes to a bit stream to RA0 that changes the color. Note that after this function
//  is called, if the series of colors written is complete, a call to wait for
//  50us must be done, using the wait_50us function.
// Arguments: Three integers, which use the least significant eight bits of each to
//             determine the values for red, blue, and green.
// Returns:   None
void writeColor(int r, int g, int b){
    int a = 0;                      //3 cycle, incl pop
    for (a = 0; a < 8; a++){        //829 cycles
        if (0b10000000 & r)
            write_1();
        else write_0();
        r = r << 1;
    }//end r for loop

    for (a = 0; a < 8; a++){
        if (0b10000000 & g)
            write_1();
        else write_0();
        g = g << 1;
    }//end g for loop

    for (a = 0; a < 8; a++){
        if (0b10000000 & b)
            write_1();
        else write_0();
        b = b << 1;
    }//end b for loop

    //wait_50us();                //806 cycles
    return;                         //3 cycles
}//end writeColor


//packColor function
//returns a packedColor value
unsigned long int packColor(int red, int grn, int blu){
    unsigned long int RGBval = 0;
    RGBval = ((long) red << 16 | ((long) grn << 8) | (long) blu);
    return RGBval;
}//end packColor

//getR function
//returns the red part of a packedColor
int getR(unsigned long int RGBval){
    int red = 0;
    red = (int) (RGBval >> 16);
    red = red & 0x00FF;
    return red;
}//end getR

//getG function
//returns the green part of a packedColor
int getG(unsigned long int RGBval){
    int green = 0;
    green = (int) (RGBval >> 8);
    green = green & 0x00FF;
    return green;
}//end getG

//getB function
//returns the blue part of a packedColor
int getB(unsigned long int RGBval){
    int blue = 0;
    blue = (int) (RGBval >> 0);
    blue = blue & 0x00FF;
    return blue;
}//end getB

//writePackCol function
//writes writes a color given a packedColor
void writePacCol(unsigned long int PackedColor){
    int red = getR(PackedColor);
    int green = getG(PackedColor);
    int blue = getB(PackedColor);
    writeColor(red,green,blue);
    return;
}//end writePacCol


//Unknown if functions below are necessary

//cycler1 function
//creates a smooth gradient from red to blue, and back
//purple in between
void cycler1(void){
    int j;
    for (j = 255; j >= 0; j--){
        drawFrame(j);
    }//end for
    for (j = 0; j < 256; j++){
        drawFrame(j);
    }//end gradient for loop
    return;
}//end cycler1

//cycler2 function
//smooth gradient from Red to Blue to Red
//black/darkness in between
void cycler2(void){
    int j;
    for (j = 255; j >= 0; j--){
        primFrame(j,1);
    }//end for
    for (j = 0; j < 256; j++){
        primFrame(j,3);
    }//end gradient for loop
    for (j = 255; j >= 0; j--){
        primFrame(j,3);
    }//end for
    for (j = 0; j < 256; j++){
        primFrame(j,1);
    }//end for
    return;
}//end cycler2


//wheeldemo function
//demonstrates the wheel implementation
void wheeldemo(void){
    int j;
    for (j = 0; j < 256; j++){
        writePacCol(Wheel(j));
        delay(8);               //delay ~8ms
    }//end wheel for loop
    for (j = 255; j >= 0; j--){
        writePacCol(Wheel(j));
        delay(8);               //delay ~8ms
    }//end wheel for loop
    return;
}//end wheeldemo


//primFrame function
//writes a color of a single frame of a 255 part primary gradient
//max brightness is 255
void primFrame(int frame,int color){
    if (color == 1){
        writeColor(frame,0,0);
        delay(4);   //~4ms delay
        return;
    }//end red if
    if (color == 2){
        writeColor(0,frame,0);
        delay(4);   //~4ms delay
        return;
    }//end green if
    if (color == 3){
        writeColor(0,0,frame);
        delay(4);   //~4ms delay
        return;
    }//end blue if
    return;
}//end primFrame


//drawFrame function
//writes a color of a single frame of a 255 part gradient
void drawFrame(int frame){
    int intRed = frame;
    int intBlue = 255 - frame;
    writeColor(intRed,0,intBlue);
    delay(4);   //delay ~4 ms
    return;
}//end drawFrame



//Wheel function
//returns a packed color given an integer between 0 and 255
unsigned long int Wheel(unsigned char WheelPos){
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return packColor(255 - WheelPos * 3, 0, WheelPos * 3);  //return packed color
    }//end if
    if (WheelPos < 170){
        WheelPos -= 85;
        return packColor(0,WheelPos * 3, 255 - WheelPos * 3);
    }//end if
    WheelPos -= 170;
    return packColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}//end Wheel
