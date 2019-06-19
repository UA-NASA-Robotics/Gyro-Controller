/* 
 * File:   changeNotification.h
 * Author: Seth Carpenter
 *
 * Created on April 21, 2018, 8:01 PM
 */

#ifndef CHANGENOTIFICATION_H
#define	CHANGENOTIFICATION_H
//#include "boolean.h"
#include "stdbool.h"
 #include <xc.h>
#define CN_PIN_COUNT 4      //the number of Change notification input pins

#define INPUT_INTURRPT_PIN_0 PORTBbits.RB9          // pin22 - IO1   - From Pin 27 on Motor
#define INPUT_INTURRPT_PIN_1 PORTBbits.RB12         // pin27 - IO3   - From pin 23 on Motor
//INPUTS from navi have internal pull downs enabled
#define INPUT_INTURRPT_PIN_2 PORTCbits.RC13         // pin47 - IO9   - From Pin 48 on Navi
#define INPUT_INTURRPT_PIN_3 PORTDbits.RD1          // pin49 - IO11  - From Pin 46 on Navi

#define OUTPUT_INTURRPT_PIN_0 LATBbits.LATB10        // pin23 - IO2   - To Pin 28 on Motor 
#define OUTPUT_INTURRPT_PIN_1 LATBbits.LATB13        // pin28 - IO4   - To Pin 24 on Motor
#define OUTPUT_INTURRPT_PIN_2 LATCbits.LATC14        // pin48 - IO10  - To Pin 47 on Navi
#define OUTPUT_INTURRPT_PIN_3 LATDbits.LATD2         // pin50 - IO12  - To Pin 45 on Navi

typedef struct{
    unsigned char pinId;    //this should be a number between 0 and 3 that corresponds to the input pin
    unsigned char prevState;
    bool changed;
}intPin_t;

intPin_t MotorPin1;
intPin_t MotorPin2;
intPin_t MasterPin1;
intPin_t MasterPin2;

void initChangeNotification();
bool pinState(intPin_t *pin);
void pinChangeNotified();
void togglePinState(intPin_t *pin);
void setPinState(intPin_t *pin, unsigned char state);
unsigned char getPinState(unsigned char ID);
#endif	/* CHANGENOTIFICATION_H */

