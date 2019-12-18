/* 
 * File:   Definitions.h
 * Author: John
 *
 * Created on April 26, 2018, 3:30 PM
 */

#ifndef DEFINITIONS_H
#define	DEFINITIONS_H


#define LED1    LATEbits.LATE4
#define LED2    LATEbits.LATE5
#define LED3    LATEbits.LATE6
#define LED4    LATEbits.LATE7

typedef enum {
    CONTROLBOX = 1,
    POZYX,
    JUICE_BOARD,
    ROUTER_CARD,
    MASTER_CONTROLLER,
    MOTOR_CONTROLLER,
    GYRO_CONTROLLER,
    STRAIN_SENSOR,
    OPTICAL_FLOW,
    RASPBERRY_PI,
    LED_CARD,
    GLOBAL_ADDRESS = 31
} Addresses_t;



#define off  1
#define on 0


#define MOTOR_ADDRESS           6

#define MY_ADDRESS              GYRO_CONTROLLER

#define UART_COMMAND_INDEX       8
#define UART_COMMAND_DATA_INDEX  9



#define CAN_COMMAND_INDEX       8
#define CAN_COMMAND_DATA_INDEX  9



//******************************************************
//                  MACRO INDEXES
//******************************************************
//**********MASTER Macros***************

    #define STARTING_CENTER         1

//**********Motor Macros****************
   
    #define ENCODER_COMMAND         1    //drive a distance
    #define CENTER_ON_START         2
    #define DIG_COMMAND             3

//**********Gyro Macros*****************
    #define ROTATION_COMMAND        1  //rotate
    #define ROTATION_MONITORING     2   

 //**************TO ALL**********************
    #define PAUSE_COMMAND           4     //Pause the cammand that is running
    #define STOP_COMMMAND           0


#endif	/* DEFINITIONS_H */

