/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "CAN_Handler/CANFastTransfer.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;
bool isLoaded = false;
intPin_t* awaitPin;
int NEXT_APP_STATE;

#define IsMacroRunning (macroRunning != 0)
int macroRunning, macroRunningData;

timers_t sec, ms100, ms10;
timers_t bootTimer, ledTime;

void APP_Initialize(void) {
    setTimerInterval(&bootTimer, 3500);
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    setTimerInterval(&sec, 1000);
    setTimerInterval(&ms100, 100);
    setTimerInterval(&ms10, 10);
    setTimerInterval(&ledTime, 50);
    initChangeNotification(&MotorPin1, &MotorPin2, &MasterPin1, &MasterPin2);
    isLoaded = true;

    DRV_TMR0_Start();

    while (!timerDone(&bootTimer)) {
        LED1 ^= 1;
        while (!timerDone(&ledTime));
        LED2 ^= 1;
        while (!timerDone(&ledTime));
        LED3 ^= 1;
        while (!timerDone(&ledTime));
        LED4 ^= 1;
        while (!timerDone(&ledTime));
    }
    initCANISRs();
    initCANFT();
    DRV_CAN0_Open();
    initMotors();

    //InitUARTModule(&DebugUart,Uart_2);

    InitFastTransferModule(&MasterFT, Master_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    InitFastTransferModule(&MotorFT, Motor_UART, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    initChangeNotification();

    //***************INIT GYROS*************************

    beginMPU(&MPU_1, MPU6050_SCALE_250DPS, MPU6050_RANGE_2G, MPU6050_Address_1);
    zeroIMUAxisGyro();

    //*************************************************   
}

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {

                appData.state = APP_STATE_COMS_CHECK;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // Update the gyro data
            updateYAxis();
            if (timerDone(&ms100) && !isMacroRunning()) {
                LED3 ^= 1;
            } else  if (timerDone(&ms100) && isMacroRunning()) {
                LED4 ^= 1;
            }
            appData.state = APP_STATE_COMS_CHECK;
            break;
        }
        case APP_STATE_COMS_CHECK:
        {
            if(ReceiveDataCAN(FT_GLOBAL))
            {
                getCANFastData(FT_GLOBAL, MASTER_CONTROLLER*5 +1);
            }
            if(getNewDataFlagStatus(FT_GLOBAL, MASTER_CONTROLLER*5 +1 ))
            {
                LED1 ^=1;
                LED2 ^=1;
                LED3 ^=1;
                LED4 ^=1;
            }
            // CAN FastTransfer Receive
            if (ReceiveDataCAN(FT_LOCAL)) {
                if (getCANFastData(FT_LOCAL, CAN_COMMAND_INDEX) != 0) {
                    configureMacro(getCANFastData(FT_LOCAL,CAN_COMMAND_INDEX), getCANFastData(FT_LOCAL,CAN_COMMAND_DATA_INDEX));
                    clearCANFastDataValue(FT_LOCAL,CAN_COMMAND_INDEX);
                    clearCANFastDataValue(FT_LOCAL,CAN_COMMAND_DATA_INDEX);
                }
            }
            if (receiveData(&MotorFT)) {
                if (MasterFT.ReceivedData[UART_COMMAND_INDEX] != 0) {
                    if (MasterFT.ReceivedData[UART_COMMAND_INDEX] == ROTATION_COMMAND) {
                        configureMacro(MasterFT.ReceivedData[UART_COMMAND_INDEX], MasterFT.ReceivedData[UART_COMMAND_DATA_INDEX]);
                    }
                }
            }
            if (receiveData(&MasterFT)) {
                if (MasterFT.ReceivedData[UART_COMMAND_INDEX] == 0) {
                    stopMacro();
                } else if (MasterFT.ReceivedData[UART_COMMAND_INDEX] != 0) {
                    if (MasterFT.ReceivedData[UART_COMMAND_INDEX] == ROTATION_COMMAND) {
                        configureMacro(MasterFT.ReceivedData[UART_COMMAND_INDEX], MasterFT.ReceivedData[UART_COMMAND_DATA_INDEX]);

                        //turnDegrees(MasterFT.ReceivedData[UART_COMMAND_DATA_INDEX]);


                    }
                }
            }
            appData.state = APP_STATE_SERVICE_MACRO;
            break;
        }
        case APP_STATE_SERVICE_MACRO:
        {
            updateYAxis();
            if (isMacroRunning()) runMacro();
        
            //            if(getMotorPosReached(&LeftMotor))
            //            {
            //                setMotorVel(&LeftMotor,0);
            //            }
            //            
            //            if(getMotorPosReached(&RightMotor))
            //            {
            //                setMotorVel(&RightMotor,0);
            //            }
            appData.state = APP_STATE_AWAITING_RESPONSE;
            break;
        }
            //This is for waiting for an interrupt pin response
        case APP_STATE_AWAITING_RESPONSE:
        {
            //            if(pinState(&MotorPin2))
            //            {
            //                if(getPinState(MotorPin2.pinId))
            //                {
            //                     configureMacro(ROTATION_MONITORING, 0);
            //                }
            //                else
            //                {
            //                    macroComplete(ROTATION_MONITORING);
            //                }
            //               
            //            }
            appData.state = APP_STATE_SERVICE_TASKS;
            //           if(awaitPin != NULL)
            //            {
            //                LED2 = off;
            //                LED3 = off;
            //
            //                while(!pinState(awaitPin))
            //                {
            //                    if(timerDone(&ms100))
            //                    {
            //                        LED1 ^=1;
            //                        LED4 ^=1;
            //                    }
            //                }
            //                LED2 = on;
            //                LED3 = on;
            //                LED1 = off;
            //                LED4 = off;
            //                delay(100);
            //                appData.state = NEXT_APP_STATE;
            //            }

            break;
        }
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

bool getLoadedState() {
    return isLoaded;
}

void setAwaitPin(intPin_t* pin, int nextState) {
    appData.state = APP_STATE_AWAITING_RESPONSE;
    awaitPin = pin;
    NEXT_APP_STATE = nextState;
}

/*******************************************************************************
 End of File
 */
