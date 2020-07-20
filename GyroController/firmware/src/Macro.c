#include "Macro.h"
#include "PID.h"
#include "Timers.h"
#include "Motor.h"
#include "MPU6050.h"
#include "Definitions.h"
#include "drivingControl.h"
#include "FastTransfer.h"
#include "Communications.h"
#include "Macro_Handler/Macro_Mgr.h"
#include "CAN_Handler/GlobalCAN_IDs.h"
#include "CAN_Handler/CAN.h"
#include "CAN_Handler/CANFastTransfer.h"
#include "DataPublishing.h"
#define MOTOR_MIN_SPEED 1000
#define MOTOR_MAX_SPEED 3000
#define ROTATION_ANGLE_TOLERANCE 2
void updateLocalGyro();
void transmitGyroDone();


bool(*runConfiguredMacro)();
bool runningMacroData = 0;
bool MacroRunning = false;
bool isInRange(float currentVal, float destinationVal, float tolerence);
double localAngle = 0;
double StartAngle = 0;
double lastAngle = 0;
double lastDir = 0;
double lastSpeed = 0;
PID_Struct_t RotatePID;
timers_t debugTimer;
timers_t updateTimer;


timers_t voidTime, voidTime2;

typedef enum {
    InitMacro = 0,
    Rotate
} RotateMacro_t;
RotateMacro_t RotateMacro = InitMacro;

bool Dummy(int val) {
    if (voidTime.timerInterval != val) {
        setTimerInterval(&voidTime, val);
    }
    return timerDone(&voidTime);
}

bool Dummy2(int val) {
    if (voidTime2.timerInterval != val) {
        setTimerInterval(&voidTime2, val);
    }
    return timerDone(&voidTime2);
}

void handleMacroStatus() {
    ReceiveDataCAN(FT_GLOBAL);
    ReceiveDataCAN(FT_LOCAL);
    /* If a macro is seen on the global bus from the router card*/
    if (getNewDataFlagStatus(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD))) {
        int macroID = getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD));
        int macroDATA = getCANFastData(FT_GLOBAL, getGBL_MACRO_INDEX(ROUTER_CARD) + 1);
        handleCANmacro(macroID, macroDATA);
    }
    if (getNewDataFlagStatus(FT_LOCAL, CAN_COMMAND_INDEX)) {
        int macroID = getCANFastData(FT_LOCAL, CAN_COMMAND_INDEX);
        int macroDATA = getCANFastData(FT_LOCAL, CAN_COMMAND_DATA_INDEX);
        handleCANmacro(macroID, macroDATA);
    }

}

void handleCANmacro(short _macroID, short _macroDATA) {
    if (_macroID == 0) {
        clearMacros();
        MotorsAllStop();
    } else {
        /* Add the macro to the queue*/
        switch (_macroID) {
            case ROTATION_COMMAND:
                setMacroCallback(turnDegrees, _macroDATA, ROTATION_COMMAND);
                break;
            default:
                break;
        }

    }
}

void configureMacro(int macroID, int macroData) {
    switch (macroID) {
        case ROTATION_COMMAND:
        {
            localAngle = 0;
            lastAngle = 0;

            lastDir = 0;
            lastSpeed = 0;

            setTimerInterval(&debugTimer, 100);
            setTimerInterval(&updateTimer, 50);



            lastAngle = getY_Angle();
            INIT_PID(&RotatePID, macroData, MOTOR_ROTATION_kp, MOTOR_ROTATION_ki, MOTOR_ROTATION_kd);
            runConfiguredMacro = turnDegrees;
            runningMacroData = macroData;
            MacroRunning = true;
            break;
        }
        case ROTATION_MONITORING:
        {
            localAngle = 0;
            lastAngle = getY_Angle();
            StartAngle = getY_Angle();
            runConfiguredMacro = monitorDrive;
            MacroRunning = true;

            break;
        }
    }
}

bool isMacroRunning() {
    return MacroRunning;
}

void runMacro() {
    if (runConfiguredMacro != NULL) {
        runConfiguredMacro();
    }
}

void stopMacro() {
    runConfiguredMacro = NULL;
    MacroRunning = false;
    LED1 = off;
    LED2 = off;
    LED3 = off;
    LED4 = off;
}

void macroComplete(MacroTypes lastMacro) {
    switch (lastMacro) {
        case Drive_Monitoing:
        {
            //MotorsAllStop();
            resetMPUAngles();
            LED1 = off;
            break;
        }
        case TURNING_MACRO:
        {
            MacroRunning = false;
            transmitGyroDone();
            resetMPUAngles();
            stopMacro();
            break;
        }
    }

}

bool monitorDrive() {
    //update the gyro data
    updateLocalGyro();
    LED1 = on;
    if (!isInRange(localAngle, 0, ROTATION_ANGLE_TOLERANCE)) {
        saveMotorParms();
        if (localAngle < 0) {
            slowLeftSpeed();
        } else {
            slowRightSpeed();
        }

        while (!isInRange(localAngle, 0, ROTATION_ANGLE_TOLERANCE - 0.5)) {
            updateLocalGyro();
        }
        restoreMotors();
        //macroComplete(MacroTypes lastMacro)
    }
    return false;
}

void updateLocalGyro() {
    updateYAxis();
    if (getY_Angle() && lastAngle != getY_Angle()) {
        localAngle += getY_Angle() - lastAngle;
        lastAngle = getY_Angle();
    }
}

void resetMacroStates() {

    RotateMacro = InitMacro;
}
int deg;

bool turnDegrees(int _rotation) {
    switch (RotateMacro) {
        case InitMacro:
            localAngle = 0;
            lastAngle = 0;

            lastDir = 0;
            lastSpeed = 0;
            deg = _rotation;
            setTimerInterval(&debugTimer, 100);
            setTimerInterval(&updateTimer, 50);

            publishDataIndex(DEVICE_MACRO);
            lastAngle = getY_Angle();
            INIT_PID(&RotatePID, _rotation, MOTOR_ROTATION_kp, MOTOR_ROTATION_ki, MOTOR_ROTATION_kd);
            RotateMacro = Rotate;
            break;
        case Rotate:
            updateYAxis();
            //update the gyro data
            if (lastAngle != getY_Angle()) {

                localAngle += getY_Angle() - lastAngle;
                lastAngle = getY_Angle();
            }
            //localAngle = getY_Angle() - StartAngle;
            if (timerDone(&debugTimer)) {
                //printf("degrees = %f  of %2f\r",localAngle,RotatePID._target);
                LED1 ^= 1;
                publishDataIndex(DEVICE_MACRO);
            }
            if (isInRange(localAngle, RotatePID._target, ROTATION_ANGLE_TOLERANCE)) {
                MotorsAllStop();
                resetMPUAngles();
                LED1 = off;
                RotateMacro = InitMacro;
                return true;
            }
            if (timerDone(&updateTimer)) {
                updateMotors(localAngle);
            }
            //macro is not complete
            return false;
            //if we are not running the motor macro we will want to stop the motors and 
            //    if(getPerformNavigationCommand() != MOTOR_COM_DRIVE)
            //    {
            //        setPerformNavigationCommand(0);
            //        MacroModeComplete();
            //        //LED7 ^= 1;
            //    } 

            //TODO: we need to keep track of the angles that are discarded as a result of the tolerance values(we are not turning back to zero and need to tally those values)

            break;
    }
    return false;
}

bool isInRange(float currentVal, float destinationVal, float tolerence) {
    if (abs(currentVal - destinationVal) < tolerence) {
        return true;
    } else {
        return false;
    }
}

void updateMotors(float updatedAngle) {
    int speed = 0;
    speed = updateOutput(&RotatePID, updatedAngle);

    if (abs(speed) < MOTOR_MIN_SPEED)
        speed = (speed > 0 ? 1 : -1) * MOTOR_MIN_SPEED;
    if (abs(speed) > MOTOR_MAX_SPEED)
        speed = (speed > 0 ? 1 : -1) * MOTOR_MAX_SPEED;

    //        lastDir = _Dir;
    //        lastSpeed = speed;if(timerDone(&debugTimer2))

    setMotor_Vel(speed, speed);

}

void testMoitorDrive(int dist, int speed) {
    //set position mode
    setMotorControlMode(&LeftMotor, Position, speed);
    setMotorControlMode(&RightMotor, Position, speed);

    //Motor Position Clear
    storeMotorPositionReached(&LeftMotor, false);
    storeMotorPositionReached(&RightMotor, false);

    sendMotorPacket(LEFTMOTORID, SSI_ENCODER_POSITION_RESET);
    sendMotorPacket(RIGHTMOTORID, SSI_ENCODER_POSITION_RESET);

    //set to drive a distance
    setMotorCounts(&LeftMotor, dist * COUNTS_PER_CENTI);
    setMotorCounts(&RightMotor, -dist * COUNTS_PER_CENTI);



    configureMacro(ROTATION_MONITORING, 0);
}

void testTurnDegrees(int degrees) {
    configureMacro(ROTATION_COMMAND, degrees);
}
#define MACRO_RETURN_STATUS_INDEX 1
#define MACRO_DONE                1

void transmitGyroDone() {
    ToSend(&MasterFT, MACRO_RETURN_STATUS_INDEX, MACRO_DONE);
    sendData(&MasterFT, MASTER_CONTROLLER);
}