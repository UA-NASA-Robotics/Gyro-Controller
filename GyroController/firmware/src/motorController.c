
/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */
#include "motorController.h"
#include "CAN.h"


//typedef union {
//  struct {
//    CAN_RX_MSG_SID msgSID;
//    CAN_MSG_EID msgEID;
//    uint8_t data[8];
//  }
//  uint8_t dataOnlyMsgData[8];
//  uint32_t messageWord[4];
//} CAN_RX_MSG_BUFFER;

void initMotorLeft(void)
{
    initMotor(MOTOR_L_ADDRESS);
}

void initMotor(uint8_t motorAddress)
{    
    sendMotorPacket(motorAddress,CLEAR_ERRORS);
    sendMotorPacket(motorAddress,POWER_DISABLE);
    sendMotorPacket(motorAddress,SET_FEEDBACK_ENCODER);
    sendMotorPacket(motorAddress,SET_SVEL_FEEDBACK_ENCODER);
    sendMotorPacket(motorAddress,SET_FEEDBACK_RESOLUTION);
    sendMotorPacket(motorAddress,SET_MOTOR);
    sendMotorPacket(motorAddress,SET_POLES);
    sendMotorPacket(motorAddress,CURRENT_LIMIT_POS);
    sendMotorPacket(motorAddress,CURRENT_LIMIT_NEG);
    sendMotorPacket(motorAddress,VELOCITY_LIMIT_POS);
    sendMotorPacket(motorAddress,VELOCITY_LIMIT_NEG);
    sendMotorPacket(motorAddress,MOTOR_POLARITY);
    sendMotorPacket(motorAddress,POWER_ENABLE);
    sendMotorPacket(motorAddress,MODE_VEL);
    sendMotorPacket(motorAddress,VEL_ACC);
    sendMotorPacket(motorAddress,VEL_DEC);                     
}


void sendMotorCommand(long value)
{    
	sendMotorPacket(MOTOR_L_ADDRESS,0x3300,0x0,value);	
}


/* *****************************************************************************
 End of File
 */