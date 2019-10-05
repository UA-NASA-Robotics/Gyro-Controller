#include "Communications.h"
#include "FastTransfer.h"
#include "Definitions.h"

void confirmMacro(unsigned char macro) {
    ToSend(&MasterFT, UART_COMMAND_INDEX, macro);
    sendData(&MasterFT, MASTER_ADDRESS);
}

