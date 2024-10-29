#include "main.h"
#include "spi.h"
#include "networks.h"
#include "SPIRIT_Types.h"
#include "nodes.h"
#include "timer.h"
#include <string.h>
#include "uart.h"

#include <stdio.h>
#include "spsgrf.h"

volatile SpiritFlagStatus xTxDoneFlag;
volatile SpiritFlagStatus xRxDoneFlag;
volatile SpiritFlagStatus xTxFifoFlag;
volatile SpiritFlagStatus xRxFifoFlag;

#define START 			0x01
#define ACK 			0x02
#define HEARTBEAT 		0x03
#define MESSAGE			0x04

uint8_t START_CODE[1] 		= {0x01};
uint8_t ACK_CODE[1]			= {0x02};
uint8_t HEARTBEAT_CODE[1] 	= {0x03};
uint8_t MESSAGE_CODE[1]		= {0x04};


/*
 * Function that forces the RF board into the READY state
 */
void SpiritGotoReadyState(void) {
  static unsigned int i;

  /* Wait for the radio to enter the ready state */
  do {

    /* Go to the ready state */
    if (g_xStatus.MC_STATE == MC_STATE_LOCK) {
      SpiritCmdStrobeReady();
    } else {
      SpiritCmdStrobeSabort();
    }

    /* Delay for state transition */
    for (i = 0; i != 0xFF; i++)
      ;
    /* Update the global status register variable */
    SpiritRefreshStatus();

  } while (g_xStatus.MC_STATE != MC_STATE_READY);
}

/*
 * Transmits data to the destination address
 */
void transmitData(uint8_t address, uint8_t *payload, uint8_t txLen) {
	/* Force READY state */
	SpiritGotoReadyState();
	/* Set destination address */
	SpiritPktStackSetDestinationAddress(address);
	xTxDoneFlag = S_RESET;
	/* Start Transmission */
	SPSGRF_StartTx(payload, txLen);
	/* Wait until transmission is done and in READY state */
	while(!xTxDoneFlag){
		SpiritRefreshStatus();
	}
}

/*
 * Receives data and the source address
 */
uint8_t receiveData(uint8_t *payload, uint8_t *recNode, uint8_t *destNode) {

	xRxDoneFlag = S_RESET;
	/* Start Receiving */
	SPSGRF_StartRx();
	/* Force READY state */
	while (!xRxDoneFlag);
	/* Get the Source Address */
	*recNode = SpiritPktStackGetReceivedSourceAddress();
	*destNode = SpiritPktStackGetReceivedDestAddress();
	return SPSGRF_GetRxData(payload);
}

/*
 * Sends a heartbeat
 */
void sendHeartbeat(void) {
	uint8_t packet[22];
	packet[0] = HEARTBEAT_CODE[0];
	memcpy((char*)packet+1, myName, strlen(myName)+1);
	transmitData(BROADCAST_ADDRESS, packet, strlen((char*)packet)+1);

}

/*
 * Joins the network
 */
void joinNetwork() {
	uint8_t packet[22] = {0};
	packet[0] = START_CODE[0];
	memcpy((char*)packet+1, myName, strlen(myName)+1);
	transmitData(BROADCAST_ADDRESS, packet, strlen((char*)packet)+1);
}


/*
 * Sends an ACK
 */
void sendAck(uint8_t dest_addr) {
	uint8_t packet[22] = {0};
	packet[0] = ACK_CODE[0];
	memcpy((char*)packet+1, myName, strlen(myName)+1);
	transmitData(dest_addr, packet, strlen((char*)packet)+1);
}

uint16_t buildPDU(uint8_t *buffer, uint8_t *payload, uint8_t type) {
	uint16_t totLen = 0;


	*buffer++ = type;
	totLen++;
	strcpy((char*)buffer, myName);

	totLen += strlen(myName) +1;	// +1 for null

	if(type == 0x04) {
		buffer += strlen(myName) + 1;
		strcpy((char*)buffer, (char*)payload);
		totLen += strlen((char*)payload) + 1;
	}
	return totLen;

}

void printMessage(uint8_t *buffer, uint8_t recNode, uint8_t destNode) {
	uint8_t srcName[21];
	strcpy((char*)srcName, (char*)buffer);
	buffer += strlen((char*)srcName) + 1;
	if(destNode == BROADCAST_ADDRESS) {
		// change color
	}
	else {
		// reset color
	}
	UART_print((char*) srcName);
	UART_print(":\x20");
	UART_print((char*)buffer);
	UART_print("\n\r");
}
