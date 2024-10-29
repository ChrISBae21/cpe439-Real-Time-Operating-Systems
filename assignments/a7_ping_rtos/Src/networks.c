#include "main.h"
#include "spi.h"
#include "networks.h"
#include "SPIRIT_Types.h"
#include "usart.h"
#include "nodes.h"
#include "timer.h"
#include <string.h>

#include <stdio.h>
#include "spsgrf.h"

volatile SpiritFlagStatus xTxDoneFlag;
volatile SpiritFlagStatus xRxDoneFlag;

uint8_t START_CODE[1] 		= {0x01};
uint8_t ACK_CODE[1]			= {0x02};
uint8_t HEARTBEAT_CODE[1] 	= {0x03};


/* Function that forces the RF board into the READY state */
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

/* Transmits data to the destination address */
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

/* Receives data and the source address */
uint8_t receiveData(uint8_t *payload, uint8_t *recNode) {
	/* Force READY state */
	SpiritGotoReadyState();
	xRxDoneFlag = S_RESET;
	/* Start Receiving */
	SPSGRF_StartRx();
	/* Force READY state */
	while (!xRxDoneFlag && !xRxTimeoutFlag);
	/* Get the Source Address */
	*recNode = SpiritPktStackGetReceivedSourceAddress();
	/* Get the payload data */
	return SPSGRF_GetRxData(payload);
}

/* Sends a heartbeat */
void sendHeartbeat(void) {
	/* move cursor right 6 spaces */
	HAL_UART_Transmit(&huart2, (const uint8_t *)"\x1B[6C", 4, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (const uint8_t *)"Sent Heartbeat\n\r", 22, HAL_MAX_DELAY);
	/* transmit the data */
	transmitData(BROADCAST_ADDRESS, HEARTBEAT_CODE, 1);
}

/* Joins the network */
void joinNetwork(void) {
	/* transmit the data */
	transmitData(BROADCAST_ADDRESS, START_CODE, 1);
}


/* Sends an ACK */
void sendAck(uint8_t dest_addr) {
	/* transmit the data */
	transmitData(dest_addr, ACK_CODE, 1);
}
