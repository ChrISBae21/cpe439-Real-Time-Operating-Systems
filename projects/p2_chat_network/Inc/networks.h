#include "SPIRIT_Types.h"

#ifndef SRC_NETWORKS_H_
#define SRC_NETWORKS_H_


void transmitData(uint8_t address, uint8_t *payload, uint8_t txLen);
uint8_t receiveData(uint8_t *payload, uint8_t *recNode, uint8_t *destNode);
void joinNetwork(void);
void sendAck(uint8_t dest_addr);
void sendHeartbeat(void);
void printMessage(uint8_t *buffer, uint8_t recNode, uint8_t destNode);

uint16_t buildPDU(uint8_t *buffer, uint8_t *payload, uint8_t type);


extern uint8_t START_CODE[1];
extern uint8_t ACK_CODE[1];
extern uint8_t HEARTBEAT_CODE[1];
extern uint8_t MESSAGE_CODE[1];


extern volatile SpiritFlagStatus xTxDoneFlag;
extern volatile SpiritFlagStatus xRxDoneFlag;
extern volatile SpiritFlagStatus xTxFifoFlag;
extern volatile SpiritFlagStatus xRxFifoFlag;

#endif /* SRC_NETWORKS_H_ */
