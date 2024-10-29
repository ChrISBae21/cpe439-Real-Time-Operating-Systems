#include "SPIRIT_Types.h"
#ifndef SRC_NETWORKS_H_
#define SRC_NETWORKS_H_

void transmitData(uint8_t address, uint8_t *payload, uint8_t txLen);
uint8_t receiveData(uint8_t *payload, uint8_t *recNode);
void joinNetwork(void);
void sendAck(uint8_t dest_addr);
void sendHeartbeat(void);

extern uint8_t START_CODE[1];
extern uint8_t ACK_CODE[1];
extern uint8_t HEARTBEAT_CODE[1];
extern volatile SpiritFlagStatus xTxDoneFlag;
extern volatile SpiritFlagStatus xRxDoneFlag;
extern volatile SpiritFlagStatus xRxTimeoutFlag;

#endif /* SRC_NETWORKS_H_ */
