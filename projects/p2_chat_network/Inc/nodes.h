#include "SPIRIT_Types.h"

#ifndef SRC_NODES_H_
#define SRC_NODES_H_
void updateNodes(uint8_t node, uint8_t *name);
void refreshNodes(void);
void printNodes(void);

uint8_t getNodeAddress(char *name);

#define NODE_TIMEOUT		110
#define OFFLINE				0


struct nodeinfo {
	uint32_t time;
	uint8_t name[20];
};


#endif /* SRC_NODES_H_ */
