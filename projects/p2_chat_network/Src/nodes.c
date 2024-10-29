#include "main.h"
#include "nodes.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"

#include "uart.h"

struct nodeinfo onlineNodes[256] = {OFFLINE};		// create a list of nodes

uint32_t TIME_COUNT = NODE_TIMEOUT;


/*
 * Updates the onlineList
 */
void updateNodes(uint8_t node, uint8_t *name) {

	strcpy((char*)onlineNodes[node].name, (char*)name);
	onlineNodes[node].time = TIME_COUNT;
}


void printNodes(void) {
	uint16_t i, numElements = 0;
	char nameBuf[64] = {0};
	char timeBuf[20] = {0};

	/* Sets the Cursor ready to print */

	UART_print("   Last Heartbeat  |  Address  |  Username\n\r");
	UART_print("-------------------|-----------|-------------\n\r");

	for(i = 0; i < 256; i++) {
		/* If they sent a heartbeat within NODE_TIMEOUT seconds */
		if((TIME_COUNT - onlineNodes[i].time) <= NODE_TIMEOUT) {
			/* Prints the online Nodes */
			snprintf(nameBuf, 64, "     %d           |        %03d |      %s\n\r", (int)onlineNodes[i].time - NODE_TIMEOUT, i, (char*)onlineNodes[i].name);
			UART_print(nameBuf);
			numElements++;
		}
	}
	/* No Nodes Online */
	if(numElements == 0) {
		UART_print("                    No Nodes Online\r\n");
	}

	/* Prints the Current Time */
	UART_print("-------------------|-----------|-------------\n\r");
	UART_print("                    Current Time: ");
	snprintf(timeBuf, 20, "%d\n\r", (int)TIME_COUNT-NODE_TIMEOUT);
	UART_print(timeBuf);
}

void refreshNodes(void) {
	uint32_t node;
	/* Go through all the nodes */
	for(node = 0; node < 256; node++) {
		/* If they sent a heartbeat within NODE_TIMEOUT seconds */
		if((TIME_COUNT - onlineNodes[node].time) > NODE_TIMEOUT) {
			onlineNodes[node].name[0] = '\0';
			onlineNodes[node].time = OFFLINE;
		}
	}

}


/* returns a 1 if the node doesnt exist*/
uint8_t getNodeAddress(char *name) {
	uint16_t i;
	for(i = 0; i < 256; i++) {
		if((TIME_COUNT - onlineNodes[i].time) <= NODE_TIMEOUT) {
			if(strcmp(name, (char*)onlineNodes[i].name) == 0) {
				return i;
			}
		}
	}
	return 1;

}
