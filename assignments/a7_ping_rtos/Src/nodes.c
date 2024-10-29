#include "main.h"
#include "nodes.h"
#include <stdio.h>
#include <string.h>

#include "usart.h"
#define MAX_NODES 256

static uint32_t onlineList[MAX_NODES];
uint32_t TIME_COUNT = NODE_TIMEOUT;

/*
 * Updates the onlineList
 */
void updateNodes(uint8_t node) {
	onlineList[node] = TIME_COUNT;
}

uint8_t refreshNodes(void) {
	uint16_t i, numElements = 0;
	char charInfoBuf[64];	// buffer large enough to hold string for the serial terminal

	/* Sets the Cursor ready to print */
	HAL_UART_Transmit(&huart2, (const uint8_t *) "\x1B[2J", 4, HAL_MAX_DELAY);	// clear screen
	HAL_UART_Transmit(&huart2, (const uint8_t *) "\x1B[H", 3, HAL_MAX_DELAY);	// move cursor to the top left corner
	HAL_UART_Transmit(&huart2, (const uint8_t *) "   Address  |  Last Heartbeat\n\r", 31, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (const uint8_t *) "------------|-------------------\n\r", 34, HAL_MAX_DELAY);
	/* Go through all the nodes */
	for(i = 0; i < 256; i++) {
		/* If they sent a heartbeat within NODE_TIMEOUT seconds */
		if((TIME_COUNT - onlineList[i]) <= NODE_TIMEOUT) {
			/* Prints the online Nodes */
			HAL_UART_Transmit(&huart2, (const uint8_t *) "\x1B[5C", 4, HAL_MAX_DELAY);	// move cursor right 5
			/* print node address */
			snprintf(charInfoBuf, 64, "%03d", i);
			HAL_UART_Transmit(&huart2, (const uint8_t *)charInfoBuf, strlen(charInfoBuf), HAL_MAX_DELAY);
			/* cursor movements */
			HAL_UART_Transmit(&huart2, (const uint8_t *) "\x1B[4C|", 5, HAL_MAX_DELAY);	// move cursor right
			HAL_UART_Transmit(&huart2, (const uint8_t *) "\x1B[6C", 5, HAL_MAX_DELAY);	// move cursor right 6
			/* print node time */
			snprintf(charInfoBuf, 64, "%d\n\r", (int)onlineList[i] - NODE_TIMEOUT);
			HAL_UART_Transmit(&huart2, (const uint8_t *)charInfoBuf, strlen(charInfoBuf), HAL_MAX_DELAY);

			numElements++;
		}
		else {
			/* Puts everyone else offline */
			onlineList[i] = OFFLINE;
		}
	}
	/* No Nodes Online */
	if(numElements == 0) {
		HAL_UART_Transmit(&huart2, (const uint8_t *)"     No Nodes Online\r\n", 22, HAL_MAX_DELAY);
	}

	/* Prints the Current Time */
	HAL_UART_Transmit(&huart2, (const uint8_t *) "------------|-------------------\n\r", 34, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (const uint8_t *)"     Current Time: ", 19, HAL_MAX_DELAY);
	snprintf(charInfoBuf, 64, "%d", (int)TIME_COUNT-NODE_TIMEOUT);
	HAL_UART_Transmit(&huart2, (const uint8_t *)charInfoBuf, strlen(charInfoBuf), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (const uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
	return numElements;
}
