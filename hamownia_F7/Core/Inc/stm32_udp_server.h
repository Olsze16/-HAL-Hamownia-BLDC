/*
 * stm32_udp_server.h
 *
 *  Created on: Jul 19, 2020
 *      Author: daniel
 */

#ifndef INC_STM32_UDP_SERVER_H_
#define INC_STM32_UDP_SERVER_H_


#include "lwip.h"
#include "lwip/udp.h"
#include "string.h"
#include "main.h"


extern struct pbuf *ppp;
//--------------------------------------------------------------
/* STM32 Mac adress */
#define myMAC_ADDR0   0x00
#define myMAC_ADDR1   0x80
#define myMAC_ADDR2   0xE1
#define myMAC_ADDR3   0x00
#define myMAC_ADDR4   0x00
#define myMAC_ADDR5   0x00

//--------------------------------------------------------------
/* Static Ip Adress */
#define IP_ADDR0   169
#define IP_ADDR1   254
#define IP_ADDR2   43
#define IP_ADDR3   48

//--------------------------------------------------------------
/* Net mask: 255.255.255.0 */
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   0
#define NETMASK_ADDR3   0

//--------------------------------------------------------------
/* Gateway: 169.254.136.1 */
#define GW_ADDR0   169
#define GW_ADDR1   254
#define GW_ADDR2   43
#define GW_ADDR3   71

//--------------------------------------------------------------
/* Ip host address */
#define HOST_IP_0   169
#define HOST_IP_1   254
#define HOST_IP_2   43
#define HOST_IP_3   71

//--------------------------------------------------------------
/* UDP port number */
#define  SERVER_UDP_PORT      7
#define  CLIENT_USP_PORT      49695   //58537

//--------------------------------------------------------------
/* Max size of buffer */
#define  UDP_RECEIVE_MSG_SIZE   150

//--------------------------------------------------------------
/* Structure with connection information */
typedef enum {
  UDP_ST_NOINIT = 0,		/* Not initialized yet */
  UDP_ST_NOCONNECT = 1, 	/* Initialized but connection not made */
  UDP_ST_RUNNING = 2		/* Initialized and connected */
}SERVER_STATUS_t;

typedef struct {
  SERVER_STATUS_t status;
  volatile uint8_t recBufer[UDP_RECEIVE_MSG_SIZE];
  uint32_t recLength;
}UDP_CONN_t;

typedef enum {
  UDP_INIT_OK = 0,
  UDP_INIT_ETH_MACDMA_ERR = 1,
  UDP_INIT_ETH_PHYINT_ERR = 2
}UDP_INIT_STATUS_t;

typedef enum {
  UDP_SER_CON_STA_CONNECT_OK = 0,
  UDP_SER_CON_STA_INIT_ERR = 1,
  UDP_SER_CON_STA_RUNNING = 2,
  UDP_SER_CON_STA_NO_LINK = 3,
  UDP_SER_CON_STA_ERROR_1 = 4,
  UDP_SER_CON_STA_ERROR_2 = 5
}UDP_SERVER_CONNECT_t;

typedef enum {
  UDP_SERVER_BUF_OFFLINE = 0,
  UDP_REVEICE_BUF_EMPTY = 1,
  UDP_RECEIVE_BUF_READY = 2
}UDP_RECEIVE_t;

//--------------------------------------------------------------
/* Global function */
uint8_t serverUDPInit(void);
//--------------------------------------------------------------
UDP_SERVER_CONNECT_t serverUDPStart(void);
//--------------------------------------------------------------
void serverUDPDisconnect(void);
//--------------------------------------------------------------
uint8_t serverUDPSendString(uint8_t *ptr);
//--------------------------------------------------------------
UDP_RECEIVE_t serverUDPWorks(uint8_t *ptr);
//--------------------------------------------------------------
void ethernetif_notify_conn_changed(struct netif *netif);
//--------------------------------------------------------------

#endif /* INC_STM32_UDP_SERVER_H_ */
