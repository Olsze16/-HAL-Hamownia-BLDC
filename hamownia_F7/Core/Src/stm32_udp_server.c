/*
 * stm32_udp_server.c
 *
 *  Created on: Jul 19, 2020
 *      Author: daniel
 */

#include "stm32_udp_server.h"
//#include "lwip.h"
//#include "lwip/init.h"
//#include "lwip/netif.h"

//#include "ip4_addr.h"

//typedef struct ip4_addr ip_addr;

/* Global variables */
uint8_t EthLinkStatus = 1;
/* pcb structure with connection information*/
struct udp_pcb *upcb_server;
/* Structure with info about UDP server address*/
struct ip4_addr OwnIPaddr;
/* Structure with info about UDP client address */
struct ip4_addr HostIPaddr;



extern struct netif gnetif;
struct pbuf *ppp;

UDP_CONN_t UDP_CONN_INF = {
		.status = UDP_ST_NOINIT,
		.recBufer = {0},
		.recLength = 0
};

//--------------------------------------------------------------
/* Static functions declarations */
static void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip4_addr *addr, u16_t port);
static void udpUserNotification(struct netif *netif);
//--------------------------------------------------------------
/*
 * @brief: 	Inicialization UDP server:
 * @param: 	none
 * @ret:	returns 0 if operation OK, returns 1 if connection already initialized
 */
//--------------------------------------------------------------
uint8_t serverUDPInit(void)
{
  /* Set operation status for default */
  uint8_t status = 0;

  /* check if server isn't initialized yet */
  if(UDP_CONN_INF.status == UDP_ST_NOINIT)
  {
	/* Set link status */
    EthLinkStatus = 0;


    /* Set notification for user */
    udpUserNotification(&gnetif);

    /* Set status for initialization */
    UDP_CONN_INF.status = UDP_ST_NOCONNECT;
  }
  else if(UDP_CONN_INF.status == UDP_ST_NOCONNECT)
  {
	  /* udp connection already initialized */
	  status = 1;
  }

  return status;
}

//--------------------------------------------------------------
/*
 * @brief: 	Start UDP server
 * @param: 	none
 * @ret:	returns 0 if operation OK, returns 1 if Error
 */
//--------------------------------------------------------------
UDP_SERVER_CONNECT_t serverUDPStart(void)
{
  UDP_SERVER_CONNECT_t status = UDP_SER_CON_STA_ERROR_2;
  err_t err;

  if(UDP_CONN_INF.status == UDP_ST_NOINIT)
  {
	  return UDP_SER_CON_STA_INIT_ERR;
  }

  if(UDP_CONN_INF.status == UDP_ST_RUNNING)
  {
	  return UDP_SER_CON_STA_RUNNING;
  }
  if(EthLinkStatus > 0)
  {
	  return UDP_SER_CON_STA_NO_LINK;
  }

  /* Create new UDP pcb */
  upcb_server = udp_new();

  /* return pointer */
  if (upcb_server == NULL)
  {
	  return UDP_SER_CON_STA_ERROR_1;
  }

#ifdef HOST_IP_0
  IP4_ADDR(&HostIPaddr, HOST_IP_0, HOST_IP_1, HOST_IP_2, HOST_IP_3 );
#endif

  IP4_ADDR(&OwnIPaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3 );

  /* Bind PCB to local IP address and port*/
  err = udp_bind(upcb_server, &OwnIPaddr, SERVER_UDP_PORT);

  if (err != ERR_OK)
  {
	  return UDP_SER_CON_STA_ERROR_2;
  }

  /* Set callback for incoming UDP data */
  udp_recv(upcb_server, udpReceiveCallback, NULL);

  /* change status */
  status = UDP_SER_CON_STA_CONNECT_OK;
  UDP_CONN_INF.status = UDP_ST_RUNNING;

  return status;
}

//--------------------------------------------------------------
/*
 * @brief: 	disconnect UDP server, delete info about established connection
 * @param: 	none
 * @ret: 	none
 */
//--------------------------------------------------------------
void serverUDPDisconnect(void)
{
  if(UDP_CONN_INF.status==UDP_ST_RUNNING)
  {
    udp_remove(upcb_server);
    UDP_CONN_INF.status=UDP_ST_NOCONNECT;
  }
}

//--------------------------------------------------------------
/*
 * @brief:	send string over UDP
 * @param:	ptr: pointer to string to send
 * @ret:	return status if error then 0, if OK then return 1
 */
//--------------------------------------------------------------
uint8_t serverUDPSendString(uint8_t *ptr)
{
  uint8_t status = ERROR;
  struct pbuf *p;
  err_t err;

  /* if server is enable */
  if(UDP_CONN_INF.status == UDP_ST_RUNNING)
  {
	/* allocate buffer */
    p = pbuf_alloc(PBUF_TRANSPORT, strlen(*ptr), PBUF_POOL);

    /* if buffer is allocated */
    if (p == NULL)
    {
    	status = 0;
    	return status;
    }

    /* copy data */
    pbuf_take(p, (char*)ptr, strlen((char*)ptr));
    ppp=p;
    /* Send data to a specified address using UDP */
    err = udp_sendto(upcb_server, p, &HostIPaddr, CLIENT_USP_PORT);

    /* clear sending buffer */

    for(uint8_t loop = 0; loop < UDP_RECEIVE_MSG_SIZE; loop++)
    {
    	UDP_CONN_INF.recBufer[loop] = 0;
    }

    /* free buffer */
    pbuf_free(p);

    if(err == ERR_OK)
    {
      status = SUCCESS;
    }
    else
    {
      /* remove a UDP pcb */
      udp_remove(upcb_server);
      UDP_CONN_INF.status=UDP_ST_NOCONNECT;
    }
  }

  return status;
}


//--------------------------------------------------------------
/*
 * @brief:	function use for sending data to client
 * @param:	ptr: pointer to string
 * @ret:	return UDP status
 */
//--------------------------------------------------------------
UDP_RECEIVE_t serverUDPWorks(uint8_t *ptr)
{
  UDP_RECEIVE_t status = UDP_REVEICE_BUF_EMPTY;
  uint32_t i;
  uint8_t value;

  if(UDP_CONN_INF.status != UDP_ST_NOINIT)
  {
	  /* Use when packet is ready to read from the interface */
	  ethernetif_input(&gnetif);

	  /* Handle timeout */
	  sys_check_timeouts();

    /* check linki status */
    if(EthLinkStatus > 0)
    {
      if(UDP_CONN_INF.status == UDP_ST_RUNNING)
      {
    	  /* remove a UDP pcb */
    	  udp_remove(upcb_server);
    	  UDP_CONN_INF.status = UDP_ST_NOCONNECT;
      }
    }
  }

  if(UDP_CONN_INF.status != UDP_ST_RUNNING)
  {
	  return UDP_SERVER_BUF_OFFLINE;
  }

  if(UDP_CONN_INF.recLength > 0)
  {
	  status = UDP_RECEIVE_BUF_READY;

	  /* copy buffer */
	  i=0;
	  value = UDP_CONN_INF.recBufer[i];

	  while((value != 0) && (i < UDP_RECEIVE_MSG_SIZE-1))
	  {
		  value=UDP_CONN_INF.recBufer[i];
		  ptr[i]=value;
		  i++;
	  }

	  /* clear last character */
	  ptr[i]=0x00;

	  UDP_CONN_INF.recLength=0;
  }

  return status;
}
//--------------------------------------------------------------
/*
 * @brief:	change link
 * @param:	netif: default network interface
 * @ret:	none
 */
void ethernetif_notify_conn_changed(struct netif *netif)
{
  if (netif_is_link_up(netif))
  {
    /* link is up, bring interface up, available for processing traffic */
    netif_set_up(netif);
  }
  else
  {
	/* link is down, bring interface down, disabling any traffic processing */
    netif_set_down(netif);
  }
}
//--------------------------------------------------------------
/*
 * @brief:	function receive data from udp client
 * @param:	arg:
 * @param:	upcb: structure with data conn information
 * @param:	p:	  pointer to structure with receive buffer
 * @param:	addr: pointer to info about ip address
 * @param:	port: port to listen too
 * @ret:	none
 */
//--------------------------------------------------------------
static void udpReceiveCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip4_addr *addr, u16_t port)
{
  uint16_t i;
  uint8_t value;
  uint8_t *ptr;

  ptr=(uint8_t*)p->payload;

  i=0;

  do
  {
	value = *ptr;
    UDP_CONN_INF.recBufer[i] = value;
    ptr++;
    i++;
  }while((value != 0) && (i < UDP_RECEIVE_MSG_SIZE-1));

  UDP_CONN_INF.recBufer[i]=0x00;
  UDP_CONN_INF.recLength = i;

  pbuf_free(p);
}

//--------------------------------------------------------------
/*
 * @brief:	check link status
 * @param:	netif: default network interface
 * @ret:	none
 */
static void udpUserNotification(struct netif *netif)
{
  if (netif_is_up(netif))
  {
	  /* Success */
	//  Usart_Uart_SendString(USART1, "Success netif is UP", LF_CR);
  }
  else
  {
	  /* Error */
	//  Usart_Uart_SendString(USART1, "Netif ERROR, cable not connected", LF_CR);
  }
}

