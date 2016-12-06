/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** \addtogroup cc2538-examples
 * @{
 *
 * \defgroup cc2538-echo-server cc2538dk UDP Echo Server Project
 *
 *  Tests that a node can correctly join an RPL network and also tests UDP
 *  functionality
 * @{
 *
 * \file
 *  An example of a simple UDP echo server for the cc2538dk platform
 */
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"

#include "sls.h"	

/*---------------------------------------------------------------------------*/
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 120
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len;


/* SLS define */

static struct led_struct_t led_db;
//static struct led_struct_t *led_db_ptr = &led_db;

static struct gw_struct_t gw_db;
static struct net_struct_t net_db;
//static struct led_struct_t *gw_db_ptr = &gw_db;

static struct cmd_struct_t cmd;
//static struct cmd_struct_t *cmdPtr = &cmd;

static char str_reply[50];
static char str_cmd[10];
static char str_arg[10];
static char str_rx[MAX_PAYLOAD_LEN];
  
static 	radio_value_t aux;

/* define prototype of fucntion call */
static void get_radio_parameter(void);
static void init_default_parameters(void);
static void reset_parameters(void);
static 	char *p;

/*---------------------------------------------------------------------------*/
PROCESS(udp_echo_server_process, "UDP echo server process");
AUTOSTART_PROCESSES(&udp_echo_server_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
	//char *search = " ";
  memset(buf, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
    leds_on(LEDS_RED);
    len = uip_datalen();
    memcpy(buf, uip_appdata, len);
    PRINTF("Received from [");
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]:%u\n", UIP_HTONS(UIP_UDP_BUF->srcport));
		PRINTF("%u bytes DATA: %s",len, buf);
		
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    server_conn->rport = UIP_UDP_BUF->srcport;

		get_radio_parameter();
		reset_parameters();
		
		
		strcpy(str_rx,buf);
		if (SLS_CC2538DK_HW)
			sscanf(str_rx,"%s %s",str_cmd, str_arg);
		else {
			/* used for SKY */
    	//PRINTF("str_rx = %s", str_rx);
  		p = strtok (str_rx," ");  
			if (p != NULL) {
				strcpy(str_cmd,p);
    		p = strtok (NULL, " ,");
				if (p != NULL) {
					strcpy(str_arg,p);
				}			
			}
		}
		
	
		
		//PRINTF("CMD = %s ARG = %s\n",str_cmd, str_arg);
		
		if (strstr(str_cmd,SLS_LED_ON)!=NULL) {
			PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
			leds_on(LEDS_GREEN);
			sprintf(str_reply, "Replied = %s", str_rx);
			led_db.status = LED_ON;
		}
		else if (strstr(str_cmd, SLS_LED_OFF)!=NULL) {
			PRINTF ("Execute CMD = %s\n",SLS_LED_OFF);
			leds_off(LEDS_GREEN);
			sprintf(str_reply, "Replied = %s", str_rx);
			led_db.status = LED_OFF;
		}
		else if (strstr(str_cmd, SLS_LED_DIM)!=NULL) {
			PRINTF ("Execute CMD = %s to value %s",SLS_LED_DIM, str_arg);
			sprintf(str_reply, "Replied = %s\n", str_rx);
			led_db.status = LED_DIM;
			led_db.dim = atoi(str_arg);
		}
		else if (strstr(str_cmd, SLS_GET_LED_STATUS)!=NULL) {
			sprintf(str_reply, "Replied: id=%u;power=%u;temp=%d;dim=%u;status=0x%02X;\n", led_db.id,
					led_db.power,	led_db.temperature, led_db.dim, led_db.status);
		}		
		else if (strstr(str_cmd, SLS_GET_NW_STATUS)!=NULL) {
			sprintf(str_reply, "Replied: channel=%u;rssi=%ddBm;lqi=%u;tx_power=%ddBm;panid=0x%02X;\n", 
					net_db.channel, net_db.rssi, net_db.lqi, net_db.tx_power, net_db.panid);
		}		
		else {
			reset_parameters();
			sprintf(str_reply,"unknown cmd\n");
		}
		PRINTF("str_reply=%s\n",str_reply);
		

		/* echo back to sender */	
    PRINTF("Echo back to [");
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]:%u %u bytes\n", UIP_HTONS(UIP_UDP_BUF->srcport), sizeof(str_reply));
    //uip_udp_packet_send(server_conn, "Server-reply\n", sizeof("Server-reply"));
    uip_udp_packet_send(server_conn, str_reply, sizeof(str_reply));
    uip_create_unspecified(&server_conn->ripaddr);
    server_conn->rport = 0;
  }
  leds_off(LEDS_RED);
  return;
}

static void reset_parameters(void) {
	memset(&str_cmd[0], 0, sizeof(str_cmd));
	memset(&str_arg[0], 0, sizeof(str_arg));
	memset(&str_reply[0], 0, sizeof(str_reply));
}

/*---------------------------------------------------------------------------*/
static void get_radio_parameter() {
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &aux);
	net_db.channel = (unsigned int) aux;
	//printf("CH: %u ", (unsigned int) aux);	

  aux = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	net_db.rssi = (int8_t)aux;
  //printf("RSSI: %ddBm ", (int8_t)aux);

  aux = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
	net_db.lqi = aux;
  //printf("LQI: %u\n", aux);

  NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &aux);
	net_db.tx_power = aux;
  //printf("   Tx Power %3d dBm", aux);
}

/*---------------------------------------------------------------------------*/
static void init_default_parameters(void) {
	led_db.id			= 0x20;				//001-00000b
	led_db.panid 	=	SLS_PAN_ID;
	led_db.power	= 120;
	led_db.dim		= 80;
	led_db.status	= LED_ON; 

	gw_db.id			= 0x40;				//010-00000b
	gw_db.panid 	=	SLS_PAN_ID;
	gw_db.power		= 120;
	gw_db.status	= GW_CONNECTED; 

	cmd.sfd = 0x7E;
	cmd.seq	= 1;
	cmd.type = MSG_TYPE_REP;
	cmd.len = 7;

	net_db.panid = SLS_PAN_ID;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_echo_server_process, ev, data)
{

  PROCESS_BEGIN();

	PRINTF("Initialization....\n");
	init_default_parameters();
		
  PRINTF("Starting UDP echo server\n");

  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(3000));

  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
