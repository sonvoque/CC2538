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

#include <stdlib.h>
#include <string.h>

#include "net/ip/uip-debug.h"
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"
#include "dev/uart1.h"
#include "lib/ringbuf.h"


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

static 	led_struct_t led_db;
//static struct led_struct_t *led_db_ptr = &led_db;

static 	gw_struct_t gw_db;
static 	net_struct_t net_db;
//static struct led_struct_t *gw_db_ptr = &gw_db;

static 	cmd_struct_t cmd, reply;
static 	cmd_struct_t *cmdPtr = &cmd;

//static 	char str_reply[80];
//static 	char str_cmd[10];
//static 	char str_arg[10];
//static 	char str_rx[MAX_PAYLOAD_LEN];
  
static 	radio_value_t aux;

//static 	char *p;
static  char rxbuf[MAX_PAYLOAD_LEN];
static 	int cmd_cnt;


static	int	state;
//static	struct	etimer	et;
//static  uip_ipaddr_t ipaddr;

/* define prototype of fucntion call */
static 	void get_radio_parameter(void);
static 	void init_default_parameters(void);
static 	void reset_parameters(void);
static 	unsigned int uart1_send_bytes(const	unsigned  char *s, unsigned int len);
static 	unsigned int send_cmd_to_led();
static 	int uart1_input_byte(unsigned char c);
//static 	void set_connection_address(uip_ipaddr_t *ipaddr);
static	void process_hello_cmd(cmd_struct_t command);
static	void print_cmd_data(cmd_struct_t command);
static void send_reply ();

/*---------------------------------------------------------------------------*/
PROCESS(udp_echo_server_process, "UDP echo server process");
AUTOSTART_PROCESSES(&udp_echo_server_process);


/*---------------------------------------------------------------------------*/
static void send_reply () {
	/* echo back to sender */	
	//PRINTF("Reply to [");
	//PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
	//PRINTF("]:%u %u bytes\n", UIP_HTONS(UIP_UDP_BUF->srcport), sizeof(res));
	uip_udp_packet_send(server_conn, &reply, sizeof(reply));
	uip_create_unspecified(&server_conn->ripaddr);
	server_conn->rport = 0;
}

/*---------------------------------------------------------------------------*/
static void process_req_cmd(cmd_struct_t cmd){

	reply = cmd;
	reply.type =  MSG_TYPE_REP;
	reply.err_code = ERR_NORMAL;

	if (state==STATE_NORMAL) {
		switch (cmd.cmd) {
			case CMD_LED_ON:
				leds_on(RED);
				led_db.status = STATUS_LED_ON;
				//PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
				break;
			case CMD_LED_OFF:
				leds_off(RED);
				led_db.status = STATUS_LED_OFF;
				//PRINTF ("Execute CMD = %s\n",SLS_LED_OFF);
				break;
			case CMD_LED_DIM:
				leds_toggle(GREEN);
				led_db.status = STATUS_LED_DIM;
				led_db.dim = cmd.arg[0];			
				//PRINTF ("Execute CMD = %s; value %d\n",SLS_LED_DIM, led_db.dim);
				break;
			case CMD_LED_REBOOT:
				send_reply();
				clock_delay(100);
				watchdog_reboot();
				break;
			case CMD_GET_LED_STATUS:
				reply.arg[0] = led_db.id;
				reply.arg[1] = led_db.power;
				reply.arg[2] = led_db.temperature;
				reply.arg[3] = led_db.dim; 
				reply.arg[4] = led_db.status;
				break;
			case CMD_GET_NW_STATUS:
				reply.arg[0] = net_db.channel;
				reply.arg[1] = net_db.rssi;
				reply.arg[2] = net_db.lqi;
				reply.arg[3] = net_db.tx_power; 
				reply.arg[4] = net_db.panid;
				break;
			case CMD_GET_GW_STATUS:
				break;

			case CMD_GET_APP_KEY:
				break;
			default:
				reply.err_code = ERR_UNKNOWN_CMD;			
		}
	}
	else if (state==STATE_HELLO) {
		//PRINTF("in HELLO state: no process REQ cmd\n");	
		switch (cmd.cmd) {
			case CMD_LED_REBOOT:
				send_reply();
				clock_delay(100);
				watchdog_reboot();
				break;
			default:
				break;
		}		
		reply = cmd;	
		reply.err_code = ERR_IN_HELLO_STATE;
	}

	//send_reply(reply);
}

/*---------------------------------------------------------------------------*/
static void process_hello_cmd(cmd_struct_t command){
	reply = command;
	reply.type =  MSG_TYPE_HELLO;
	reply.err_code = ERR_NORMAL;

	if (state==STATE_HELLO) {
		switch (command.cmd) {
			case CMD_LED_HELLO:
				state = STATE_HELLO;
				//PRINTF("State: %d \n",state);			
				break;
			case CMD_SET_APP_KEY:
				state = STATE_NORMAL;
				//PRINTF("Set APP-KEY: ");
				//print_cmd_data(command);
				//PRINTF("State: %d \n",state);			
				break;
			default:
				reply.err_code = ERR_IN_HELLO_STATE;
				break;
		}	
	//send_reply(reply);
	}				
}

/*---------------------------------------------------------------------------*/
static void print_cmd_data(cmd_struct_t command) {
	uint8_t i;	
  	PRINTF("data = [");
	for (i=0;i<MAX_CMD_DATA_LEN;i++) 
    	PRINTF("0x%02X,",command.arg[i]);
  	PRINTF("]\n");
}

/*---------------------------------------------------------------------------*/
static void tcpip_handler(void)	{
	//char *search = " ";
	memset(buf, 0, MAX_PAYLOAD_LEN);
  	if(uip_newdata()) {
    	leds_on(LEDS_RED);
    	len = uip_datalen();
    	memcpy(buf, uip_appdata, len);
    	//PRINTF("Received from [");
    	//PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    	//PRINTF("]:%u ", UIP_HTONS(UIP_UDP_BUF->srcport));
		//PRINTF("%u bytes DATA\n",len);
		
    	uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    	server_conn->rport = UIP_UDP_BUF->srcport;

		get_radio_parameter();
		reset_parameters();
		
		//p = &buf;	
		//cmdPtr = (cmd_struct_t *)(&buf);
		cmd = *(cmd_struct_t *)(&buf);
		PRINTF("Rx Cmd-Struct: sfd=0x%02X; len=%d; seq=%d; type=0x%02X; cmd=0x%02X; err_code=0x%02X\n",cmd.sfd, cmd.len, 
										cmd.seq, cmd.type, cmd.cmd, cmd.err_code);
		print_cmd_data(cmd);
		
		reply = cmd;		
		if (cmd.type==MSG_TYPE_REQ) {
			process_req_cmd(cmd);
			reply.type = MSG_TYPE_REP;
		}
		else if (cmd.type==MSG_TYPE_HELLO) {
			process_hello_cmd(cmd);	
			reply.type = MSG_TYPE_HELLO;
		}
		else if (cmd.type==MSG_TYPE_EMERGENCY) {
		}

		//prepare reply and response to sender
		send_reply();


		/* send command to LED-driver */
		if (SLS_CC2538DK_HW) {		
			send_cmd_to_led();
		}	
  	}
	leds_off(LEDS_RED);
	return;
}


/*---------------------------------------------------------------------------*/
static int uart1_input_byte(unsigned char c)
{
  	//static uint8_t overflow = 0 ;
	//printf("uart1 routine start..\n");

	if (c==SFD) {
		rxbuf[cmd_cnt]=c;
		cmd_cnt=1;
	}
	else {
		rxbuf[cmd_cnt]=c;
		cmd_cnt++;
		if (cmd_cnt==sizeof(cmd_struct_t)) {
			cmd_cnt=0;
			PRINTF("Get cmd from LED-driver %s \n",rxbuf);
			leds_on(BLUE);
		}
	}
	return 1;
}

/*---------------------------------------------------------------------------*/
static unsigned int uart1_send_bytes(const	unsigned  char *s, unsigned int len) {
	unsigned int i = 0;
	for (i=0;i<len;i++) {
		uart_write_byte(1, *(s+i));
   	}
	//PRINTF("UART1 send %d bytes\n",len);
   return i;
}

/*---------------------------------------------------------------------------*/
static unsigned int send_cmd_to_led() {
	uart1_send_bytes((unsigned  char *)(&cmd), sizeof(cmd_struct_t));
	return 1;
}

/*---------------------------------------------------------------------------*/
static void reset_parameters(void) {
	//memset(&str_cmd[0], 0, sizeof(str_cmd));
	//memset(&str_arg[0], 0, sizeof(str_arg));
	memset(&reply, 0, sizeof(reply));
}

/*---------------------------------------------------------------------------*/
static void get_radio_parameter(void) {
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

	state = STATE_HELLO;

	led_db.id		= 0x20;				//001-0 0000b
	led_db.panid 	= SLS_PAN_ID;
	led_db.power	= 120;
	led_db.dim		= 80;
	led_db.status	= STATUS_LED_ON; 
	led_db.temperature = 37;

	gw_db.id		= 0x40;				//010-0 0000b
	gw_db.panid 	= SLS_PAN_ID;
	gw_db.power		= 120;
	gw_db.status	= GW_CONNECTED; 

	cmd.sfd  = SFD;
	cmd.seq	 = 1;
	cmd.type = MSG_TYPE_REP;
	cmd.len  = 7;

	net_db.panid 	= SLS_PAN_ID;

	// init UART1 
	if (SLS_CC2538DK_HW) {
		uart_init(UART1_CONF_UART); 
 		uart_set_input(UART1_CONF_UART,uart1_input_byte);
 	}	
}

/*---------------------------------------------------------------------------
void process_hello_state(void){
	PRINTF("HELLO STATE...\n");
	set_connection_address(&ipaddr);
  	server_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);	

    PRINTF("Echo HELLO to [");
    PRINT6ADDR(&ipaddr);
    PRINTF("]:%u %u bytes\n", 3000, sizeof(reply));
    uip_udp_packet_send(server_conn, "Hello", strlen("Hello"));
    
    //state = NORMAL_STATE;
}

void process_emergency_state(void){
	//PRINTF("EMERGENCY STATE...\n");
}

void process_normal_state(void){
	//PRINTF("NORMAL STATE...\n");
}
*/

/*
void timer_event_handler(void){
	etimer_set(&et, 10*CLOCK_SECOND);
	
 	if (state==HELLO_STATE) {
 		process_hello_state();	
 	}
 	else if (state==EMERGENCY_STATE) {
 		process_emergency_state();
 	}
 	else if (state==NORMAL_STATE) {
 		process_normal_state();
 	}
 	
}
*/

/*
static void set_connection_address(uip_ipaddr_t *ipaddr) {
  // server is the tunslip6 addr: [aaaa::1]
  uip_ip6addr(ipaddr, 0xaaaa,0x0000,0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001);
}
*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_echo_server_process, ev, data) {

	PROCESS_BEGIN();

	//Init timer
  	//etimer_set(&et, 10*CLOCK_SECOND);

	init_default_parameters();
		
	server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
	udp_bind(server_conn, UIP_HTONS(3000));
	//PRINTF("Starting SLS UDP-echo-server: listen port 3000, TTL=%u\n",server_conn->ttl);

	while(1) {
    	PROCESS_YIELD();

		//if (ev == PROCESS_EVENT_TIMER) {
		//	timer_event_handler();
 		//}	
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
		/*
		strcpy(str_rx,buf);
		if (SLS_CC2538DK_HW)
			sscanf(str_rx,"%s %s",str_cmd, str_arg);
		else {
    		p = strtok (str_rx," ");  
			if (p != NULL) {
				strcpy(str_cmd,p);
    			p = strtok (NULL, " ,");
				if (p != NULL) {
					strcpy(str_arg,p);
				}			
			}
		}
		
		//PRINTF("str_rx = %s", str_rx); 		
		//PRINTF("CMD = %s ARG = %s\n",str_cmd, str_arg);		
		
		if (strstr(str_cmd,SLS_LED_ON)!=NULL) {
			PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
			leds_on(LEDS_GREEN);
			sprintf(str_reply, "Replied=%s", str_rx);
			led_db.status = LED_ON;
		}
		else if (strstr(str_cmd, SLS_LED_OFF)!=NULL) {
			PRINTF ("Execute CMD = %s\n",SLS_LED_OFF);
			leds_off(LEDS_GREEN);
			sprintf(str_reply, "Replied=%s", str_rx);
			led_db.status = LED_OFF;
		}
		else if (strstr(str_cmd, SLS_LED_DIM)!=NULL) {
			PRINTF ("Execute CMD = %s; value %s\n",SLS_LED_DIM, str_arg);
			leds_toggle(LEDS_BLUE);
			sprintf(str_reply, "Replied=%s", str_rx);
			led_db.status = LED_DIM;
			led_db.dim = atoi(str_arg);
		}
		else if (strstr(str_cmd, SLS_GET_LED_STATUS)!=NULL) {
			sprintf(str_reply, "Replied:id=%u;power=%u;temp=%d;dim=%u;status=0x%02X;", led_db.id,
					led_db.power,	led_db.temperature, led_db.dim, led_db.status);
		}		
		else if (strstr(str_cmd, SLS_GET_NW_STATUS)!=NULL) {
			sprintf(str_reply, "Replied:channel=%u;rssi=%ddBm;lqi=%u;tx_power=%ddBm;panid=0x%02X;", 
					net_db.channel, net_db.rssi, net_db.lqi, net_db.tx_power, net_db.panid);
		}		
		else {
			reset_parameters();
			sprintf(str_reply,"unknown cmd");
		}
		//PRINTF("str_reply=%s\n",str_reply);
		*/