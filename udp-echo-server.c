/*
|-------------------------------------------------------------------|
| HCMC University of Technology                                     |
| Telecommunications Departments                                    |
| Wireless Embedded Firmware for Smart Lighting System (SLS)        |
| Version: 1.0                                                      |
| Author: sonvq@hcmut.edu.vn                                        |
| Date: 01/2017                                                     |
| HW support in ISM band: TelosB, CC2538, CC2530, CC1310, z1        |
|-------------------------------------------------------------------|*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

//#include <stdlib.h>
//#include <string.h>

#include "net/ip/uip-debug.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/watchdog.h"
#include "dev/uart1.h" 

#include "random.h"

//#include "dev/button-sensor.h"
//#include "lib/ringbuf.h"

#ifdef SLS_USING_CC2538DK
#include "dev/uart.h"
#endif

#include "sls.h"	
#include "aes.h"	


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

static 	cmd_struct_t cmd, reply, emer_reply;
//static 	cmd_struct_t *cmdPtr = &cmd;
static 	radio_value_t aux;
static	int	state;

#ifdef SLS_USING_CC2538DK
static  char rxbuf[MAX_PAYLOAD_LEN];		/* used for UART0 interface */
static 	int cmd_cnt;
#endif

/*define timers */
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static	struct	etimer	et;
//static 	struct 	ctimer ct;
//static	struct	rtimer	rt;
static	uint8_t	emergency_status;
static 	uint16_t timer_cnt =0;	// use for multiple timer events

/* define prototype of fucntion call */
//static 	void set_connection_address(uip_ipaddr_t *ipaddr);
static 	void get_radio_parameter(void);
static 	void init_default_parameters(void);
static 	void reset_parameters(void);

#ifdef 	SLS_USING_CC2538DK
static 	unsigned int uart0_send_bytes(const	unsigned  char *s, unsigned int len);
static 	int uart0_input_byte(unsigned char c);
//static 	unsigned int uart1_send_bytes(const	unsigned  char *s, unsigned int len);
//static 	int uart1_input_byte(unsigned char c);
#endif 

static 	void send_cmd_to_led_driver();
static	void process_hello_cmd(cmd_struct_t command);
static	void print_cmd_data(cmd_struct_t command);
static 	void send_reply (cmd_struct_t res);
static	void blink_led (unsigned char led);
static 	uint8_t is_cmd_of_nw (cmd_struct_t cmd);
static 	uint8_t is_cmd_of_led(cmd_struct_t cmd);
static 	void send_asyn_msg();
//static 	void float2Bytes(float val,uint8_t* bytes_array);
static 	void get_next_hop_addr();
static 	uint8_t is_connected();


static 	uint16_t 	hash( uint16_t a); 
static	void		gen_crc_for_cmd(cmd_struct_t *cmd);
static	uint8_t 	check_crc_for_cmd(cmd_struct_t *cmd);
static	uint16_t 	gen_crc16(uint8_t *data_p, unsigned short  length);
static	void 		encrypt_payload(cmd_struct_t *cmd, uint8_t* key);
static	void 		decrypt_payload(cmd_struct_t *cmd, uint8_t* key);
static void encrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv);
static void decrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv); 



/*---------------------------------------------------------------------------*/
PROCESS(udp_echo_server_process, "UDP echo server process");
AUTOSTART_PROCESSES(&udp_echo_server_process);



/*---------------------------------------------------------------------------*/
static uint16_t gen_crc16(uint8_t *data_p, unsigned short  length) {
    unsigned char i;
    unsigned int data;
    unsigned int crc = 0xffff;
    uint8_t len;
    len = length;

    if (len== 0)
        return (~crc);
    do    {
        for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
            else  crc >>= 1;
        }
    } while (--len);

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);
    return (crc);
}

/*---------------------------------------------------------------------------*/
static	void gen_crc_for_cmd(cmd_struct_t *cmd) {
    uint16_t crc16_check;
    uint8_t byte_arr[MAX_CMD_LEN-2];
    memcpy(&byte_arr, cmd, MAX_CMD_LEN-2);
    crc16_check = gen_crc16(byte_arr, MAX_CMD_LEN-2);
    cmd->crc = (uint16_t)crc16_check;
    PRINTF("\nGenerate CRC16 process ... done,  0x%04X \n", crc16_check);
}

//-------------------------------------------------------------------------------------------
static uint8_t check_crc_for_cmd(cmd_struct_t *cmd) {
    uint16_t crc16_check;
    uint8_t byte_arr[MAX_CMD_LEN-2];
    memcpy(&byte_arr, cmd, MAX_CMD_LEN-2);
    crc16_check = gen_crc16(byte_arr, MAX_CMD_LEN-2);
	//PRINTF("CRC-cal = 0x%04X; CRC-val =  0x%04X \n",crc16_check,cmd->crc);
    if (crc16_check == cmd->crc) {
        PRINTF("CRC16...matched\n");
        return TRUE;
    }
    else{
        PRINTF("CRC16 ...failed\n");
        return FALSE;        
    }
}


/*---------------------------------------------------------------------------*/
void phex_16(uint8_t* data_16) { // in chuoi hex 16 bytes
    unsigned char i;
    for(i = 0; i < 16; ++i)
        PRINTF("%.2x ", data_16[i]);
    PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
void phex_64(uint8_t* data_64) { // in chuoi hex 64 bytes
    unsigned char i;
    for(i = 0; i < 4; ++i) 
        phex_16(data_64 + (i*16));
    PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
// ma hoa 64 bytes
static void encrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv) { 
    uint8_t data_temp[MAX_CMD_LEN];

    memcpy(data_temp, data, MAX_CMD_LEN);
    PRINTF("\nData: \n");
    phex_64(data);

    AES128_CBC_encrypt_buffer(data_encrypted, data, 64, key, iv);

    PRINTF("\nData encrypted: \n");
    phex_64(data_encrypted);
}

/*---------------------------------------------------------------------------*/
static void  decrypt_cbc(uint8_t* data_decrypted, uint8_t* data_encrypted, uint8_t* key, uint8_t* iv)  {
    uint8_t data_temp[MAX_CMD_LEN];

    memcpy(data_temp, data_encrypted, MAX_CMD_LEN);
    printf("\nData encrypted: \n");
    phex_64(data_encrypted);

    AES128_CBC_decrypt_buffer(data_decrypted+0,  data_encrypted+0,  16, key, iv);
    AES128_CBC_decrypt_buffer(data_decrypted+16, data_encrypted+16, 16, 0, 0);
    AES128_CBC_decrypt_buffer(data_decrypted+32, data_encrypted+32, 16, 0, 0);
    AES128_CBC_decrypt_buffer(data_decrypted+48, data_encrypted+48, 16, 0, 0);

    PRINTF("Data decrypt: \n");
    phex_64(data_decrypted);
}


/*---------------------------------------------------------------------------*/
static uint16_t hash(uint16_t a) {
	uint32_t tem;
	tem =a;
	tem = (a+0x7ed55d16) + (tem<<12);
	tem = (a^0xc761c23c) ^ (tem>>19);
	tem = (a+0x165667b1) + (tem<<5);
	tem = (a+0xd3a2646c) ^ (tem<<9);
	tem = (a+0xfd7046c5) + (tem<<3);
	tem = (a^0xb55a4f09) ^ (tem>>16);
   return tem & 0xFFFF;
}


//-------------------------------------------------------------------------------------------
static void encrypt_payload(cmd_struct_t *cmd, uint8_t* key) {
#if (USING_AES_128==1)
    uint8_t payload[MAX_CMD_LEN];
    PRINTF(" - Encryption AES process ... done \n");
    memcpy(&payload, cmd, MAX_CMD_LEN);
    encrypt_cbc((uint8_t *)cmd, payload, key, iv);
#endif
}

//-------------------------------------------------------------------------------------------
static void decrypt_payload(cmd_struct_t *cmd, uint8_t* key) {
#if (USING_AES_128==1)
    decrypt_cbc((uint8_t *)cmd, (uint8_t *)cmd, key, iv);
    PRINTF(" - Decryption AES process ... done \n");
#endif
}

static void make_packet_for_node(cmd_struct_t *cmd, uint8_t* key, uint8_t encryption_en) {
	if (encryption_en==TRUE) {
		encrypt_payload(cmd, key);
	}
}

/*---------------------------------------------------------------------------*/
static void check_packet_for_node(cmd_struct_t *cmd, uint8_t* key, uint8_t encryption_en) {
	if (encryption_en==TRUE)
		decrypt_payload(cmd, key);
}


/*---------------------------------------------------------------------------*/
static void process_req_cmd(cmd_struct_t cmd){
	uint16_t rssi_sent, i;

	reply = cmd;
	reply.type =  MSG_TYPE_REP;
	reply.err_code = ERR_NORMAL;

	if (state==STATE_NORMAL) {
		switch (cmd.cmd) {
			case CMD_RF_HELLO:
				//leds_on(RED);
				//PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
				break;
			case CMD_RF_AUTHENTICATE:
				break;

			case CMD_RF_LED_ON:
				leds_on(BLUE);
				led_db.status = STATUS_LED_ON;
				//PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
				break;
			case CMD_RF_LED_OFF:
				leds_off(BLUE);
				led_db.status = STATUS_LED_OFF;
				//PRINTF ("Execute CMD = %d\n",CMD_LED_OFF);
				break;
			case CMD_RF_LED_DIM:
				leds_toggle(BLUE);
				led_db.status = STATUS_LED_DIM;
				led_db.dim = cmd.arg[0];			
				//PRINTF ("Execute CMD = %d; value %d\n",CMD_LED_DIM, led_db.dim);
				break;
			case CMD_GET_RF_STATUS:
				reply.arg[0] = led_db.id;
				reply.arg[1] = led_db.power;
				reply.arg[2] = led_db.temperature;
				reply.arg[3] = led_db.dim; 
				reply.arg[4] = led_db.status;
				break;
			/* network commands */				
			case CMD_RF_REBOOT:
				send_reply(reply);
				clock_delay(50000);
				watchdog_reboot();
				break;
			case CMD_GET_NW_STATUS:
				reply.arg[0] = net_db.channel;
				rssi_sent = net_db.rssi + 200;
				PRINTF("rssi_sent = %d\n", rssi_sent);
				reply.arg[1] = (rssi_sent & 0xFF00) >> 8;			
				reply.arg[2] = rssi_sent & 0xFF;					
				reply.arg[3] = net_db.lqi;
				reply.arg[4] = net_db.tx_power; 
				reply.arg[5] = (net_db.panid >> 8);
				reply.arg[6] = (net_db.panid) & 0xFF;		
				//insert next hop addr
				for (i=0; i<16; i++) {
					reply.arg[i+7] = net_db.next_hop[i];
				}				
				break;
			case CMD_GET_GW_STATUS:
				break;
			case CMD_GET_APP_KEY:
				memcpy(&reply.arg,&net_db.app_code,16);
				break;
			case CMD_RF_REPAIR_ROUTE:
				rpl_repair_root(RPL_DEFAULT_INSTANCE);
				break;
			default:
				reply.err_code = ERR_UNKNOWN_CMD;			
		}
	}
	else if (state==STATE_HELLO) {
		reply = cmd;	
		reply.err_code = ERR_IN_HELLO_STATE;
	}
	
}

/*---------------------------------------------------------------------------*/
static void process_hello_cmd(cmd_struct_t command){
	uint16_t rssi_sent, i;	
	uint32_t tem;

	get_radio_parameter();
	reply = command;
	reply.type =  MSG_TYPE_HELLO;
	reply.err_code = ERR_NORMAL;

	if (state==STATE_HELLO) {
		switch (command.cmd) {

			case CMD_RF_HELLO:
				leds_off(RED);
				break;

			case CMD_RF_AUTHENTICATE: 
				tem = (command.arg[0] << 8) | command.arg[1];
				net_db.challenge_code = tem & 0xFFFF;
				net_db.challenge_code_res = hash(net_db.challenge_code);
				PRINTF("challenge_code = 0x%04X \n", net_db.challenge_code);
				PRINTF("challenge_res = 0x%04X \n", net_db.challenge_code_res);

				reply.arg[0] = (net_db.challenge_code_res >> 8 ) & 0xFF;
				reply.arg[1] = (net_db.challenge_code_res) & 0xFF;

				reply.arg[4] = net_db.channel;
				rssi_sent = net_db.rssi + 200;
				PRINTF("rssi_sent = %d\n", rssi_sent);
				reply.arg[5] = (rssi_sent & 0xFF00) >> 8;			
				reply.arg[6] = rssi_sent & 0xFF;			
				reply.arg[7] = net_db.lqi;
				reply.arg[8] = net_db.tx_power; 
				reply.arg[9] = (net_db.panid >> 8);
				reply.arg[10] = (net_db.panid) & 0xFF;	
				//next hop
				for (i=0; i<16; i++) {
					reply.arg[i+11] = net_db.next_hop[i];
				}
				break;

			case CMD_SET_APP_KEY:
				state = STATE_NORMAL;
				leds_on(GREEN);
				memcpy(&net_db.app_code,&cmd.arg,16);
				net_db.authenticated = TRUE;
				break;
			default:
				reply.err_code = ERR_IN_HELLO_STATE;
				break;
		}	
	}
	else {
		switch (command.cmd) {
			case CMD_RF_HELLO:
				break;

			case CMD_RF_AUTHENTICATE: 
				tem = (command.arg[0] << 8) | command.arg[1];
				net_db.challenge_code = tem & 0xFFFF;
				net_db.challenge_code_res = hash(net_db.challenge_code);
				PRINTF("challenge_code = 0x%04X \n", net_db.challenge_code);
				PRINTF("challenge_res = 0x%04X \n", net_db.challenge_code_res);

				reply.arg[0] = (net_db.challenge_code_res >> 8 ) & 0xFF;
				reply.arg[1] = (net_db.challenge_code_res) & 0xFF;

				reply.arg[4] = net_db.channel;
				rssi_sent = net_db.rssi + 200;
				PRINTF("rssi_sent = %d\n", rssi_sent);
				reply.arg[5] = (rssi_sent & 0xFF00) >> 8;			
				reply.arg[6] = rssi_sent & 0xFF;			
				reply.arg[7] = net_db.lqi;
				reply.arg[8] = net_db.tx_power; 
				reply.arg[9] = (net_db.panid >> 8);
				reply.arg[10] = (net_db.panid) & 0xFF;	
				for (i=0; i<16; i++) {
					reply.arg[i+11] = net_db.next_hop[i];
				}
				break;
		}
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
static uint8_t is_cmd_of_nw (cmd_struct_t cmd) {
	return  (cmd.cmd==CMD_GET_RF_STATUS) ||
			(cmd.cmd==CMD_GET_NW_STATUS) ||
			(cmd.cmd==CMD_RF_HELLO) ||
			(cmd.cmd==CMD_RF_LED_ON) ||
			(cmd.cmd==CMD_RF_LED_OFF) ||
			(cmd.cmd==CMD_RF_LED_DIM) ||			
			(cmd.cmd==CMD_RF_TIMER_ON) ||			
			(cmd.cmd==CMD_RF_TIMER_OFF) ||			
			(cmd.cmd==CMD_SET_APP_KEY) ||		
			(cmd.cmd==CMD_GET_APP_KEY) ||	
			(cmd.cmd==CMD_RF_REBOOT) ||		
			(cmd.cmd==CMD_RF_REPAIR_ROUTE) ||
			(cmd.cmd==CMD_RF_AUTHENTICATE);		
}

static uint8_t is_cmd_of_led (cmd_struct_t cmd) {
	return !is_cmd_of_nw(cmd);
}

/*---------------------------------------------------------------------------*/
static void tcpip_handler(void)	{
	//char *search = " ";
	memset(buf, 0, MAX_PAYLOAD_LEN);
  	if(uip_newdata()) {
  		//blink_led(BLUE);
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
		
		//p = &buf;	cmdPtr = (cmd_struct_t *)(&buf);
		cmd = *(cmd_struct_t *)(&buf);
		check_packet_for_node(&cmd, net_db.app_code, FALSE);
		PRINTF("Rx Cmd-Struct: sfd=0x%02X; len=%d; seq=%d; type=0x%02X; cmd=0x%02X; err_code=0x%02X\n",cmd.sfd, cmd.len, 
										cmd.seq, cmd.type, cmd.cmd, cmd.err_code);
		print_cmd_data(cmd);
		check_crc_for_cmd(&cmd);
		
		reply = cmd;		
		/* get a REQ */
		if (is_cmd_of_nw(cmd)){
			if (cmd.type==MSG_TYPE_REQ) {
				process_req_cmd(cmd);
				reply.type = MSG_TYPE_REP;
			}
			/* get a HELLO */
			else if (cmd.type==MSG_TYPE_HELLO) {
				process_hello_cmd(cmd);	
				reply.type = MSG_TYPE_HELLO;
				//send_reply(reply);	
			}
			else if (cmd.type==MSG_TYPE_ASYNC) {
			}
			PRINTF("Reply for NW command: ");
			send_reply(reply);
		}	

		/* LED command */
		/* send command to LED-driver */
		//send_cmd_to_led_driver();
		if (is_cmd_of_led(cmd)){
			if (state==STATE_NORMAL) {
#ifdef SLS_USING_CC2538DK		
				send_cmd_to_led_driver();
#endif


#ifdef SLS_USING_SKY		
 				/* used for Cooja simulate the reply from LED driver */
				PRINTF("Reply for LED-driver command: ");
				send_reply(reply);
#endif
			}	
		}	
  	}

	return;
}

/*---------------------------------------------------------------------------*/
static void send_reply (cmd_struct_t res) {
	cmd_struct_t response;

	response = res;
	gen_crc_for_cmd(&response);
	make_packet_for_node(&response,net_db.app_code, FALSE);

	/* echo back to sender */	
	PRINTF("Reply to [");
	PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
	PRINTF("]:%u %u bytes\n", UIP_HTONS(UIP_UDP_BUF->srcport), sizeof(res));
	uip_udp_packet_send(server_conn, &response, sizeof(response));

	/* Restore server connection to allow data from any node */
	uip_create_unspecified(&server_conn->ripaddr);
	//memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
	//server_conn->rport = 0;
	blink_led(BLUE);
}
/*---------------------------------------------------------------------------*/
static void blink_led(unsigned char led) {
#ifdef SLS_USING_CC2538DK
	leds_on(led);
	clock_delay_usec((uint16_t)3000000);
	leds_off(led);
#endif	
}

/*---------------------------------------------------------------------------*/
#ifdef SLS_USING_CC2538DK
static int uart0_input_byte(unsigned char c) {
	if (c==SFD) {
		cmd_cnt=1;
		rxbuf[cmd_cnt-1]=c;
	}
	else {
		cmd_cnt++;
		rxbuf[cmd_cnt-1]=c;
		if (cmd_cnt==sizeof(cmd_struct_t)) {		/* got the full reply */
			cmd_cnt=0;
			emer_reply = *((cmd_struct_t *)(&rxbuf));
			PRINTF("Get cmd from LED-driver %s \n",rxbuf);
			/* processing emergency reply */
			if (emer_reply.type == MSG_TYPE_ASYNC) {
				emergency_status = TRUE;
				send_asyn_msg();
			}
			else {	//send reply
				reply = emer_reply;
				send_reply(reply);		/* got a Reply from LED-driver, send to orginal node */
				//blink_led(BLUE);
			}
		}
	}
	return 1;
}

/*---------------------------------------------------------------------------*/
static unsigned int uart0_send_bytes(const	unsigned  char *s, unsigned int len) {
	unsigned int i;
	for (i = 0; i<len; i++) {
		uart_write_byte(0, (uint8_t) (*(s+i)));
   	}   
   return 1;
}
#endif


/*---------------------------------------------------------------------------*/
static void send_cmd_to_led_driver() {
#ifdef SLS_USING_CC2538DK
	uart0_send_bytes((const unsigned  char *)(&cmd), sizeof(cmd));	
#endif
}

/*---------------------------------------------------------------------------*/
static void reset_parameters(void) {
	memset(&reply, 0, sizeof(reply));
}

/*---------------------------------------------------------------------------*/
static void get_radio_parameter(void) {
#ifndef SLS_USING_CC2530DK
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &aux);
	net_db.channel = (unsigned int) aux;
	PRINTF("CH: %u, ", (unsigned int) aux);	

 	aux = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	net_db.rssi = (int8_t)aux;
 	PRINTF("RSSI: %ddBm, ", net_db.rssi);

	aux = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
	net_db.lqi = aux;
 	PRINTF("LQI: %u, ", aux);

	NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &aux);
	net_db.tx_power = aux;
 	PRINTF("Tx Power %3d dBm\n", aux);
#endif 	
}




/*---------------------------------------------------------------------------*/
static void init_default_parameters(void) {
	state = STATE_HELLO;
	led_db.id		= LED_ID_MASK;				
	led_db.panid 	= SLS_PAN_ID;
	led_db.power	= 120;
	led_db.dim		= 80;
	led_db.status	= STATUS_LED_ON; 
	led_db.temperature = 37;

	gw_db.id		= GW_ID_MASK;				
	gw_db.panid 	= SLS_PAN_ID;
	gw_db.power		= 150;
	gw_db.status	= GW_CONNECTED; 

	cmd.sfd  = SFD;
	cmd.seq	 = 0;
	cmd.type = MSG_TYPE_REP;
	cmd.len  = sizeof(cmd_struct_t);

	net_db.panid 	= SLS_PAN_ID;
	net_db.connected = FALSE;
	net_db.lost_connection_cnt = 0;
	net_db.authenticated = FALSE;

	emergency_status = DEFAULT_EMERGENCY_STATUS;


	// init UART0-1
#ifdef SLS_USING_CC2538DK
	uart_init(0); 		
 	uart_set_input(0,uart0_input_byte);
#endif

}

/*---------------------------------------------------------------------------*/
static void set_connection_address(uip_ipaddr_t *ipaddr) {
  // change this IP address depending on the node that runs the server!
  uip_ip6addr(ipaddr, 0xaaaa,0x0000,0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001);
}



/*---------------------------------------------------------------------------*/
static void send_asyn_msg(){
#ifdef SLS_USING_SKY
	int i;
	//emer_reply = reply;
	for (i=0; i<MAX_CMD_DATA_LEN; i++)
		emer_reply.arg[i] = MAX_CMD_DATA_LEN-i-1;
#endif

	//sprintf(buf, "Emergency msg %d from the client", ++seq_id);
	emer_reply.type = MSG_TYPE_ASYNC;
	emer_reply.err_code = ERR_NORMAL;
	make_packet_for_node(&emer_reply, net_db.app_code, FALSE);
	uip_udp_packet_send(client_conn, &emer_reply, sizeof(emer_reply));
	
	/* debug only*/	
	PRINTF("Client sending ASYNC msg to: ");
	PRINT6ADDR(&client_conn->ripaddr);
	PRINTF(" (msg: %s)\n", (char*)&emer_reply);
}

/*
static void ctimer_callback(void *ptr) {
	//uint32_t *ctimer_ticks = ptr;
 	//PRINTF("ctimer fired now: \t%ld\n", *ctimer_ticks);	
	if (state==STATE_NORMAL) {	
		if (emergency_status==TRUE) {	
			clock_delay(random_rand()%100);
			emer_reply.err_code = ERR_NORMAL;
			send_asyn_msg();
			//emergency_status = FALSE;		// send once or continuously
		}
	}
}
*/


/*---------------------------------------------------------------------------*/
static void et_timeout_hanler(){
	timer_cnt++;
	// count in 300s
	if (timer_cnt==10)
		timer_cnt =0;

	/* 90s  send an async msg*/
	if ((timer_cnt % 3)==0) {
		if ((state==STATE_NORMAL) && (emergency_status==TRUE)) {	
			clock_delay(random_rand()%100);
			emer_reply.err_code = ERR_NORMAL;
			send_asyn_msg();
			emergency_status = FALSE;		// send once or continuously
		}
	}

	/* if joined network, signal to LED RED, check join/disjoin in 30s */
	if (is_connected()==TRUE) {
	    //PRINTF("joined the network \n");
	    leds_on(RED);
		get_next_hop_addr();
    	net_db.connected = TRUE;
	    net_db.lost_connection_cnt=0;
	    if (net_db.authenticated==FALSE) {
			emer_reply.cmd = ASYNC_MSG_JOINED;
			emer_reply.err_code = ERR_NORMAL;
			send_asyn_msg();
			//PRINTF("Send authenticated msg \n");
	    }
    }	
    else {
	    //PRINTF("disjoined the network \n");
	    leds_off(RED);   
	    net_db.lost_connection_cnt++; 	
	    // if lost connection in 150s then confirm connected = FALSE
	    if (net_db.lost_connection_cnt==5) {	
	    	net_db.connected = FALSE;
	    	net_db.authenticated= FALSE;
	    	net_db.lost_connection_cnt=0;
	    }
    }
}	


/*---------------------------------------------------------------------------*/
static uint8_t is_connected() {
    rpl_dag_t *dag = rpl_get_any_dag();
    if(dag && dag->instance->def_route)
    	return TRUE;
    else
    	return FALSE;
}

/*---------------------------------------------------------------------------*/
static void get_next_hop_addr(){
#if UIP_CONF_IPV6_RPL
	//int i;
    rpl_dag_t *dag = rpl_get_any_dag();
    if(dag && dag->instance->def_route) {
	    memcpy(&net_db.next_hop, &dag->instance->def_route->ipaddr, sizeof(uip_ipaddr_t));
	    //PRINTF("Next_hop addr [%d] = ", sizeof(uip_ipaddr_t));
	    //for (i=0; i<sizeof(uip_ipaddr_t);i++) {
	    //	PRINTF("0x%02X ", net_db.next_hop[i]);
	    //}
	    //PRINTF("\n");
    } 
#endif        
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_echo_server_process, ev, data) {

	PROCESS_BEGIN();

  	/* Variables inside a thread should be declared as static */
  	//static uint32_t ticks = 0;

  	NETSTACK_MAC.off(1);
	init_default_parameters();

	/* setup server connection for querry */
	server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  	if(server_conn == NULL) {
    	PROCESS_EXIT();
  	}
  	udp_bind(server_conn, UIP_HTONS(SLS_NORMAL_PORT));

  	/* setup client connection for asyn message */
  	set_connection_address(&server_ipaddr);
	client_conn = udp_new(&server_ipaddr, UIP_HTONS(SLS_EMERGENCY_PORT), NULL);

	/* timer for events */
	etimer_set(&et, CLOCK_SECOND*30);

 	while(1) {
    	PROCESS_YIELD();
    	if(ev == tcpip_event) {
    		get_next_hop_addr();
      		tcpip_handler();
    	}
    	
    	/* ev timeout */
    	else if (ev==PROCESS_EVENT_TIMER) {
    		et_timeout_hanler();
    		etimer_restart(&et);
    	}
 	
 		/* The callback timer triggers a given function when the timer expires.  It
   		* takes as parameters the ctimer structure, the time period, a function to
   		* invoke when it expires, and alternatively a pointer to data
   		*/
		//ticks++;
  		//ctimer_set(&ct, CLOCK_SECOND*40, ctimer_callback, &ticks);  
  	}

	PROCESS_END();
}