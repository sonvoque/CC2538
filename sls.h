/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \author Antonio Lignan <alinan@zolertia.com>
 */

#ifndef SLS_
#define SLS_

#define SLS_PAN_ID	 IEEE802154_CONF_PANID

/*---------------------------------------------------------------------------*/
/* This is the UDP port used to receive data */
/* Response will be echoed back to DST port */
#define UDP_SERVER_LISTEN_PORT   3000

#define SLS_LED_ON	"led_on"
#define SLS_LED_OFF	"led_off"
#define SLS_LED_DIM	"led_dim"

#define SLS_GET_LED_STATUS	"get_led_status"
#define SLS_GET_GW_STATUS		"get_gw_status"
#define SLS_GET_NW_STATUS		"get_nw_status"

#define SLS_CC2538DK_HW		0

enum {
	LED_OFF							= 0x01,
	LED_ON							= 0x02,
	LED_DIM							= 0x03,
	GW_CONNECTED				= 0x04,
	GW_DISCONNECTED			= 0x05,
	MSG_TYPE_REQ				= 0x06,
	MSG_TYPE_REP				= 0x07,
	NODE_CONNECTED			= 0x08,
	NODE_DISCONNECTED		= 0x09,
};



/*---------------------------------------------------------------------------*/
struct led_struct_t {
	uint16_t	id;
	uint16_t  panid;
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
	uint8_t		dim;	
	uint8_t		status;
};

struct gw_struct_t {
	uint16_t	id;
	uint16_t	panid;		
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
	uint8_t		status;
};

struct env_struct_t {
	uint16_t	id;
	uint16_t	panid;		
	uint16_t	temp;
	uint16_t	humidity;
	uint16_t	light;
	uint16_t	pir;
	uint16_t	rain;
	uint8_t		status;
};
/* This data structure is used to store the packet content (payload) */

struct net_struct_t {
	radio_value_t radio;
	uint8_t				channel;	
	int8_t				rssi;
	int8_t				tx_power;
	int8_t				lqi;
	uint16_t			panid;
};

struct cmd_struct_t {
	uint8_t  	sfd;
	uint16_t 	seq;
	uint8_t		type;
	uint8_t 	len;
	uint8_t		cmd;
	uint8_t 	arg1;
	uint8_t		agr2;
	uint8_t		arg3;
	uint8_t		err_code;
	uint16_t	crc;		
};


/*---------------------------------------------------------------------------*/
#endif /* __TEST_EXAMPLE__ */
