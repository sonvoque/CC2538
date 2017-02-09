/*
|-------------------------------------------------------------------|
| HCMC University of Technology                                     |
| Telecommunications Departments                                    |
| Wireless Embedded Firmware for Smart Lighting System (SLS)        |
| Version: 1.0                                                      |
| Author: sonvq@hcmut.edu.vn                                        |
| Date: 01/2017                                                     |
|-------------------------------------------------------------------|
*/

#ifndef SLS_H_
#define SLS_H_

//#define IEEE802154_CONF_PANID		0xCAFE
#define SLS_PAN_ID	 IEEE802154_CONF_PANID


enum {
	SLS_NORMAL_PORT			= 3000,
	SLS_EMERGENCY_PORT		= 3001,
};

/*---------------------------------------------------------------------------*/
/* This is the UDP port used to receive data */
/* Response will be echoed back to DST port */
#define UDP_SERVER_LISTEN_PORT   	SLS_NORMAL_PORT
#define UDP_CLIENT_SEND_PORT   		SLS_EMERGENCY_PORT


/*
SLS_CC2538DK_HW = 1 : for compiling to CC2538dk
SLS_CC2538DK_HW = 0 : for compiling to SKY used in Cooja simulation
*/
#define SLS_CC2538DK_HW		1


#if (SLS_CC2538DK_HW)
#define SLS_USING_CC2538DK
#else
#define SLS_USING_SKY
#endif

#define	SFD 	0x7F

//redefine led
#define BLUE		LEDS_ORANGE
#define RED			LEDS_GREEN
#define GREEN		LEDS_BLUE


#define MAX_CMD_DATA_LEN	16
#define MAX_CMD_LEN	sizeof(cmd_struct_t)

typedef enum {false=0, true=1} bool;
#define DEFAULT_EMERGENCY_STATUS true

enum {	
	// msg type
	MSG_TYPE_REQ			= 0x01,
	MSG_TYPE_REP			= 0x02,
	MSG_TYPE_HELLO			= 0x03,
	MSG_TYPE_EMERGENCY		= 0x04,
};


enum {
	//command id
	CMD_LED_OFF				= 0x01,
	CMD_LED_ON				= 0x02,
	CMD_LED_DIM				= 0x03,
	CMD_GET_LED_STATUS 		= 0x04,
	CMD_GET_NW_STATUS 		= 0x05,
	CMD_GET_GW_STATUS 		= 0x06,
	CMD_TIMER_ON 			= 0x07,
	CMD_TIMER_OFF 			= 0x08,
	CMD_SET_APP_KEY			= 0x09,
	CMD_GET_APP_KEY			= 0x0A,
	CMD_LED_HELLO 			= 0x0B,
	CMD_LED_REBOOT			= 0x0C,
	CMD_REPAIR_ROUTE		= 0x0D,
	CMD_GW_HELLO			= 0x0E,
};

enum {
	//status of LED
	STATUS_LED_ON			= 0x01,
	STATUS_LED_OFF			= 0x02,
	STATUS_LED_DIM			= 0x03,	
	STATUS_LED_ERROR		= 0x04,
};


enum {	
	// node status
	NODE_CONNECTED			= 0x01,
	NODE_DISCONNECTED		= 0x02,
	NODE_POWER_ON			= 0x03,
	NODE_POWER_OFF			= 0x04,
	NODE_ERROR				= 0x05,
};

enum{
	//gateway status
	GW_CONNECTED			= 0x01,
	GW_DISCONNECTED			= 0x02,
	GW_POWER_ON				= 0x03,
	GW_POWER_OFF			= 0x04,
	GW_ERROR				= 0x05,
};

enum {
	// error code
	ERR_NORMAL				= 0x00,
	ERR_UNKNOWN_CMD			= 0x01,
	ERR_IN_HELLO_STATE		= 0x02,
	ERR_TIME_OUT			= 0x03,
	ERR_EMERGENCY			= 0x04,	
};

enum {
	//state machine
	STATE_HELLO				= 0x00,
	STATE_NORMAL			= 0x01,
	STATE_EMERGENCY			= 0x02,
};


/*---------------------------------------------------------------------------*/
struct led_struct_t {
	uint16_t	id;			/*000xxxxx xxxxxxxx */
	uint16_t  	panid;		/* default = 0xABCD */ 
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
	uint8_t		dim;	
	uint8_t		status;
};

/*---------------------------------------------------------------------------*/
//	used by gateway
struct gw_struct_t {
	uint16_t	id;			/*001xxxxx xxxxxxxx */
	uint16_t	panid;		
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
	uint8_t		status;
};

/*---------------------------------------------------------------------------*/
//	used in the future
struct env_struct_t {
	uint16_t	id;			/*010xxxxx xxxxxxxx */
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
	uint8_t			channel;	
	int8_t			rssi;
	int8_t			lqi;
	int8_t			tx_power;
	uint16_t		panid;
	uint16_t		node_addr;
	unsigned char	app_code[16];
};

/*---------------------------------------------------------------------------*/
//	sfd = 0x7F
//	seq: transaction id;
//	type: 	REQUEST/REPLY/HELLO
//	len: 	used for App node_id
//	cmd:	command id
//	err_code: code returned in REPLY, sender check this field to know the REQ status
//	arg[16]: data payload
struct cmd_struct_t {
	uint8_t  	sfd;
	uint8_t 	len;
	uint16_t 	seq;
	uint8_t		type;
	uint8_t		cmd;
	uint16_t	err_code;
	uint8_t 	arg[MAX_CMD_DATA_LEN];
};


typedef struct cmd_struct_t		cmd_struct_t;
typedef struct net_struct_t		net_struct_t;
typedef struct gw_struct_t		gw_struct_t;
typedef struct led_struct_t		led_struct_t;



#endif /* SLS_H_ */
