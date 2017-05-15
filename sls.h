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

//#define IEEE802154_CONF_PANID		0xDCBA
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
SLS_CC2538DK_HW = 2 : for compiling to CC2530DK  */
#define SLS_CC2538DK_HW		1

#if (SLS_CC2538DK_HW==0)
#define SLS_USING_SKY
#endif
#if (SLS_CC2538DK_HW==1)
#define SLS_USING_CC2538DK
#endif
#if (SLS_CC2538DK_HW==2)
#define SLS_USING_CC2530DK
#endif


#define	SFD 	0x7F		/* Start of SLS frame Delimitter */

//redefine leds
#define BLUE		LEDS_ORANGE
#define RED			LEDS_GREEN
#define GREEN		LEDS_BLUE

#define GW_ID_MASK		0x0000
#define LED_ID_MASK		0x1000
#define METTER_ID_MASK	0x2000
#define ENV_ID_MASK		0x4000


#define MAX_CMD_DATA_LEN	56	
#define MAX_CMD_LEN	sizeof(cmd_struct_t)

typedef enum {false=0, true=1} bool;

#define DEFAULT_EMERGENCY_STATUS true
#define EMERGENCY_TIME  30

enum {	
	// msg type
	MSG_TYPE_REQ			= 0x01,
	MSG_TYPE_REP			= 0x02,
	MSG_TYPE_HELLO			= 0x03,
	MSG_TYPE_EMERGENCY		= 0x04,
};


enum {
	//command id
	CMD_GET_RF_STATUS 		= 0xFF,
	CMD_GET_NW_STATUS 		= 0xFE,

	CMD_GET_GW_STATUS 		= 0xFD,
	CMD_GW_HELLO			= 0xFC,
	CMD_GW_SHUTDOWN			= 0xFB,
	CMD_GW_TURN_ON_ALL		= 0xFA,
	CMD_GW_TURN_OFF_ALL		= 0xF9,
	CMD_GW_DIM_ALL			= 0xF8,
	

	CMD_RF_LED_OFF			= 0xF7,
	CMD_RF_LED_ON			= 0xF6,
	CMD_RF_LED_DIM			= 0xF5,
	CMD_RF_HELLO 			= 0xF4,
	CMD_RF_TIMER_ON 		= 0xF3,
	CMD_RF_TIMER_OFF 		= 0xF2,
	CMD_SET_APP_KEY			= 0xF1,
	CMD_GET_APP_KEY			= 0xF0,
	CMD_RF_REBOOT			= 0xEF,
	CMD_RF_REPAIR_ROUTE		= 0xEE,

	CMD_GW_SET_TIMEOUT		= 0xED,
	CMD_GW_MULTICAST_CMD	= 0xEC,
	CMD_GW_BROADCAST_CMD	= 0xEB,
	CMD_GW_GET_EMER_INFO	= 0xEA,
	CMD_GW_TURN_ON_ODD		= 0xE9,
	CMD_GW_TURN_ON_EVEN		= 0xE8,
	CMD_GW_TURN_OFF_ODD		= 0xE7,
	CMD_GW_TURN_OFF_EVEN	= 0xE6,
	CMD_GW_DIM_ODD			= 0xE5,
	CMD_GW_DIM_EVEN			= 0xE4,


	/* for LED-driver */
	CMD_LED_PING			= 0x01,
	CMD_LED_SET_RTC			= 0x02,
	CMD_LED_RTC 			= 0x03,
	CMD_LED_MODE			= 0x04,
	CMD_LED_GET_STATUS      = 0x15,
	CMD_LED_EMERGENCY		= 0x06,
	CMD_LED_DIM				= 0x08,
	CMD_LED_SET_ID			= 0x0E,
	CMD_LED_GET_ID			= 0x0F,
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
	ERR_BROADCAST_CMD		= 0x05,	
	ERR_MULTICAST_CMD		= 0x06,
	ERR_RF_LOST_POWER		= 0x07,
	ERR_GW_LOST_POWER		= 0x08,
};

enum {
	//state machine
	STATE_HELLO				= 0x00,
	STATE_NORMAL			= 0x01,
	STATE_EMERGENCY			= 0x02,
};

/*---------------------------------------------------------------------------*/
//	used by gateway
struct gw_struct_t {
	uint16_t	id;			/*0000xxxx xxxxxxxx */
	uint16_t	panid;		
	uint8_t		status;
	/* data of device */
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
};

/*---------------------------------------------------------------------------*/
struct led_struct_t {
	uint16_t	id;			/*0001xxxx xxxxxxxx */
	uint16_t  	panid;						
	uint8_t		status;
	/* data of device */
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	power;
	uint16_t	temperature;
	uint16_t	lux;
	uint8_t		dim;	
};

struct power_metter {
	uint16_t	id;			/*0010xxxx xxxxxxxx */
	uint16_t  	panid;		
	uint8_t		status;	
	/* data of device */
	uint16_t	voltage;
	uint16_t	current;
	uint16_t	voltage_1;
	uint16_t	voltage_2;
	uint16_t	voltage_3;
	uint16_t	current_1;
	uint16_t	current_2;
	uint16_t	current_3;
};



/*---------------------------------------------------------------------------*/
//	used in the future
struct env_struct_t {
	uint16_t	id;			/*0011xxxx xxxxxxxx */
	uint16_t	panid;		
	uint8_t		status;
	/* data of device */
	uint16_t	temp;
	uint16_t	humidity;
	uint16_t	light;
	uint16_t	pir;
	uint16_t	rain;
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

union float_byte_convert {
    float f;
    uint8_t bytes[4];
};

typedef struct cmd_struct_t		cmd_struct_t;
typedef struct net_struct_t		net_struct_t;
typedef struct gw_struct_t		gw_struct_t;
typedef struct led_struct_t		led_struct_t;



#endif /* SLS_H_ */
