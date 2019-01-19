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
SLS_USING_HW = 0 : for compiling to SKY/Z1 used in Cooja simulation
SLS_USING_HW = 1 : for compiling to CC2538dk: 2.4Ghz
SLS_USING_HW = 2 : for compiling to CC2530DK: 2.4Ghz  
SLS_USING_HW = 3 : for compiling to CC1310, CC1350: Sub-1GHz  */
#define SLS_USING_HW		0

#if (SLS_USING_HW==0)
#define SLS_USING_SKY
#endif
#if (SLS_USING_HW==1)
#define SLS_USING_CC2538DK
#endif
#if (SLS_USING_HW==2)
#define SLS_USING_CC2530DK
#endif
#if (SLS_USING_HW==3)
#define SLS_USING_CC13xx
#endif
#if (SLS_USING_HW==4)
#define SLS_USING_Z1
#endif


//redefine leds
#ifdef SLS_USING_CC2538DK
#define RED			LEDS_ORANGE
#define GREEN		LEDS_BLUE
#define BLUE		LEDS_GREEN
#endif

#ifdef SLS_USING_SKY
#define RED			LEDS_RED
#define BLUE		LEDS_BLUE
#define GREEN		LEDS_GREEN
#endif

#ifdef SLS_USING_CC13xx /* launchpad board */
#define RED			LEDS_RED
#define BLUE		LEDS_BLUE
#define GREEN		LEDS_GREEN
#endif

#ifdef SLS_USING_Z1 /* launchpad board */
#define RED			LEDS_RED
#define BLUE		LEDS_BLUE
#define GREEN		LEDS_GREEN
#endif


#define	SFD 	0x7F		/* Start of SLS frame Delimitter */

#define GW_ID_MASK		0x0000
#define LED_ID_MASK		0x1000
#define METTER_ID_MASK	0x2000
#define ENV_ID_MASK		0x4000


#define MAX_CMD_DATA_LEN	54	
#define MAX_CMD_LEN			sizeof(cmd_struct_t)

enum {FALSE=0, TRUE=1,};

#define CC2538DK_HAS_SENSOR  FALSE

#define DEFAULT_EMERGENCY_STATUS TRUE
#define EMERGENCY_TIME  30 		//seconds


#define SLS_USING_AES_128		0  //set this to enable AES-128 encryption
#define POLY 0x8408
static uint8_t iv[16]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
                           0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };


enum {	
	// msg type
	MSG_TYPE_REQ			= 0x01,
	MSG_TYPE_REP			= 0x02,
	MSG_TYPE_HELLO			= 0x03,
	MSG_TYPE_ASYNC			= 0x04,
};

enum {	
	// msg type
	ASYNC_MSG_JOINED		= 0x01,
	ASYNC_MSG_LED_DRIVER	= 0x02,
	ASYNC_MSG_SENT			= 0x03,
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

	CMD_GW_RELOAD_FW		= 0xE3,
	CMD_RF_AUTHENTICATE		= 0xE2,


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
	ERR_CMD_CRC_ERROR		= 0x09,
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
	uint8_t			connected;
	uint8_t			lost_connection_cnt;
	unsigned char	app_code[16];
	unsigned char	next_hop[16];
	uint16_t 		challenge_code;
	uint16_t 		challenge_code_res;
	uint8_t			authenticated;
};

/*---------------------------------------------------------------------------*/
//	sfd[1] 			= 0x7F (Start of Frame Delimitter)
//	len[1]: 		used for App node_id
//	seq[2]: 		transaction id;
//	type[1]: 		REQUEST/REPLY/HELLO
//	cmd[1]:			command id
//	err_code[2]: 	code returned in REPLY, sender check this field to know the REQ status
//	arg[54]: 		data payload
//	crc[2]			CRC check

struct cmd_struct_t {
	uint8_t  	sfd;
	uint8_t 	len;
	uint16_t 	seq;
	uint8_t		type;
	uint8_t		cmd;
	uint16_t	err_code;
	uint8_t 	arg[MAX_CMD_DATA_LEN];
	uint16_t	crc;
};

union float_byte_convert {
    float f;
    uint8_t bytes[4];
};

typedef struct cmd_struct_t		cmd_struct_t;
typedef struct net_struct_t		net_struct_t;
typedef struct gw_struct_t		gw_struct_t;
typedef struct led_struct_t		led_struct_t;

//void 		print_cmd_data(cmd_struct_t command);
uint16_t 	hash( uint16_t a); 
void		gen_crc_for_cmd(cmd_struct_t *cmd);
uint8_t 	check_crc_for_cmd(cmd_struct_t *cmd);
uint16_t 	gen_crc16(uint8_t *data_p, unsigned short  length);
void 		encrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv);
void 		decrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv);
void 		encrypt_payload(cmd_struct_t *cmd, uint8_t* key);
void 		decrypt_payload(cmd_struct_t *cmd, uint8_t* key);
void		float2Bytes(float val,uint8_t* bytes_array);
void 		phex_16(uint8_t* data_16);
void 		phex_64(uint8_t* data_64);
	
#endif /* SLS_H_ */
