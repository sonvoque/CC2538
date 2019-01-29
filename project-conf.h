/*
 * Copyright (c) 2015, Swedish Institute of Computer Science.
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
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#ifndef WITH_NON_STORING
#define WITH_NON_STORING 1 /* Set this to run with non-storing mode */
#endif /* WITH_NON_STORING */

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES

#ifdef TEST_MORE_ROUTES
/* configure number of neighbors and routes */
#define NBR_TABLE_CONF_MAX_NEIGHBORS     10
#define UIP_CONF_MAX_ROUTES   30
#else
/* configure number of neighbors and routes */
#define NBR_TABLE_CONF_MAX_NEIGHBORS     10
#define UIP_CONF_MAX_ROUTES   10
#endif /* TEST_MORE_ROUTES */


/* define RDC and MAC here */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     	nullmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     	nullrdc_driver 		//nullrdc_driver, cxmac_driver, contikimac_driver
#undef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_CONF_802154_AUTOACK       1

/* Define as minutes */
#define RPL_CONF_DEFAULT_LIFETIME_UNIT   60

/* 10 minutes lifetime of routes */
#define RPL_CONF_DEFAULT_LIFETIME        10

#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME 1

#if WITH_NON_STORING
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 		40 /* Number of links maintained at the root. Can be set to 0 at non-root nodes. */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES 		0 /* No need for routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NON_STORING /* Mode of operation*/
#endif /* WITH_NON_STORING */

/* son define */
#ifndef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER  			1
#endif

#define UIP_CONF_IPV6_RPL  			1
/* ND and Routing */
#define UIP_CONF_ND6_SEND_RA        0
#define UIP_CONF_IP_FORWARD         0


/* Low Power Mode */
#define LPM_CONF_ENABLE       		0		/**< Set to 0 to disable LPM entirely */
#define LPM_CONF_MAX_PM       		1


/* RF parameters define*/
#define CC2538_RF_CONF_CHANNEL    	26
#define CC2538_RF_CONF_TX_POWER		0xFF	// +7dBm
#define CC2538_RF_CONF_AUTOACK 		1 



#define DEBUG 1			//		DEBUG_NONE


#ifndef STARTUP_CONF_VERBOSE
#define STARTUP_CONF_VERBOSE        0 /**< Set to 0 to decrease startup verbosity */
#endif


/* using this prefix to work with 6lbr ONLY */
//#define UIP_CONF_DS6_DEFAULT_PREFIX 	0xaaaa


/* configuration of Link Layer Security */
/* NONCORESEC_CONF_SEC_LVL:
0x00 No security Data is not encrypted. Data authenticity is not validated.
0x01 AES-CBC-MAC-32 MIC-32 Data is not encrypted. Data authenticity is validated.
0x02 AES-CBC-MAC-64 MIC-64 Data is not encrypted. Data authenticity is validated.
0x03 AES-CBC-MAC-128 MIC-128 Data is not encrypted. Data authenticity is validated.
0x04 AES-CTR ENC Data is encrypted. Data authenticity is not validated.
0x05 AES-CCM-32 AES-CCM-32 Data is encrypted. Data authenticity is validated.
0x06 AES-CCM-64 AES-CCM-64 Data is encrypted. Data authenticity is validated.
0x07 AES-CCM-128 AES-CCM-128 Data is encrypted. Data authenticity is validated*/


/* Note: LLSEC can not used on Z1: error compilation */
#define SECURITY_EN		0

#if (SECURITY_EN)

/* software-based AES */
#if (!SLS_USING_HW)
#undef AES_128_CONF    
#define AES_128_CONF aes_128_driver
#endif 

#undef LLSEC802154_CONF_ENABLED
#define LLSEC802154_CONF_ENABLED          1

#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER              noncoresec_framer

#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC               noncoresec_driver

#undef NONCORESEC_CONF_SEC_LVL
#define NONCORESEC_CONF_SEC_LVL  5      
/* if using Level 7, it may cause delay in CC2538 and command not responding 
So optimum value: NONCORESEC_CONF_SEC_LVL=6 */

#define LLSEC_ANTIREPLAY_ENABLED 0 			/* disable anti-replay */
#define LLSEC_REBOOT_WORKAROUND_ENABLED 1
#define NONCORESEC_CONF_KEY { 0x00 , 0x01 , 0x02 , 0x03 , \
							  0x04 , 0x05 , 0x06 , 0x07 , \
							  0x08 , 0x09 , 0x0A , 0x0B , \
							  0x0C , 0x0D , 0x0E , 0x0F }

#endif /* SECURITY_EN */



#endif
