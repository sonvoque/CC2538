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


uint16_t 	hash( uint16_t a); 
void		gen_crc_for_cmd(cmd_struct_t *cmd);
uint8_t 	check_crc_for_cmd(cmd_struct_t *cmd);
uint16_t 	gen_crc16(uint8_t *data_p, unsigned short  length);
void 		encrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv);
void 		decrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv);
void 		encrypt_payload(cmd_struct_t *cmd, uint8_t* key);
void 		decrypt_payload(cmd_struct_t *cmd, uint8_t* key);